#include <future>
#include <optional>
#include <string>
#include <thread>

#include <valhalla/baldr/openlr.h>
#include <valhalla/incidents/utils.h>
#include <valhalla/incidents/worker.h>
#include <valhalla/proto_conversions.h>
#include <valhalla/tyr/actor.h>

namespace vb = valhalla::baldr;
namespace vi = valhalla::incidents;
namespace vm = valhalla::midgard;
namespace vt = valhalla::tyr;

namespace {

static valhalla::Api get_locate_base_req() {
  valhalla::Api base_request;
  base_request.mutable_options()->set_action(valhalla::Options::locate);
  base_request.mutable_options()->set_costing_type(valhalla::Costing::auto_);
  base_request.mutable_options()->set_verbose(true);

  return base_request;
}

static valhalla::Api get_route_base_req() {
  valhalla::Api base_request;
  base_request.mutable_options()->set_action(valhalla::Options::route);
  base_request.mutable_options()->set_costing_type(valhalla::Costing::auto_);
  // no narrative
  base_request.mutable_options()->set_directions_type(valhalla::DirectionsType::none);
  // force a unidirectional astar for performance reasons
  base_request.mutable_options()->set_date_time_type(static_cast<valhalla::Options::DateTimeType>(1));
  // set to shortest as per openlr requirement and no maneuver penalty
  valhalla::Costing* costing =
      &(*base_request.mutable_options()->mutable_costings())[valhalla::Costing::auto_];
  costing->mutable_options()->set_shortest(true);
  costing->mutable_options()->set_maneuver_penalty(0.f);

  return base_request;
}

static valhalla::Api get_trace_base_req() {
  valhalla::Api base_request;
  base_request.mutable_options()->set_action(valhalla::Options::trace_attributes);
  base_request.mutable_options()->set_costing_type(valhalla::Costing::auto_);
  base_request.mutable_options()->set_shape_match(valhalla::ShapeMatch::walk_or_snap);
  base_request.mutable_options()->set_filter_action(valhalla::FilterAction::include);
  base_request.mutable_options()->add_filter_attributes("edge.id");
  valhalla::Costing* costing =
      &(*base_request.mutable_options()->mutable_costings())[valhalla::Costing::auto_];
  costing->mutable_options()->set_shortest(true);
  costing->mutable_options()->set_maneuver_penalty(0.f);

  return base_request;
}

std::vector<vi::RankEdge>
rank_edges(const std::optional<rapidjson::GenericArray<false, rapidjson::Value>>& edges,
           const vb::OpenLR::LocationReferencePoint& lrp) {
  std::vector<vi::RankEdge> ranked_edges;

  const auto& lrp_rc = static_cast<valhalla::RoadClass>(lrp.frc);
  for (const auto& edge : *edges) {
    valhalla::RoadClass osm_rc;
    valhalla::RoadClass_Enum_Parse(edge["edge"]["classification"]["classification"].GetString(),
                                   &osm_rc);

    ranked_edges.emplace_back(vb::GraphId(edge["edge_id"]["id"].GetUint64()),
                              edge["edge"]["geo_attributes"]["length"].GetUint(),
                              edge["percent_along"].GetFloat(), edge["correlated_lon"].GetDouble(),
                              edge["correlated_lat"].GetDouble());
    auto& rank_edge = ranked_edges.back();

    // TODO: test if this makes sense!
    if (osm_rc == lrp_rc) {
      // if same FRC
      rank_edge.frc_diff = 1;
    } else if (std::abs(static_cast<int>(osm_rc) - static_cast<int>(lrp_rc)) == 1) {
      // if 1 FRC difference
      rank_edge.frc_diff = 2;
    } else {
      rank_edge.frc_diff = 3;
    }

    // compute distance difference
    vm::PointLL lrp_ll = vm::PointLL(lrp.longitude, lrp.latitude);
    vm::PointLL osm_ll =
        vm::PointLL(edge["correlated_lon"].GetDouble(), edge["correlated_lat"].GetDouble());
    rank_edge.dist_diff = lrp_ll.Distance(osm_ll);

    // compute heading difference
    // TODO: for the last segment we'll need to flip the bearing!
    rank_edge.heading_diff = std::abs(edge["heading"].GetFloat() - static_cast<float>(lrp.bearing));
  }

  std::sort(ranked_edges.begin(), ranked_edges.end(),
            [](const vi::RankEdge& lhs, const vi::RankEdge& rhs) {
              if (lhs.frc_diff == rhs.frc_diff) {
                if (lhs.dist_diff == rhs.frc_diff) {
                  return lhs.heading_diff < rhs.heading_diff;
                } else {
                  return lhs.dist_diff < rhs.dist_diff;
                }
              } else {
                return lhs.frc_diff < rhs.frc_diff;
              }
            });

  return ranked_edges;
}

/**
 * Matches a segment between LRP pairs to the road network and returns the
 * IDs of the edges
 *
 * @param actor the Actor instance to do the map matching
 * @param a_ranked the start LRP's vector of ranked edges by /locate
 * @param b_ranked the destination LRP's vector of ranked edges by /locate
 * @param a_lrp the start LRP object
 * @param b_lrp the destination LRP object
 * @param is_last_segment then we need to flip the bearing (if used)
 * @param use_heading whether or not to use the bearing as fallback (it's more complex)
 *
 */
std::vector<vb::GraphId> match_lrps(valhalla::tyr::actor_t& actor,
                                    const std::vector<vi::RankEdge>& a_ranked,
                                    const std::vector<vi::RankEdge>& b_ranked,
                                    const vb::OpenLR::LocationReferencePoint& a_lrp,
                                    const vb::OpenLR::LocationReferencePoint& b_lrp,
                                    const bool is_last_segment,
                                    const bool use_heading) {
  auto is_within_limit = [&a_lrp](uint32_t route_dist) -> bool {
    const auto limit = a_lrp.distance < 1000.0 ? 60 : 100;
    return std::abs(static_cast<int>(a_lrp.distance - static_cast<int>(route_dist))) < limit;
  };

  std::vector<vb::GraphId> edge_ids;
  for (auto a = 0U; a < a_ranked.size(); a++) {
    const auto& start_cand = a_ranked[a];
    for (auto b = 0U; b < b_ranked.size(); b++) {
      const auto& end_cand = b_ranked[b];

      // trivial, so we can return that single ID
      if (!use_heading && start_cand.graph_id == end_cand.graph_id) {
        const auto start_offset = static_cast<float>(start_cand.length) * start_cand.percent_along;
        const auto actual_length =
            (static_cast<float>(end_cand.length) * end_cand.percent_along) - start_offset;
        if (actual_length <= 0.f || !is_within_limit(static_cast<uint32_t>(actual_length))) {
          return {};
        }

        return {start_cand.graph_id};
      }

      // make the route request
      valhalla::Api route_req = get_route_base_req();
      vi::get_route_req(route_req,
                        std::make_pair(vm::PointLL{start_cand.corr_lon, start_cand.corr_lat},
                                       static_cast<uint32_t>(a_lrp.bearing)),
                        std::make_pair(vm::PointLL{end_cand.corr_lon, end_cand.corr_lat},
                                       static_cast<uint32_t>(b_lrp.bearing)),
                        is_last_segment, use_heading);
      actor.act(route_req);

      // see if it we were matching the length difference and return the edge IDs accordingly
      const auto& directions_leg = route_req.directions().routes(0).legs(0);
      if (is_within_limit(
              static_cast<uint32_t>(directions_leg.summary().length() * vm::kMetersPerKm))) {
        // request trace_attributes and pull out all edge IDs
        valhalla::Api trace_req = get_trace_base_req();
        trace_req.mutable_options()->set_encoded_polyline(directions_leg.shape());
        actor.act(trace_req);
        const auto& trip_leg = trace_req.trip().routes(0).legs(0);
        for (int i = 1; i < trip_leg.node().size(); i++) {
          edge_ids.emplace_back(vb::GraphId(trip_leg.node(i - 1).edge().id()));
        }

        return edge_ids;
      }
    }
  }

  return {};
}

void match_edges(valhalla::tyr::actor_t& actor,
                 rapidjson::Value::ConstValueIterator openlr_start,
                 rapidjson::Value::ConstValueIterator openlr_end,
                 std::promise<std::vector<std::vector<vb::GraphId>>>& result) {

  auto remove_id_dups = [](std::vector<vb::GraphId>& edge_ids) {
    auto last = std::unique(edge_ids.begin(), edge_ids.end());
    edge_ids.erase(last, edge_ids.end());
  };

  std::vector<std::vector<vb::GraphId>> openlr_edge_ids(openlr_end - openlr_start);
  for (; openlr_start != openlr_end; openlr_start++) {
    auto openlr = vb::OpenLR::OpenLr(openlr_start->GetString(), true);

    // TODO: it happens frequently that offsets are >> 1
    // need to respect that for the first and last segment
    if (static_cast<uint32_t>(openlr.noff) > 1 || static_cast<uint32_t>(openlr.poff) > 1) {
      LOG_ERROR("openLR offsets are more than 1/256th of the path");
    }

    const auto segment_count = openlr.lrps.size() - 1ULL;
    if (segment_count > 1) {
      LOG_WARN("Received " + std::to_string(segment_count) + " segments");
    }

    std::optional<rapidjson::GenericArray<false, rapidjson::Value>> b_edges;
    std::optional<std::vector<vi::RankEdge>> b_ranked;
    for (size_t i = 0; i < segment_count; i++) {
      const auto is_last_segment = i == openlr.lrps.size() - 2;

      // first the origin
      const auto a_lrp = openlr.lrps[i];
      valhalla::Api a_locate_req = get_locate_base_req();
      vi::get_locate_req(a_locate_req, a_lrp, false);
      // slight optimization: take the last dest lrp's edges here if it exists (all but first segment)
      rapidjson::Document a_res;
      a_res.Parse(actor.act(a_locate_req));
      const auto a_edges = b_edges.has_value() ? *b_edges : a_res.GetArray()[0]["edges"].GetArray();
      const auto a_ranked = b_ranked.has_value() ? *b_ranked : rank_edges(a_edges, a_lrp);

      // then the destination
      const auto b_lrp = openlr.lrps[i + 1];
      valhalla::Api b_locate_req = get_locate_base_req();
      vi::get_locate_req(b_locate_req, b_lrp, is_last_segment);
      rapidjson::Document b_res;
      b_res.Parse(actor.act(b_locate_req));
      b_edges = std::make_optional(b_res.GetArray()[0]["edges"].GetArray());
      b_ranked = std::make_optional(rank_edges(b_edges, b_lrp));

      // try with heading if the first try wasn't successful
      auto edge_ids = match_lrps(actor, a_ranked, *b_ranked, a_lrp, b_lrp, is_last_segment, false);
      if (!edge_ids.size()) {
        LOG_WARN("1st match attempt didn't work for openlr " +
                 std::string(openlr_start->GetString()));
        edge_ids = match_lrps(actor, a_ranked, *b_ranked, a_lrp, b_lrp, is_last_segment, true);
      }

      // collect the ids; if there are none, it's an error
      if (edge_ids.size()) {
        remove_id_dups(edge_ids);
        openlr_edge_ids.emplace_back(std::move(edge_ids));
        continue;
      }

      LOG_ERROR("2nd match attempt didn't work for openlr " + std::string(openlr_start->GetString()));
    }
  }
  result.set_value(openlr_edge_ids);
}

// auto a_i_limit = a_ranked.size() - 1ULL;
// auto b_i_limit = b_ranked.size() - 1ULL;
// while (true) {

// do the map matching and return if we're within 60 m shape length

// we go through the 2 vectors by comparing in a way of
// a0 + b0
// a1 + b0
// a0 + b1
// a1 + b1 etc
// if (a_i == a_i_limit && b_i == b_i_limit) {
//   // if we got here, we can't find a path between any combination of a & b
//   return {};
// } else if (a_i == a_i_limit) {
//   // if a is done
//   b_i++;
// } else if (b_i == b_i_limit) {
//   // if b is done
//   a_i++;
// } else if (a_i == b_i) {
//   // first increment a -> a1 + b0
//   a_i++;
// } else if (a_i == (b_i + 1U)) {
//   // if a is 1 further, decrement a and increment b -> a0 + b1
//   b_i++;
//   a_i--;
// } else if (b_i == (a_i + 1U)) {
//   // if b is 1 further, increment a -> a1 + b1
//   a_i++;
// }
// }
} // namespace

namespace valhalla {
namespace incidents {

// matches the OpenLR entries to a list of edge Graph IDs
std::vector<std::vector<vb::GraphId>>
incident_worker_t::get_matched_edges(const rapidjson::Document& req_doc) {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);

  // Holds a vector of GraphId's for each openlr segment
  std::vector<std::promise<std::vector<std::vector<vb::GraphId>>>> results(threads.size());

  // Divvy up the work
  const auto& openlrs_enc = req_doc.GetArray();
  size_t floor = openlrs_enc.Size() / threads.size();
  size_t at_ceiling = openlrs_enc.Size() - (threads.size() * floor);

  rapidjson::Value::ConstValueIterator openlr_start = openlrs_enc.Begin();
  rapidjson::Value::ConstValueIterator openlr_end = openlrs_enc.Begin();

  LOG_INFO("Received " + std::to_string(openlrs_enc.Size()) + " openLR strings to correlate");

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t openlr_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    openlr_start = openlr_end;
    // Where the range end
    std::advance(openlr_end, openlr_count);
    // Make the thread
    threads[i].reset(new std::thread(match_edges, std::ref(actors[i]), openlr_start, openlr_end,
                                     std::ref(results[i])));
  }

  // Join all the threads to wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  std::vector<std::vector<vb::GraphId>> openlr_graphids(openlrs_enc.Size());
  for (auto& promise : results) {
    try {
      const auto& ids = promise.get_future().get();
      openlr_graphids.insert(openlr_graphids.end(), ids.begin(), ids.end());
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  return openlr_graphids;
}

} // namespace incidents
} // namespace valhalla