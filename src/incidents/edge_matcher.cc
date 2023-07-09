#include <future>
#include <optional>
#include <string>
#include <thread>

#include <valhalla/baldr/openlr.h>
#include <valhalla/incidents/utils.h>
#include <valhalla/incidents/worker.h>
#include <valhalla/proto_conversions.h>
#include <valhalla/tyr/actor.h>
#include <valhalla/tyr/serializers.h>

namespace vb = valhalla::baldr;
namespace vi = valhalla::incidents;
namespace vm = valhalla::midgard;
namespace vt = valhalla::tyr;

namespace {

enum class LRPOrder : bool { FIRST, LAST };

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
  base_request.mutable_options()->set_date_time("2023-03-03T09:00");
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
  // base_request.mutable_options()->set_format(valhalla::Options_Format_pbf);
  base_request.mutable_options()->set_costing_type(valhalla::Costing::auto_);
  base_request.mutable_options()->set_shape_match(valhalla::ShapeMatch::edge_walk);
  base_request.mutable_options()->set_filter_action(valhalla::FilterAction::include);
  base_request.mutable_options()->add_filter_attributes("edge.id");
  base_request.mutable_options()->add_filter_attributes("edge.length");
  base_request.mutable_options()->add_filter_attributes("shape");
  base_request.mutable_options()->add_filter_attributes("edge.begin_shape_index");
  base_request.mutable_options()->add_filter_attributes("edge.end_shape_index");

  return base_request;
}

std::vector<vi::RankEdge>
rank_edges(const std::optional<rapidjson::GenericArray<false, rapidjson::Value>>& edges,
           const vb::OpenLR::LocationReferencePoint& lrp,
           const LRPOrder lrp_order) {
  std::vector<vi::RankEdge> ranked_edges;

  const auto& lrp_rc = static_cast<valhalla::RoadClass>(lrp.frc);
  double lrp_bearing;
  for (const auto& edge : *edges) {
    // don't look at node-snapped edges which are obviously not part of it
    if (lrp_order == LRPOrder::FIRST) {
      if (!(edge["percent_along"].GetFloat() < 0.99999f))
        continue;
      lrp_bearing = lrp.bearing;
    } else {

      if (!(edge["percent_along"].GetFloat() > 0.00001f))
        continue;
      // normalize the heading to 0 <= heading <= 360
      lrp_bearing =
          (lrp.bearing + 180. > -360.) > 360. ? lrp.bearing + 180. - 360. : lrp.bearing + 180.;
    }
    valhalla::RoadClass osm_rc;
    valhalla::RoadClass_Enum_Parse(edge["edge"]["classification"]["classification"].GetString(),
                                   &osm_rc);

    ranked_edges.emplace_back(vb::GraphId(edge["edge_id"]["value"].GetUint64()),
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
    rank_edge.dist_diff = static_cast<uint16_t>(lrp_ll.Distance(osm_ll));

    // compute heading difference
    auto bearing_diff = std::fabs(edge["heading"].GetDouble() - lrp_bearing);
    rank_edge.heading_diff = bearing_diff > 360.0001 ? bearing_diff - 360. : bearing_diff;
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
 * @param openlr_edges fill in the vector of edge ids
 * @param a_ranked the start LRP's vector of ranked edges by /locate
 * @param b_ranked the destination LRP's vector of ranked edges by /locate
 * @param a_lrp the start LRP object
 * @param b_lrp the destination LRP object
 * @param is_last_segment then we need to flip the bearing (if used)
 * @param use_heading whether or not to use the bearing as fallback (it's more complex)
 *
 */
void match_lrps(valhalla::tyr::actor_t& actor,
                std::vector<vi::OpenLrEdge>& openlr_edges,
                const std::vector<vi::RankEdge>& a_ranked,
                const std::vector<vi::RankEdge>& b_ranked,
                const vb::OpenLR::LocationReferencePoint& a_lrp,
                const vb::OpenLR::LocationReferencePoint& b_lrp,
                const bool use_heading = false) {
  auto is_within_limit = [&a_lrp](uint32_t route_dist) -> bool {
    const auto limit = a_lrp.distance < 1000.0 ? 60 : 100;
    return std::abs(static_cast<int>(a_lrp.distance - static_cast<int>(route_dist))) <= limit;
  };

  for (auto a = 0U; a < a_ranked.size(); a++) {
    const auto& start_cand = a_ranked[a];
    for (auto b = 0U; b < b_ranked.size(); b++) {
      const auto& end_cand = b_ranked[b];

      // trivial, so we can return with that single ID
      if (start_cand.graph_id == end_cand.graph_id) {
        const auto start_offset = static_cast<float>(start_cand.length) * start_cand.percent_along;
        // same edge, so percent_along is in route direction
        const auto end_offset = static_cast<float>(end_cand.length) * end_cand.percent_along;
        const auto actual_length = end_offset - start_offset;
        if (actual_length <= 0.f || !is_within_limit(static_cast<uint32_t>(start_cand.length))) {
          continue;
        }

        openlr_edges.emplace_back(start_cand.graph_id, start_cand.length,
                                  static_cast<uint8_t>(start_cand.percent_along * 255.f),
                                  static_cast<uint8_t>(end_cand.percent_along * 255.f));
        return;
      }

      const auto start_cand_ll = vm::PointLL{start_cand.corr_lon, start_cand.corr_lat};
      const auto end_cand_ll = vm::PointLL{end_cand.corr_lon, end_cand.corr_lat};

      // make the route request
      valhalla::Api route_req = get_route_base_req();
      vi::get_route_req(route_req,
                        std::make_pair(start_cand_ll, static_cast<uint32_t>(a_lrp.bearing)),
                        std::make_pair(end_cand_ll, static_cast<uint32_t>(b_lrp.bearing)),
                        use_heading);
      actor.act(route_req);

      // see if it we were matching the length difference and return the edge IDs accordingly
      // reminder: offsets don't play a role when comparing distances, so LRPs are fine!
      const auto& route_nav_leg = route_req.directions().routes(0).legs(0);
      const auto& route_trip_leg = route_req.trip().routes(0).legs(0);

      vi::print_route(route_trip_leg);

      if (is_within_limit(
              static_cast<uint32_t>(route_nav_leg.summary().length() * vm::kMetersPerKm))) {

        // request trace_attributes and pull out all edge IDs
        valhalla::Api trace_req = get_trace_base_req();
        trace_req.mutable_options()->set_encoded_polyline(route_trip_leg.shape());
        actor.act(trace_req);
        const auto& trace_trip_leg = trace_req.trip().routes(0).legs(0);
        const auto& shape_pts = vm::decode<std::vector<vm::PointLL>>(trace_trip_leg.shape());

        vi::print_route(trace_trip_leg);

        // collect all the traversed edge IDs
        for (int i = 1; i < trace_trip_leg.node().size(); i++) {
          const auto& node = trace_trip_leg.node(i - 1);
          if (!node.has_edge()) {
            continue;
          }
          // first & last edge can have partial length_km
          float matched_length = node.edge().length_km();
          float total_length = node.edge().total_length_km();
          if (matched_length != total_length) {
            // if this happens it has got to be the first or last edge
            // else smth is really wrong with map matching
            // this only happens for TomTom/OSM combo
            if (!(i == 1 || i == (trace_trip_leg.node().size() - 1))) {
              throw std::runtime_error("WTF: Edge 'index' " + std::to_string(i) + " had an offset!");
            }
          }
          auto default_offset = static_cast<uint8_t>(matched_length / total_length * 255.f);
          openlr_edges.emplace_back(vb::GraphId(node.edge().id()), total_length * vm::kMetersPerKm,
                                    default_offset, default_offset);
        }

        // _somehow_ the map matching can return irrelevant edges in the beginning
        // TODO: make it reproduceable and file a bug!
        // Whole logic here assumes that only happens at node snaps
        // skip those until we find the right one
        if (!(openlr_edges[0].edge_id == start_cand.graph_id)) {
          auto node = trace_trip_leg.node().begin();
          while (!(node->edge().length_km() > 0.001f)) {
            openlr_edges.erase(openlr_edges.begin());
            node++;
            continue;
          }
        }
      }
    }
  }
}

void match_edges(valhalla::tyr::actor_t& actor,
                 rapidjson::Value::ConstValueIterator openlr_start,
                 rapidjson::Value::ConstValueIterator openlr_end,
                 std::promise<std::vector<vi::OpenLrEdge>>& result) {

  // assume 20 edges per openlr segment
  std::vector<vi::OpenLrEdge> openlrs_edges;
  openlrs_edges.reserve((openlr_end - openlr_start) * 20);
  for (; openlr_start != openlr_end; openlr_start++) {
    const auto openlr = vb::OpenLR::OpenLr(openlr_start->GetString(), true);
    const auto segment_count = openlr.lrps.size() - 1U;
    if (segment_count > 1) {
      LOG_WARN("Received " + std::to_string(segment_count) + " segments");
    }

    // get the poff/noff; NOTE: this has a 255th resolution
    auto poff_meter =
        static_cast<float>(openlr.getLength() * (static_cast<double>(openlr.poff) / 255.));
    auto noff_meter =
        static_cast<float>(openlr.getLength() * (static_cast<double>(openlr.noff) / 255.));

    std::vector<vi::OpenLrEdge> local_edges;
    local_edges.reserve(20);
    for (size_t i = 0; i < segment_count; i++) {
      // first the origin
      const auto a_lrp = openlr.lrps[i];
      valhalla::Api a_locate_req = get_locate_base_req();
      vi::get_locate_req(a_locate_req, a_lrp, false);
      rapidjson::Document a_res;
      a_res.Parse(actor.act(a_locate_req));
      const auto a_ranked =
          rank_edges(a_res.GetArray()[0]["edges"].GetArray(), a_lrp, LRPOrder::FIRST);

      // then the destination
      const auto b_lrp = openlr.lrps[i + 1];
      valhalla::Api b_locate_req = get_locate_base_req();
      vi::get_locate_req(b_locate_req, b_lrp, true);
      rapidjson::Document b_res;
      b_res.Parse(actor.act(b_locate_req));
      const auto b_ranked =
          rank_edges(b_res.GetArray()[0]["edges"].GetArray(), b_lrp, LRPOrder::LAST);

      const auto size_before = local_edges.size();

      // try without heading filter if the first try wasn't successful
      match_lrps(actor, local_edges, a_ranked, b_ranked, a_lrp, b_lrp, true);
      if (local_edges.size() == size_before) {
        LOG_WARN("1st match attempt didn't work for openlr " +
                 std::string(openlr_start->GetString()));
        match_lrps(actor, local_edges, a_ranked, b_ranked, a_lrp, b_lrp, false);
      }

      // collect the ids; if there are none, it's an error
      if (local_edges.size() == size_before) {
        LOG_ERROR("2nd match attempt didn't work for openlr " +
                  std::string(openlr_start->GetString()));
      }
    }

    // remove duplicates
    auto last = std::unique(local_edges.begin(), local_edges.end());
    local_edges.erase(last, local_edges.end());

    // deal with poff/noff; for first/last edge we already took note above in case the LRPs
    // are not snapped to nodes
    auto first_edge = local_edges.begin();
    auto first_edge_poff =
        static_cast<float>(first_edge->first_node_offset) / 255.f * first_edge->length;
    if (first_edge_poff < poff_meter) {
      first_edge++;
      uint32_t delete_count = 0;
      // we're skipping entire edges at the front until we're past the poff
      while (true) {
        delete_count++;
        first_edge_poff += first_edge->length;
        if (first_edge_poff > poff_meter) {
          break;
        }
        first_edge++;
        // if at this point there's no nodes left, that means we already looked at the
        // last segment just before, this shouldn't happen
        if (first_edge == local_edges.end()) {
          throw std::runtime_error("The OpenLR's poff was larger than the entire matched length");
        }
      }
      // remove the skipped edges, invalidates first_edge
      local_edges.erase(local_edges.begin(), local_edges.begin() + delete_count);
    }
    local_edges.front().first_node_offset = static_cast<uint8_t>(
        ((local_edges.front().length - (first_edge_poff - poff_meter)) / local_edges.front().length) *
        255.f);

    auto last_edge = local_edges.rbegin();
    auto last_edge_noff = static_cast<float>(last_edge->last_node_offset) / 255.f * last_edge->length;
    if (last_edge_noff < noff_meter) {
      last_edge++;
      uint32_t delete_count = 0;
      // we're skipping entire edges at the front until we're past the poff
      while (true) {
        delete_count++;
        last_edge_noff += last_edge->length;
        if (last_edge_noff > noff_meter) {
          break;
        }
        last_edge++;
        // if at this point there's no nodes left, that means we already looked at the
        // last segment just before, this shouldn't happen
        if (last_edge == local_edges.rend()) {
          throw std::runtime_error("The OpenLR's noff was larger than the entire matched length");
        }
      }
      // remove the skipped edges
      for (uint32_t i = 0; i < delete_count; i++) {
        local_edges.pop_back();
      }
    }
    local_edges.back().last_node_offset =
        static_cast<uint8_t>(((last_edge_noff - noff_meter) / local_edges.back().length) * 255.f);

    // insert into the outer vector
    std::move(local_edges.begin(), local_edges.end(), std::back_inserter(openlrs_edges));

    if (openlrs_edges.size()) {
      openlrs_edges.back().is_last = true;
    }
  }

  openlrs_edges.shrink_to_fit();
  result.set_value(std::move(openlrs_edges));
}
} // namespace

namespace valhalla {
namespace incidents {

// matches the OpenLR entries to a list of edge Graph IDs
std::vector<OpenLrEdge> incident_worker_t::get_matched_edges(const rapidjson::Document& req_doc) {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);

  // Holds a vector of GraphId's for each openlr segment
  std::vector<std::promise<std::vector<vi::OpenLrEdge>>> results(threads.size());

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
    // Where the range ends
    std::advance(openlr_end, openlr_count);
    // Make the thread
    threads[i].reset(new std::thread(match_edges, std::ref(actors[i]), openlr_start, openlr_end,
                                     std::ref(results[i])));
  }

  // Join all the threads to wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  std::vector<OpenLrEdge> openlrs_edges;
  for (auto& promise : results) {
    try {
      const auto& ids = promise.get_future().get();
      openlrs_edges.insert(openlrs_edges.end(), ids.begin(), ids.end());
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  return openlrs_edges;
}

} // namespace incidents
} // namespace valhalla