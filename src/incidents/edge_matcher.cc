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
                const double poff_meters = 0.,
                const double noff_meters = 0.,
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
        const auto actual_length =
            (static_cast<float>(end_cand.length) * end_cand.percent_along) - start_offset;
        if (actual_length <= 0.f || !is_within_limit(static_cast<uint32_t>(actual_length))) {
          continue;
        }

        openlr_edges.emplace_back(start_cand.graph_id);
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

      if (is_within_limit(
              static_cast<uint32_t>(route_nav_leg.summary().length() * vm::kMetersPerKm))) {

        // request trace_attributes and pull out all edge IDs
        valhalla::Api trace_req = get_trace_base_req();
        trace_req.mutable_options()->set_encoded_polyline(route_trip_leg.shape());
        actor.act(trace_req);
        const auto& trace_trip_leg = trace_req.trip().routes(0).legs(0);
        const auto& shape_pts = vm::decode<std::vector<vm::PointLL>>(trace_trip_leg.shape());

        // collect all the traversed edge IDs
        for (int i = 1; i < trace_trip_leg.node().size(); i++) {
          if (!trace_trip_leg.node(i - 1).has_edge()) {
            continue;
          }
          const auto& node = trace_trip_leg.node(i - 1);
          openlr_edges.emplace_back(vb::GraphId(node.edge().id()));
        }

        if (!(openlr_edges[0].edge_id == start_cand.graph_id)) {
          auto node = trace_trip_leg.node().begin();
          while (!(node->edge().length_km() > 0.001f)) {
            // _somehow_ the map matching can return irrelevant edges in the beginning
            // skip those until we find the right one
            openlr_edges.erase(openlr_edges.begin());
            node++;
            continue;
          }
        }

        // save the poff as distance in absolute meters
        // calculate the real offset from the start of the first edge to the openlr.poff
        auto poff_edge_offset = start_cand.length - (start_cand.length * start_cand.percent_along);
        auto start_edge_length = start_cand.length;
        // skip the first edge as long as the edge_offset is smaller than poff
        if (static_cast<double>(poff_edge_offset) < poff_meters) {
          auto node = trace_trip_leg.node().begin();
          node++; // increment to get to the next edge
          while (true) {
            // remove the previous edge's id from the vector, expensive but shouldn't be many elements
            // and a rather rare situation hopefully (skipping an entire edge)
            openlr_edges.erase(openlr_edges.begin());

            // if we're finally beyond the poff we know we found the right edge, so break
            poff_edge_offset += node->edge().length_km() * vm::kMetersPerKm;
            if (poff_edge_offset > poff_meters) {
              start_edge_length = node->edge().length_km() * vm::kMetersPerKm;
              break;
            }

            // if at this point there's no nodes left, that means we already looked at the
            // last segment just before, this shouldn't happen
            node++;
            if (node == trace_trip_leg.node().end()) {
              throw std::runtime_error("The OpenLR's poff was larger than the entire matched length");
            }
          }
        }
        // save the first_node_offset (as 1/255th of length)
        const auto first_edge_offset = poff_edge_offset - static_cast<float>(poff_meters);
        assert(first_edge_offset >= 0.f);
        openlr_edges.front().first_node_offset =
            static_cast<uint8_t>((start_edge_length - first_edge_offset) / start_edge_length * 255.f);

        // save the noff as distance in absolute meters
        // calculate the real offset from the end of the last edge to the openlr.noff
        // TODO: should we really take the /locate result here or rather the matched path's?
        auto noff_edge_offset = end_cand.length * end_cand.percent_along;
        auto end_edge_length = end_cand.length;
        // skip the last edge as long as the edge_offset is smaller than poff
        if (noff_edge_offset < noff_meters) {
          auto node = trace_trip_leg.node().rbegin();
          node++; // increment to get to the next last edge
          while (true) {
            // remove the previous edge's id from the vector, expensive but shouldn't be many elements
            // and a rather rare situation hopefully (skipping an entire edge)
            openlr_edges.pop_back();

            // if we're finally beyond the noff we know we found the right edge, so break
            noff_edge_offset += node->edge().length_km() * vm::kMetersPerKm;
            if (noff_edge_offset > noff_meters) {
              end_edge_length = node->edge().length_km() * vm::kMetersPerKm;
              break;
            }

            // if at this point there's no nodes left, that means we already looked at the
            // last segment just before, this shouldn't happen
            node++;
            if (node == trace_trip_leg.node().rend()) {
              throw std::runtime_error("The OpenLR's noff was larger than the entire matched length");
            }
          }
        }
        // save the last_node_offset (as 1/255th of length)
        const auto last_edge_offset = noff_edge_offset - static_cast<float>(noff_meters);
        assert(last_edge_offset >= 0.f);
        openlr_edges.back().last_node_offset =
            static_cast<uint8_t>(last_edge_offset / end_edge_length * 255.f);

        // quick sanity check
        assert((first_edge_offset + last_edge_offset) < static_cast<float>(a_lrp.distance));
        return;
      }
    }
  }
}

void match_edges(valhalla::tyr::actor_t& actor,
                 rapidjson::Value::ConstValueIterator openlr_start,
                 rapidjson::Value::ConstValueIterator openlr_end,
                 std::promise<std::vector<vi::OpenLrEdge>>& result) {

  auto remove_id_dups = [](std::vector<vi::OpenLrEdge>& edge_ids) {
    auto last = std::unique(edge_ids.begin(), edge_ids.end());
    edge_ids.erase(last, edge_ids.end());
  };

  // assume 20 edges per openlr segment
  std::vector<vi::OpenLrEdge> openlrs_edges;
  openlrs_edges.reserve((openlr_end - openlr_start) * 20);
  for (; openlr_start != openlr_end; openlr_start++) {
    const auto openlr = vb::OpenLR::OpenLr(openlr_start->GetString(), true);
    const auto segment_count = openlr.lrps.size() - 1U;
    if (segment_count > 1) {
      LOG_WARN("Received " + std::to_string(segment_count) + " segments");
    }

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
      double poff_meter = i == 0 ? (static_cast<double>(openlr.poff) / 255.) * a_lrp.distance : 0.;
      uint32_t noff_meter =
          i == (segment_count - 1) ? (static_cast<double>(openlr.noff) / 255.) * a_lrp.distance : 0.;

      // try without heading filter if the first try wasn't successful
      match_lrps(actor, local_edges, a_ranked, b_ranked, a_lrp, b_lrp, poff_meter, noff_meter, true);
      if (local_edges.size() == size_before) {
        LOG_WARN("1st match attempt didn't work for openlr " +
                 std::string(openlr_start->GetString()));
        match_lrps(actor, local_edges, a_ranked, b_ranked, a_lrp, b_lrp, poff_meter, noff_meter,
                   false);
      }

      // collect the ids; if there are none, it's an error
      if (local_edges.size() != size_before) {
        local_edges.shrink_to_fit();
        local_edges.back().is_last = true;
        remove_id_dups(local_edges);
        openlrs_edges.insert(openlrs_edges.end(), local_edges.begin(), local_edges.end());
        continue;
      }

      LOG_ERROR("2nd match attempt didn't work for openlr " + std::string(openlr_start->GetString()));
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