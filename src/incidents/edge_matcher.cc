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
  costing->mutable_options()->set_ignore_closures(true);

  return base_request;
}

static valhalla::Api get_trace_base_req() {
  valhalla::Api base_request;
  base_request.mutable_options()->set_action(valhalla::Options::trace_attributes);
  base_request.mutable_options()->set_costing_type(valhalla::Costing::auto_);
  // base_request.mutable_options()->set_format(valhalla::Options_Format_pbf);
  base_request.mutable_options()->set_shape_match(valhalla::ShapeMatch::edge_walk);
  base_request.mutable_options()->set_filter_action(valhalla::FilterAction::include);
  base_request.mutable_options()->add_filter_attributes("edge.id");
  base_request.mutable_options()->add_filter_attributes("edge.length");
  base_request.mutable_options()->add_filter_attributes("shape");
  base_request.mutable_options()->add_filter_attributes("edge.begin_shape_index");
  base_request.mutable_options()->add_filter_attributes("edge.end_shape_index");
  valhalla::Costing* costing =
      &(*base_request.mutable_options()->mutable_costings())[valhalla::Costing::auto_];
  costing->mutable_options()->set_shortest(true);
  costing->mutable_options()->set_ignore_closures(true);

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
float match_lrps(valhalla::tyr::actor_t& actor,
                 std::vector<vi::OpenLrEdge>& openlr_edges,
                 const std::vector<vi::RankEdge>& a_ranked,
                 const std::vector<vi::RankEdge>& b_ranked,
                 const vb::OpenLR::LocationReferencePoint& a_lrp,
                 const vb::OpenLR::LocationReferencePoint& b_lrp,
                 const std::string& openlr_string,
                 const bool use_heading = false) {
  auto is_within_limit = [&a_lrp](float route_dist) -> bool {
    const auto limit = a_lrp.distance < 1000.0 ? 60 : 100;
    return std::abs(static_cast<int>(a_lrp.distance - static_cast<double>(route_dist))) <= limit;
  };

  auto min_encountered_dist = std::numeric_limits<float>::max();

  for (auto a = 0U; a < a_ranked.size(); a++) {
    const auto& start_cand = a_ranked[a];
    for (auto b = 0U; b < b_ranked.size(); b++) {
      const auto& end_cand = b_ranked[b];

      // trivial, so we can return with that single ID
      if (start_cand.graph_id == end_cand.graph_id) {
        const auto start_offset = start_cand.length * start_cand.percent_along;
        // same edge, so percent_along is in route direction
        const auto end_offset = end_cand.length * end_cand.percent_along;
        const auto actual_length = end_offset - start_offset;
        if (actual_length <= 0.f || !is_within_limit(start_cand.length)) {
          continue;
        }

        openlr_edges.emplace_back(start_cand.graph_id, start_cand.length, start_offset,
                                  end_cand.length - end_offset);
        return actual_length;
      }

      const auto start_cand_ll = vm::PointLL{start_cand.corr_lon, start_cand.corr_lat};
      const auto end_cand_ll = vm::PointLL{end_cand.corr_lon, end_cand.corr_lat};

      // make the route request
      valhalla::Api route_req = get_route_base_req();
      vi::get_route_req(route_req,
                        std::make_pair(start_cand_ll, static_cast<uint32_t>(a_lrp.bearing)),
                        std::make_pair(end_cand_ll, static_cast<uint32_t>(b_lrp.bearing)),
                        use_heading);
      try {
        actor.act(route_req);
      } catch (valhalla::valhalla_exception_t& e) {
        LOG_ERROR("[valhalla] " + openlr_string + " : routing failed with " + std::string(e.what()));
        continue;
      }

      // see if it we were matching the length difference and return the edge IDs accordingly
      // reminder: offsets don't play a role when comparing distances, so LRPs are fine!
      const auto& route_nav_leg = route_req.directions().routes(0).legs(0);
      const auto& route_trip_leg = route_req.trip().routes(0).legs(0);

      // vi::print_route(route_trip_leg);

      // record the minimum calculated distance so we know approx what was going on
      auto route_dist = route_nav_leg.summary().length() * vm::kMetersPerKm;
      min_encountered_dist = std::min(min_encountered_dist, route_dist);

      if (is_within_limit(route_dist)) {

        // request trace_attributes and pull out all edge IDs
        valhalla::Api trace_req = get_trace_base_req();
        trace_req.mutable_options()->set_encoded_polyline(route_trip_leg.shape());
        try {
          actor.act(trace_req);
        } catch (valhalla::valhalla_exception_t& e) {
          LOG_ERROR("[valhalla] " + openlr_string + " : Map Matching failed with " +
                    std::string(e.what()));
          continue;
        }
        const auto& trace_trip_leg = trace_req.trip().routes(0).legs(0);
        const auto& shape_pts = vm::decode<std::vector<vm::PointLL>>(trace_trip_leg.shape());

        // vi::print_route(trace_trip_leg);

        // collect all the traversed edge IDs
        std::vector<float> matched_lengths;
        for (int i = 1; i < trace_trip_leg.node().size(); i++) {
          const auto& node = trace_trip_leg.node(i - 1);
          if (!node.has_edge()) {
            continue;
          } else if (node.edge().length_km() < 0.001f) {
            // https://github.com/valhalla/valhalla/issues/4193
            // Whole logic here assumes that only happens at node snaps
            // skip those until we find the right one
            continue;
          }
          // first & last edge can have partial length_km
          float matched_length = node.edge().length_km();
          float total_length = node.edge().total_length_km();
          if (matched_length != total_length) {
            // if this happens it has got to be the first or last edge
            // else smth is really wrong with map matching
            // this should only happen for TomTom/OSM combo
            if (!(i == 1 || i == (trace_trip_leg.node().size() - 1))) {
              LOG_ERROR("[valhalla] " + openlr_string + " : WTF: Edge 'index' " + std::to_string(i) +
                        " had an offset!");
            }
          }

          matched_lengths.push_back(matched_length * vm::kMetersPerKm);
          openlr_edges.emplace_back(vb::GraphId(node.edge().id()), total_length * vm::kMetersPerKm,
                                    0.f, 0.f);
        }

        // now that we have the list of edges, handle first & last edges' offsets properly
        auto& first_edge = openlr_edges.front();
        first_edge.poff_start_offset = first_edge.length - matched_lengths.front();
        first_edge.noff_start_offset = 0.f;

        auto& last_edge = openlr_edges.back();
        last_edge.noff_start_offset = last_edge.length - matched_lengths.back();
        last_edge.poff_start_offset = 0.f;

        // if we made it this far and there's still edges left, we have succeeded
        if (openlr_edges.size() == 1) {
          LOG_WARN("[valhalla] " + openlr_string +
                   " : Detected trivial route which wasn't caught by the trivial route handler!");
        }
        if (openlr_edges.size()) {
          return route_dist;
        }
        LOG_WARN("[valhalla] " + openlr_string +
                 " : matching succeeded, but removing irrelevant edges removed all edges.");
      }
    }
  }

  return min_encountered_dist;
}

void match_edges(valhalla::tyr::actor_t& actor,
                 rapidjson::Value::ConstValueIterator openlr_start,
                 rapidjson::Value::ConstValueIterator openlr_end,
                 std::promise<std::vector<vi::OpenLrEdge>>& result) {

  // assume 20 edges per openlr segment
  std::vector<vi::OpenLrEdge> openlrs_edges;
  openlrs_edges.reserve((openlr_end - openlr_start) * 20);
  for (; openlr_start != openlr_end; openlr_start++) {

    // wrap this into a huge try/except block, and log out errors
    const auto openlr = vb::OpenLR::OpenLr(openlr_start->GetString(), true);
    const auto segment_count = openlr.lrps.size() - 1U;

    // get the poff/noff; NOTE: this has a 255th resolution
    auto poff_meter =
        static_cast<float>(openlr.getLength() * (static_cast<double>(openlr.poff) / 255.));
    auto noff_meter =
        static_cast<float>(openlr.getLength() * (static_cast<double>(openlr.noff) / 255.));

    // sanity check the poff/noff, apparently they can sum up to more than the entire length
    // which is bullshit and TomTom's fault
    if (poff_meter + noff_meter > openlr.getLength()) {
      LOG_ERROR("[TomTom] " + std::string(openlr_start->GetString()) +
                " : poff + noff is larger than the openLR length");
      continue;
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
      if (a_res.GetArray()[0]["edges"].IsNull()) {
        LOG_ERROR("[valhalla] " + std::string(openlr_start->GetString()) +
                  " : No edges found for 1st LRP of segment " + std::to_string(i));
        continue;
      }
      const auto a_ranked =
          rank_edges(a_res.GetArray()[0]["edges"].GetArray(), a_lrp, LRPOrder::FIRST);

      // then the destination
      const auto b_lrp = openlr.lrps[i + 1];
      valhalla::Api b_locate_req = get_locate_base_req();
      vi::get_locate_req(b_locate_req, b_lrp, true);
      rapidjson::Document b_res;
      b_res.Parse(actor.act(b_locate_req));
      if (b_res.GetArray()[0]["edges"].IsNull()) {
        LOG_ERROR("[valhalla] " + std::string(openlr_start->GetString()) +
                  " : No edges found for 2nd LRP of segment " + std::to_string(i));
        continue;
      }
      const auto b_ranked =
          rank_edges(b_res.GetArray()[0]["edges"].GetArray(), b_lrp, LRPOrder::LAST);

      if (!a_ranked.size() || !b_ranked.size()) {
        LOG_ERROR("[valhalla] " + std::string(openlr_start->GetString()) +
                  " : No suitable edges after ranking for 1st or 2nd LRP of segment " +
                  std::to_string(i));
        continue;
      }

      const auto size_before = local_edges.size();

      // try without heading filter if the first try wasn't successful
      auto min_dist = match_lrps(actor, local_edges, a_ranked, b_ranked, a_lrp, b_lrp,
                                 openlr_start->GetString(), true);
      if (local_edges.size() == size_before) {
        auto dist_diff = std::fabs(a_lrp.distance - static_cast<double>(min_dist));
        LOG_WARN("[valhalla] " + std::string(openlr_start->GetString()) +
                 " : 1st match attempt didn't work with a min distance diff of " +
                 std::to_string(dist_diff));

        // 2nd try, if not successful, it's an error
        min_dist = match_lrps(actor, local_edges, a_ranked, b_ranked, a_lrp, b_lrp,
                              openlr_start->GetString(), false);
        if (local_edges.size() == size_before) {
          auto dist_diff = std::fabs(a_lrp.distance - static_cast<double>(min_dist));
          LOG_ERROR("[valhalla] " + std::string(openlr_start->GetString()) +
                    " : 2nd match attempt didn't work with a min distance diff of " +
                    std::to_string(dist_diff));
        }
      }
    }

    // remove duplicate edge IDs
    auto last = std::unique(local_edges.begin(), local_edges.end());
    local_edges.erase(last, local_edges.end());
    if (!local_edges.size()) {
      LOG_ERROR("[valhalla] " + std::string(openlr_start->GetString()) +
                " : no edges left after removing duplicates.");
      continue;
    }

    // openlr offsets have a big-ish inaccuracy, allow up to 2 m snapping resolution
    // this makes sure we rather erase 1-2 edges too many than too few
    auto a_smaller_b = [](const float a, const float b) { return (a - (b + 2.f)) > 0; };

    // deal with poff/noff
    if (poff_meter) {
      auto first_edge = local_edges.begin();
      auto first_edge_poff = first_edge->length - first_edge->poff_start_offset;
      if (!a_smaller_b(first_edge_poff, poff_meter)) {
        first_edge++;
        uint32_t delete_count = 0;
        // we're skipping entire edges at the front until we're past the poff
        while (true) {
          // if at this point there's no nodes left, that means we already looked at the
          // last segment just before, this shouldn't happen
          if (first_edge == local_edges.end()) {
            LOG_ERROR("[tomtom] " + std::string(openlr_start->GetString()) +
                      " : poff was larger than the entire first "
                      "OpenLR segment! Why have a full segment if you'll just skip it completely?!");
            poff_meter = poff_meter - first_edge_poff;
            break;
          }

          delete_count++;
          first_edge_poff += first_edge->length;
          if (a_smaller_b(first_edge_poff, poff_meter)) {
            break;
          }
          first_edge++;
        }
        // remove the skipped edges, invalidates first_edge
        local_edges.erase(local_edges.begin(), local_edges.begin() + delete_count);
      }
      local_edges.front().poff_start_offset =
          std::max(local_edges.front().length - (first_edge_poff - poff_meter), 0.f);
    }

    if (noff_meter) {
      auto last_edge = local_edges.rbegin();
      auto last_edge_noff = last_edge->length - last_edge->noff_start_offset;
      if (!a_smaller_b(last_edge_noff, noff_meter)) {
        last_edge++;
        uint32_t delete_count = 0;
        // we're skipping entire edges at the front until we're past the poff
        while (true) {
          // if at this point there's no nodes left, that means we already looked at the
          // last segment just before, this shouldn't happen
          if (last_edge == local_edges.rend()) {
            LOG_ERROR("[tomtom] " + std::string(openlr_start->GetString()) +
                      " : poff was larger than the entire first "
                      "OpenLR segment! Why have a full segment if you'll just skip it completely?!");
            noff_meter = noff_meter - last_edge_noff;
            break;
          }

          delete_count++;
          last_edge_noff += last_edge->length;
          if (a_smaller_b(last_edge_noff, noff_meter)) {
            break;
          }
          last_edge++;
        }
        // remove the skipped edges
        for (uint32_t i = 0; i < delete_count; i++) {
          local_edges.pop_back();
        }
      }
      local_edges.back().noff_start_offset =
          std::max(local_edges.back().length - (last_edge_noff - noff_meter), 0.f);
    }

    if (local_edges.size()) {
      local_edges.back().is_last = true;
      std::move(local_edges.begin(), local_edges.end(), std::back_inserter(openlrs_edges));
    }
    local_edges.clear();
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