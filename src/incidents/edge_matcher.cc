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
static valhalla::Api get_base_req() {
  valhalla::Api base_request;
  base_request.mutable_options()->set_action(valhalla::Options::locate);
  base_request.mutable_options()->set_costing_type(valhalla::Costing::auto_);
  base_request.mutable_options()->set_verbose(true);

  return base_request;
}

std::vector<vi::RankEdge>
rank_edges(const std::optional<rapidjson::GenericArray<false, rapidjson::Value>>& edges,
           const vb::OpenLR::LocationReferencePoint& lrp) {
  std::vector<vi::RankEdge> ranked_edges;

  const auto& lrp_rc = static_cast<valhalla::RoadClass>(lrp.frc);
  for (const auto& edge : *edges) {
    valhalla::RoadClass osm_rc;
    valhalla::RoadClass_Enum_Parse(edge["classification"]["classification"].GetString(), &osm_rc);

    ranked_edges.emplace_back();
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

void match_edges(valhalla::tyr::actor_t& actor,
                 rapidjson::Value::ConstValueIterator openlr_start,
                 rapidjson::Value::ConstValueIterator openlr_end,
                 std::promise<std::vector<std::vector<vb::GraphId>>>& result) {
  for (; openlr_start != openlr_end; openlr_start++) {
    auto openlr = vb::OpenLR::OpenLr(openlr_start->GetString(), true);

    if (static_cast<uint32_t>(openlr.noff) > 1 || static_cast<uint32_t>(openlr.poff) > 1) {
      // LOG_ERROR("openLR offsets are more than 1/256th of the path: positive: " +
      // std::string(openlr.poff) +
      //          ", negative: " + std::string(openlr.noff));
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

      const auto a_lrp = openlr.lrps[i];
      valhalla::Api a_locate_req = get_base_req();
      vi::get_locate_req(a_locate_req, a_lrp, false);
      // slight optimization: take the last dest lrp's edges here if it exists (all but first segment)
      rapidjson::Document a_res;
      a_res.Parse(actor.act(a_locate_req));
      const auto a_edges = b_edges.has_value() ? *b_edges : a_res.GetArray()[0]["edges"].GetArray();
      const auto a_ranked = b_ranked.has_value() ? *b_ranked : rank_edges(a_edges, a_lrp);

      const auto b_lrp = openlr.lrps[i + 1];
      valhalla::Api b_locate_req = get_base_req();
      vi::get_locate_req(b_locate_req, b_lrp, is_last_segment);
      rapidjson::Document b_res;
      b_res.Parse(actor.act(b_locate_req));
      b_edges = std::make_optional(b_res.GetArray()[0]["edges"].GetArray());
      b_ranked = std::make_optional(rank_edges(b_edges, b_lrp));
    }

    std::cout << std::to_string(openlr.getLength()) << std::endl;
  }
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
  auto a_i = 0U;
  auto b_i = 0U;
  auto a_i_limit = a_ranked.size() - 1ULL;
  auto b_i_limit = b_ranked.size() - 1ULL;
  // if we reach the size of either container, we're done
  while (true) {

    // do the map matching and return if we're within 60 m shape length

    // here we increment 1 by 1, so a0 & b0, a1 & b0, a1 & b1, a2 & b1, a2 & b2 etc.
    // TODO: this is flawed! Need to come up with a better decision model!

    if (a_i == b_i && b_i < b_i_limit) {
      // a_i == b_i, then a0 & b1
      b_i++;
    } else if (a_i == b_i && a_i < a_i_limit) {
      // a_i == b_i, then a1 & b1
      a_i++;
    } else if ((a_i == (b_i + 1U)) && a_i < a_i_limit) {
      // a_i == b_i + 1
      b_i--;
      a_i++;
    } else if ((b_i == (a_i + 1U)) && b_i < b_i_limit) {
      // b_i == a_i + 1
      b_i++;
    } else if (a_i > b_i && a_i < a_i_limit) {
      // a_i >> b_i
      a_i++;
    } else if (b_i > a_i && b_i < b_i_limit) {
      // b_i >> a_i
      b_i++;
    } else {
      // exhausted all options..
      return {};
    }
  }
}
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

  for (const auto& openlr : openlrs_enc) {
    std::cout << openlr.GetString() << std::endl;
  }

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

  std::vector<std::vector<vb::GraphId>> openlr_graphids;
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