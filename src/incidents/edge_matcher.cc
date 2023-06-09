#include <future>
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

void rank_edges(const rapidjson::GenericArray<false, rapidjson::Value>& edges,
                vb::OpenLR::LocationReferencePoint& lrp) {
  std::vector<vi::RankEdge> ranked_edges;

  const auto& lrp_rc = static_cast<valhalla::RoadClass>(lrp.frc);
  for (const auto& edge : edges) {
    valhalla::RoadClass osm_rc;
    valhalla::RoadClass_Enum_Parse(edge["classification"]["classification"].GetString(), &osm_rc);

    ranked_edges.emplace_back(3, 3, 3);
    auto& rank_edge = ranked_edges.back();

    // TODO: test if this makes sense!
    if (osm_rc == lrp_rc) {
      rank_edge.frc_diff = 1;
    } else if (std::abs(static_cast<int>(osm_rc) - static_cast<int>(lrp_rc)) == 1) {
      rank_edge.frc_diff = 2;
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
}

void match_edges(valhalla::tyr::actor_t& actor,
                 rapidjson::Value::ConstValueIterator openlr_start,
                 rapidjson::Value::ConstValueIterator openlr_end,
                 std::promise<vi::tile_edges_t>& result) {
  for (; openlr_start != openlr_end; openlr_start++) {
    auto openlr = vb::OpenLR::OpenLr(openlr_start->GetString(), true);

    if (openlr.noff > 1 || openlr.poff > 1) {
      LOG_ERROR("openLR offsets are more than 1/256th of the path: positive: " + openlr.poff +
                ", negative: " + openlr.noff);
    }

    const auto segment_count = openlr.lrps.size() - 1;
    if (segment_count > 1) {
      LOG_WARN("Received " + std::to_string(segment_count) + " segments");
    }

    for (size_t i = 0; i < segment_count; i++) {
      const auto is_last_segment = i == openlr.lrps.size() - 2;

      const auto a_lrp = openlr.lrps[i];
      auto a_locate_req = vi::get_locate_req(a_lrp, is_last_segment);
      const auto a_edges = rapidjson::read_json(actor.act(a_locate_req))["edges"].GetArray();

      const auto b_lrp = openlr.lrps[i + 1];
      auto b_locate_req = vi::get_locate_req(a_lrp, is_last_segment);
      const auto b_edges = rapidjson::read_json(actor.act(a_locate_req))["edges"].GetArray();
    }

    std::cout << std::to_string(openlr.getLength()) << std::endl;
  }
}
} // namespace

namespace valhalla {
namespace incidents {

// matches the OpenLR entries to a list of edge Graph IDs
std::vector<baldr::GraphId> incident_worker_t::update_traffic(const rapidjson::Document& req_doc) {
  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);

  // Holds a map of tile_id (tile id + level) to vector of pure edge IDs
  std::vector<std::promise<tile_edges_t>> results(threads.size());

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
    threads[i].reset(new std::thread(match_edges, std::cref(actors[i]), openlr_start, openlr_end,
                                     std::ref(results[i])));
  }

  // Join all the threads to wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  return {};
}

} // namespace incidents
} // namespace valhalla