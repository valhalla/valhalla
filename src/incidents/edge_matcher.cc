#include <future>
#include <thread>

#include <valhalla/baldr/openlr.h>
#include <valhalla/incidents/worker.h>
#include <valhalla/tyr/actor.h>

namespace vi = valhalla::incidents;
namespace vb = valhalla::baldr;
namespace vt = valhalla::tyr;

namespace {
void match_edges(const valhalla::tyr::actor_t& actor,
                 rapidjson::Value::ConstValueIterator openlr_start,
                 rapidjson::Value::ConstValueIterator openlr_end,
                 std::promise<vi::tile_edges_t>& result) {
  for (; openlr_start != openlr_end; openlr_start++) {
    auto openlr = vb::OpenLR::OpenLr(openlr_start->GetString(), true);
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