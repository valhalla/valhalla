#include "test.h"

#include <prime_server/http_protocol.hpp>
#include <prime_server/http_util.hpp>
#include <prime_server/prime_server.hpp>

#include <boost/property_tree/ptree.hpp>

#include <chrono>
#include <csignal>
#include <functional>
#include <stdexcept>
#include <thread>

#include "baldr/rapidjson_utils.h"
#include "tyr/actor.h"

using namespace prime_server;
using namespace valhalla;

worker_t::result_t
disk_work(const std::list<zmq::message_t>& job, void* request_info, worker_t::interrupt_function_t&) {
  worker_t::result_t result{false};
  try {
    // check the disk
    auto request =
        http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
    return http::disk_result(request, *static_cast<http_request_info_t*>(request_info),
                             "test/data/utrecht_tiles");
  } catch (const std::exception& e) {
    http_response_t response(400, "Bad Request", e.what());
    response.from_info(*static_cast<http_request_info_t*>(request_info));
    result.messages = {response.to_string()};
  }
  return result;
}

zmq::context_t context;
void start_server() {
  // change these to tcp://known.ip.address.with:port if you want to do this across machines
  std::string result_endpoint = "ipc:///tmp/http_test_result_endpoint";
  std::string request_interrupt = "ipc:///tmp/http_test_request_interrupt";
  std::string proxy_endpoint = "ipc:///tmp/http_test_proxy_endpoint";

  // server
  std::thread server(std::bind(&http_server_t::serve,
                               http_server_t(context, "tcp://*:8004", proxy_endpoint + "_upstream",
                                             result_endpoint, request_interrupt, true)));
  server.detach();

  // load balancer for file serving
  std::thread file_proxy(std::bind(&proxy_t::forward, proxy_t(context, proxy_endpoint + "_upstream",
                                                              proxy_endpoint + "_downstream")));
  file_proxy.detach();

  // file serving thread
  std::thread file_worker(
      std::bind(&worker_t::work, worker_t(context, proxy_endpoint + "_downstream", "ipc:///dev/null",
                                          result_endpoint, request_interrupt,
                                          std::bind(&disk_work, std::placeholders::_1,
                                                    std::placeholders::_2, std::placeholders::_3))));
  file_worker.detach();

  std::this_thread::sleep_for(std::chrono::seconds(1));
}

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

boost::property_tree::ptree make_conf(const std::string& tile_dir, bool tile_url_gz) {
  // fake up config against pine grove traffic extract
  auto conf = json_to_pt(R"({
      "mjolnir":{"tile_url":"127.0.0.1:8004"},
      "loki":{
        "actions":["locate","route","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes","transit_available"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
      },
      "thor":{"logging":{"long_request": 110}},
      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
      "meili":{"customizable": ["breakage_distance"],
               "mode":"auto","grid":{"cache_size":100240,"size":500},
               "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
               "max_route_distance_factor":3,"max_route_time_factor":3,"max_search_radius":100,"route":true,
               "search_radius":50,"sigma_z":4.07,"turn_penalty_factor":200}},
      "service_limits": {
        "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
        "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
        "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
        "skadi": {"max_shape": 750000,"min_resample": 10.0},
        "trace": { "max_best_paths": 4, "max_best_paths_shape": 100, "max_distance": 200000.0, "max_gps_accuracy": 100.0, "max_search_radius": 100, "max_shape": 16000 },
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })");

  conf.get_child("mjolnir").put("tile_dir", tile_dir);
  conf.get_child("mjolnir").put("tile_cache_gz", tile_url_gz);
  conf.get_child("loki").put("use_connectivity", false);
  return conf;
}

void clear_cache(const std::string& cache_dir) {
  // TODO: get this from filesystem test sample code
}

void test_route(const std::string& tile_dir, bool tile_url_gz) {
  auto conf = make_conf(tile_dir, tile_url_gz);
  tyr::actor_t actor(conf);

  auto route_json = actor.route(R"({"locations":[{"lat":52.09620,"lon": 5.11909,"type":"break"},
          {"lat":52.09585,"lon":5.11934,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route = json_to_pt(route_json);

  // TODO: check result
  if (route_json.find("Wijckskade") == std::string::npos ||
      route_json.find("Lauwerstraat") == std::string::npos)
    throw std::logic_error("didn't find the right street names in the route");
}

void test_no_cache_no_gz() {
  test_route("", false);
}

void test_cache_no_gz() {
  test_route("url_tile_cache", false);
  clear_cache("url_tile_cache");
}

void test_no_cache_gz() {
  test_route("", true);
}

void test_cache_gz() {
  test_route("url_tile_cache", true);
  clear_cache("url_tile_cache");
}

int main() {
  test::suite suite("http_tiles");

  // start a file server for utrecht tiles
  suite.test(TEST_CASE(start_server));

  suite.test(TEST_CASE(test_no_cache_no_gz));

  suite.test(TEST_CASE(test_cache_no_gz));

  suite.test(TEST_CASE(test_no_cache_gz));

  suite.test(TEST_CASE(test_cache_gz));

  return suite.tear_down();
}
