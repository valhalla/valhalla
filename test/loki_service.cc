#include "test.h"

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <thread>
#include <unistd.h>
#include <valhalla/midgard/logging.h>

#include "loki/service.h"


using namespace valhalla;
using namespace prime_server;

namespace {
  const std::vector<http_request_t> requests {
    http_request_t(OPTIONS, ""),
    http_request_t(HEAD, ""),
    http_request_t(PUT, ""),
    http_request_t(DELETE, ""),
    http_request_t(TRACE, ""),
    http_request_t(CONNECT, ""),
    http_request_t(GET, ""),
    http_request_t(POST, ""),
    http_request_t(GET, "/route?json={"),
    http_request_t(POST, "/route", "{"),
    http_request_t(GET, "/route"),
    http_request_t(POST, "/route"),
    http_request_t(GET, "/many_to_one"),
    http_request_t(POST, "/many_to_many"),
    http_request_t(GET, "/locate?json={\"locations\":[{\"lon\":0}]}"),
    http_request_t(POST, "/locate", "{\"locations\":[{\"lon\":0}]}"),
    http_request_t(GET, "/route?json={\"locations\":[{\"lon\":0,\"lat\":90}]}"),
    http_request_t(POST, "/route", "{\"locations\":[{\"lon\":0,\"lat\":90}]}"),
    http_request_t(GET, "/route?json={\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}]}"),
    http_request_t(POST, "/route", "{\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}]}"),
    http_request_t(GET, "/route?json={\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}]}"),
    http_request_t(POST, "/route", "{\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}]}"),
    http_request_t(GET, "/route?json={\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":-90}], \"costing\": \"pedestrian\"}"),
    http_request_t(POST, "/route", "{\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":-90}], \"costing\": \"pedestrian\"}"),
    http_request_t(GET, "/locate?json={\"locations\":[{\"lon\":0,\"lat\":90}], \"costing\": \"yak\"}"),
    http_request_t(POST, "/locate", "{\"locations\":[{\"lon\":0,\"lat\":90}], \"costing\": \"yak\"}"),
    http_request_t(GET, "/route?json={\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}"
        ",{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}"
        ",{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}], \"costing\": \"auto\"}"),
    http_request_t(POST, "/route", "{\"locations\":[{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}"
        ",{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}"
        ",{\"lon\":0,\"lat\":90},{\"lon\":0,\"lat\":90}], \"costing\": \"auto\"}"),
    http_request_t(GET, "/one_to_many?json={\"sources\":[{\"lon\":0,\"lat\":90}]}"),
    http_request_t(GET, "/many_to_one?json={\"targets\":[{\"lon\":0,\"lat\":90}]}"),
    http_request_t(GET, "/many_to_many?json={\"locations\":[{\"lon\":0,\"lat\":90}]}"),
    http_request_t(GET, "/sources_to_targets?json={\"targets\":[{\"lon\":0,\"lat\":90}]}"),
    http_request_t(GET, "/sources_to_targets?json={\"locations\":[{\"lon\":0,\"lat\":90}]}")
  };

  const std::vector<std::pair<uint16_t,std::string> > responses {
    {405, std::string("{\"code\":405,\"status\":\"Method Not Allowed\",\"error\":\"Try a POST or GET request instead\"}")},
    {405, std::string("{\"code\":405,\"status\":\"Method Not Allowed\",\"error\":\"Try a POST or GET request instead\"}")},
    {405, std::string("{\"code\":405,\"status\":\"Method Not Allowed\",\"error\":\"Try a POST or GET request instead\"}")},
    {405, std::string("{\"code\":405,\"status\":\"Method Not Allowed\",\"error\":\"Try a POST or GET request instead\"}")},
    {405, std::string("{\"code\":405,\"status\":\"Method Not Allowed\",\"error\":\"Try a POST or GET request instead\"}")},
    {405, std::string("{\"code\":405,\"status\":\"Method Not Allowed\",\"error\":\"Try a POST or GET request instead\"}")},
    {404, std::string("{\"code\":404,\"status\":\"Not Found\",\"error\":\"Try any of: \'\\/locate\' \'\\/route\' \'\\/one_to_many\' \'\\/many_to_one\' \'\\/many_to_many\' \'\\/sources_to_targets\' \'\\/optimized_route\' \'\\/isochrone\' \"}")},
    {404, std::string("{\"code\":404,\"status\":\"Not Found\",\"error\":\"Try any of: \'\\/locate\' \'\\/route\' \'\\/one_to_many\' \'\\/many_to_one\' \'\\/many_to_many\' \'\\/sources_to_targets\' \'\\/optimized_route\' \'\\/isochrone\' \"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Failed to parse json request\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Failed to parse json request\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficiently specified required parameter \'locations\'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficiently specified required parameter \'locations\'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficiently specified required parameter \'locations\' or \'sources & targets\'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficiently specified required parameter \'locations\' or \'sources & targets\'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Failed to parse location\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Failed to parse location\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficient number of locations provided\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficient number of locations provided\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"No edge\\/node costing provided\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"No edge\\/node costing provided\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"No edge\\/node costing provided\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"No edge\\/node costing provided\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Path distance exceeds the max distance limit\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Path distance exceeds the max distance limit\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"No costing method found for 'yak'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"No costing method found for 'yak'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Exceeded max locations of 20.\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Exceeded max locations of 20.\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficiently specified required parameter \'locations\' or \'sources & targets\'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficiently specified required parameter \'locations\' or \'sources & targets\'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficient number of locations provided\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficiently specified required parameter \'locations\' or \'sources & targets\'\"}")},
    {400, std::string("{\"code\":400,\"status\":\"Bad Request\",\"error\":\"Insufficient number of locations provided\"}")}
  };


  void start_service(zmq::context_t& context) {
    //server
    std::thread server(std::bind(&http_server_t::serve,
      http_server_t(context, "ipc:///tmp/test_loki_server", "ipc:///tmp/test_loki_proxy_in", "ipc:///tmp/test_loki_results")));
    server.detach();

    //load balancer
    std::thread proxy(std::bind(&proxy_t::forward,
      proxy_t(context, "ipc:///tmp/test_loki_proxy_in", "ipc:///tmp/test_loki_proxy_out")));
    proxy.detach();

    //make the config file
    boost::property_tree::ptree config;
    std::stringstream json; json << "{ \
      \"mjolnir\": { \"tile_dir\": \"test/tiles\" }, \
      \"loki\": { \"actions\": [ \"locate\",\"route\",\"one_to_many\",\"many_to_one\",\"many_to_many\",\"sources_to_targets\",\"optimized_route\",\"isochrone\" ], \
                  \"logging\": { \"long_request\": 100.0 }, \
                  \"service\": { \"proxy\": \"ipc:///tmp/test_loki_proxy\" } }, \
      \"thor\": { \"service\": { \"proxy\": \"ipc:///tmp/test_thor_proxy\" } }, \
      \"httpd\": { \"service\": { \"loopback\": \"ipc:///tmp/test_loki_results\" } }, \
      \"service_limits\": { \
        \"auto\": { \"max_distance\": 5000000.0, \"max_locations\": 20 }, \
        \"pedestrian\": { \"max_distance\": 250000.0, \"max_locations\": 50, \
        \"min_transit_walking_distance\": 1, \"max_transit_walking_distance\": 10000 }, \
        \"sources_to_targets\": { \"max_distance\": 200000.0, \"max_locations\": 50}, \
        \"optimized_route\": { \"max_distance\": 200000.0, \"max_locations\": 50}, \
        \"isochrone\": { \"max_contours\": 4, \"max_time\": 120, \"max_locations\": 1}, \
        \"max_shape\": 750000,\
        \"min_resample\": 10.0 \
      }, \
     \"costing_options\": { \"auto\": {}, \"pedestrian\": {} } \
      }";
    boost::property_tree::json_parser::read_json(json, config);

    //service worker
    std::thread worker(valhalla::loki::run_service, config);
    worker.detach();
  }

  void test_failure_requests() {
    //start up the service
    zmq::context_t context;
    start_service(context);

    //client makes requests and gets back responses in a batch fashion
    auto request = requests.cbegin();
    std::string request_str;
    http_client_t client(context, "ipc:///tmp/test_loki_server",
      [&request, &request_str]() {
        //we dont have any more requests so bail
        if(request == requests.cend())
          return std::make_pair<const void*, size_t>(nullptr, 0);
        //get the string of bytes to send formatted for http protocol
        request_str = request->to_string();
        //LOG_INFO("Loki Test Request :: " + request_str + '\n');
        ++request;
        return std::make_pair<const void*, size_t>(request_str.c_str(), request_str.size());
      },
      [&request](const void* data, size_t size) {
        auto response = http_response_t::from_string(static_cast<const char*>(data), size);
        if(response.code != responses[request - requests.cbegin() - 1].first)
          throw std::runtime_error("Expected Response Code: " + std::to_string(responses[request - requests.cbegin() - 1].first) +", Actual Response Code: " + std::to_string(response.code));
        if(response.body != responses[request - requests.cbegin() - 1].second)
          throw std::runtime_error("Expected Response: " + responses[request - requests.cbegin() - 1].second +", Actual Response: " + response.body);

        return request != requests.cend();
      }, 1
    );
    //request and receive
    client.batch();
  }
}

int main(void) {
  //make this whole thing bail if it doesnt finish fast
  alarm(90);

  test::suite suite("Loki Service");

  //test failures
  suite.test(TEST_CASE(test_failure_requests));

  //test successes
  //suite.test(TEST_CASE(test_success_requests));

  return suite.tear_down();
}
