#include "test.h"

#include "thor/service.h"
#include <valhalla/midgard/logging.h>
#include <unistd.h>

#include <thread>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::thor;


namespace {
  std::list<std::pair<std::string, std::string>> request_responses {
    {"/route?json={\"locations\":[{\"lat\":45.725925340669626,\"lon\":-74.00004386901855,\"type\":\"break\",\"date_time\":\"2015-07-06T12:15:00\"},{\"lat\":40.77157186825386,\"lon\":-73.95438194274901,\"type\":\"break\"}],\"costing\":\"bicycle\"}","Path distance exceeds the max distance limit."}
    //{"/optimized_order_route?json={\"locations\":[{\"lat\":40.761041,\"lon\":-73.99395},{\"lat\":40.75896,\"lon\":-73.985023},{\"lat\":40.753239,\"lon\":-73.979359},{\"lat\":40.748557,\"lon\":-74.007511},{\"lat\":40.759351,\"lon\":-74.002705},{\"lat\":40.731519,\"lon\":-73.98674},{\"lat\":40.716688,\"lon\":-73.985195},{\"lat\":40.71942,\"lon\":-74.009399},{\"lat\":40.744786,\"lon\":-73.997726},{\"lat\":40.765591,\"lon\":-73.961678},{\"lat\":40.784961,\"lon\":-73.95052},{\"lat\":40.799127,\"lon\":-73.945198},{\"lat\":40.78756,\"lon\":-73.975754},{\"lat\":40.771182,\"lon\":-73.986053},{\"lat\":40.735161,\"lon\":-73.998756}],\"costing\":\"auto\",\"units\":\"mi\"}","Failed to parse json request"}
  };

  void test_failure_requests() {

    //service worker
    boost::property_tree::ptree config;
    config.add("thor.logging.long_request_route", "110.0");
    config.add("thor.logging.long_request_manytomany", "15000.0");

    thor_worker_t worker(config);
    for (auto& req_resp : request_responses) {
      std::list<zmq::message_t> messages;
      void* request_info;
      messages.emplace_back(zmq::message_t(static_cast<void*>(&req_resp.first[0]), req_resp.first.size()));
      auto result = worker.work(messages, request_info);
      if (result.intermediate)
        throw std::logic_error("This cant be intermediate right now we are only testing error scenarios");
      //cant actually use !=
      if(result.messages.front() != req_resp.second)
        throw std::logic_error("The Error HTTP returned does not match what we were expecting!");
    }
  }
}

int main(void) {
  //make this whole thing bail if it doesnt finish fast
  alarm(30);

  test::suite suite("Thor Service");

  suite.test(TEST_CASE(test_failure_requests));

  return suite.tear_down();
}

