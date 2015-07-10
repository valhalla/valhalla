#include "test.h"

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
#include <boost/property_tree/ptree.hpp>
#include <thread>

#include "skadi/service.h"


using namespace valhalla;
using namespace prime_server;

namespace {

  void start_service(zmq::context_t& context) {
    //server
    std::thread server(std::bind(&http_server_t::serve,
      http_server_t(context, "ipc://test_skadi_server", "ipc://test_skadi_proxy_upstream", "ipc://test_skadi_results")));
    server.detach();

    //load balancer
    std::thread proxy(std::bind(&proxy_t::forward,
      proxy_t(context, "ipc://test_skadi_proxy_upstream", "ipc://test_skadi_proxy_downstream")));
    proxy.detach();

    //service worker
    boost::property_tree::ptree config;
    //TODO: fill out the config
    std::thread worker(valhalla::skadi::run_service, config);
    worker.detach();
  }

  void test_requests() {
    //start up the service
    zmq::context_t context;
    start_service(context);

    //client makes requests and gets back responses in a batch fashion
    http_client_t client(context, "ipc://test_http_server",
      []() {
        //TODO: send the request
        return std::make_pair<const void*, size_t>(nullptr, 0);
      },
      [](const void* data, size_t size) {
        //TODO: check the response
        return false;
      }, 1
    );
    //request and receive
    client.batch();
  }
}

int main(void) {
  test::suite suite("elevation service");

  suite.test(TEST_CASE(test_requests));

  return suite.tear_down();
}
