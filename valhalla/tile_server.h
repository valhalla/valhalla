#pragma once

#include <string>

namespace zmq {
struct context_t;
} // namespace zmq

namespace valhalla {
class test_tile_server_t {
  // change these to tcp://known.ip.address.with:port if you want to do this across machines
  std::string m_result_endpoint{"ipc:///tmp/http_test_result_endpoint"};
  std::string m_request_interrupt{"ipc:///tmp/http_test_request_interrupt"};
  std::string m_proxy_endpoint{"ipc:///tmp/http_test_proxy_endpoint"};

  std::string m_url{"127.0.0.1:8004"};

public:
  void start(const std::string& tile_source_dir, zmq::context_t& context);
  void set_url(const std::string& url) {
    m_url = url;
  }

  void set_proxy_endpoint(const std::string& end_point) {
    m_proxy_endpoint = end_point;
  }

  void set_request_interrupt(const std::string& interrupt_address) {
    m_request_interrupt = interrupt_address;
  }

  void set_result_endpoint(const std::string& result_endpoint) {
    m_result_endpoint = result_endpoint;
  }
};

} // namespace valhalla