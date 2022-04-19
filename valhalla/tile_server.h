#pragma once

#include <string>

namespace zmq {
struct context_t;
} // namespace zmq

namespace valhalla {
class test_tile_server_t {
  std::string m_url{"*:8004"};

public:
  void start(const std::string& tile_source_dir, zmq::context_t& context);
  void set_url(const std::string& url) {
    m_url = url;
  }
};

} // namespace valhalla