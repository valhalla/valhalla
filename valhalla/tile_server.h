#pragma once

#include <string>

namespace zmq {
struct context_t;
} // namespace zmq

namespace valhalla {
class test_tile_server_t {
public:
  static const std::string server_url;
  static void start(const std::string& tile_source_dir, zmq::context_t& context);
};

} // namespace valhalla