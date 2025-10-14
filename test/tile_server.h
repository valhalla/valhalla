#pragma once

#include <string>

namespace zmq {
struct context_t;
} // namespace zmq

namespace valhalla {
class test_tile_server_t {
  std::string m_url{"*:8004"};
  std::string m_user_pw;

public:
  void
  start(const std::string& tile_source_dir, const std::string& tar_path, zmq::context_t& context);
  void set_url(const std::string& url) {
    m_url = url;
  }
  void set_user_pw(const std::string& user_pw) {
    m_user_pw = user_pw;
  }
};

} // namespace valhalla