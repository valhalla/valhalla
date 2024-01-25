#include "config.h"

namespace {
struct config_singleton_t {
protected:
  boost::property_tree::ptree config_;

  config_singleton_t() = delete;

  config_singleton_t(const std::string& config_file_or_inline) {
    if (config_file_or_inline.empty()) {
      throw std::runtime_error("Config singleton was not initialized before usage");
    }

    try {
      if (std::filesystem::is_regular_file(config_file_or_inline)) {
        rapidjson::read_json(config_file_or_inline, config_);
      } else {
        auto inline_config = std::stringstream(config_file_or_inline);
        rapidjson::read_json(inline_config, config_);
      }
    } catch (const std::filesystem::filesystem_error& e) {
      if (e.code() == std::errc::filename_too_long) {
        auto inline_config = std::stringstream(config_file_or_inline);
        rapidjson::read_json(inline_config, config_);
      } else {
        throw e;
      }
    }
  }

public:
  config_singleton_t(config_singleton_t const&) = delete;
  void operator=(const config_singleton_t&) = delete;
  friend const boost::property_tree::ptree&
  valhalla::config(const std::string& config_file_or_inline);
};
} // namespace

namespace valhalla {

const boost::property_tree::ptree& config(const std::string& config_file_or_inline) {
  static config_singleton_t instance(config_file_or_inline);
  return instance.config_;
}

}; // namespace valhalla
