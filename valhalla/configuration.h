#ifndef VALHALLA_CONFIGURATION_H_
#define VALHALLA_CONFIGURATION_H_

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include <boost/property_tree/ptree.hpp>
#include <stdexcept>
#include <string>

namespace {

struct config_singleton_t {
protected:
  boost::property_tree::ptree config_;

  config_singleton_t () = delete;
  config_singleton_t(const std::string& config_file_or_inline) {
    if (config_file_or_inline.empty()) {
      throw std::runtime_error("no config provided");
    }

    if (filesystem::is_regular_file(config_file_or_inline)) {
      rapidjson::read_json(config_file_or_inline, config_);
    } else {
      auto inline_config = std::stringstream(config_file_or_inline);
      rapidjson::read_json(inline_config, config_);
    }
  }

public:
  config_singleton_t(config_singleton_t const&) = delete;
  void operator=(const config_singleton_t&) = delete;

  static const boost::property_tree::ptree& get_config(const std::string& config_file_or_inline) {
    static config_singleton_t instance(config_file_or_inline);
    return instance.config_;
  }
};
}

namespace valhalla {
  inline const boost::property_tree::ptree& config(const std::string& config_file_or_inline = ""){
    return config_singleton_t::get_config(config_file_or_inline);
  }
} // namespace valhalla

#endif