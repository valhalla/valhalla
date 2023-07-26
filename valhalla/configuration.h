#ifndef VALHALLA_CONFIGURATION_H_
#define VALHALLA_CONFIGURATION_H_

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include <boost/property_tree/ptree.hpp>
#include <stdexcept>
#include <string>

namespace valhalla {
namespace configuration {

class Configuration {
protected:
  Configuration() = delete;
  Configuration(const std::string& config_file_or_inline) {
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

  boost::property_tree::ptree config_;

public:
  Configuration(Configuration const&) = delete;
  void operator=(const Configuration&) = delete;

  static Configuration& instance(const std::string& config_file_or_inline = "") {
    static Configuration instance(config_file_or_inline);
    return instance;
  }

  inline const boost::property_tree::ptree& config() const {
    return config_;
  }
};

void configure(const std::string& config_file_or_inline) {
  Configuration::instance(config_file_or_inline);
}

} // namespace configuration

inline const boost::property_tree::ptree& config() {
  return configuration::Configuration::instance().config();
}

} // namespace valhalla

#endif