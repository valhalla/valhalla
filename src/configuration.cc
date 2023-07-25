#include "configuration.h"
#include "filesystem.h"
#include <stdexcept>

namespace valhalla {
namespace configuration {
Configuration::Configuration(const std::string& config_file_or_inline) {
  if (config_file_or_inline.empty()) {
    throw std::runtime_error("no config provided");
  }
             
  if(filesystem::is_regular_file(config_file_or_inline)) {
    rapidjson::read_json(config_file_or_inline, config_);
  } else {
    auto inline_config = std::stringstream(config_file_or_inline);
    rapidjson::read_json(inline_config, config_);
  }
}

void configure(const std::string& config_file_or_inline) {
  Configuration::instance(config_file_or_inline);
}

} // namespace configuration

const boost::property_tree::ptree& config() {
  return configuration::Configuration::instance().config();
}

} // namespace valhalla
