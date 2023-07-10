#include "configuration.h"

namespace valhalla {
namespace config {
Configuration::Configuration(const std::string& config_file) : config_file_(config_file) {
  rapidjson::read_json(config_file_, config_);
}
} // namespace config

void config::Configure(const std::string& config_file) {
  Configuration::GetInstance(config_file);
}

const boost::property_tree::ptree& config::GetConfig() {
  return config::Configuration::GetInstance().GetConfig();
}
} // namespace valhalla