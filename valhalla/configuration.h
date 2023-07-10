#ifndef VALHALLA_CONFIGURATION_H_
#define VALHALLA_CONFIGURATION_H_

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>
#include <string>

namespace valhalla {
namespace config {

class Configuration {
protected:
  Configuration() = delete;
  Configuration(const std::string& config_file);

  boost::property_tree::ptree config_;
  std::string config_file_;

public:
  Configuration(Configuration const&) = delete;
  void operator=(const Configuration&) = delete;

  static Configuration& GetInstance(const std::string& config_file = "") {
    static Configuration instance(config_file);
    return instance;
  }

  const std::string& GetConfigFile() const {
    return config_file_;
  }

  const boost::property_tree::ptree& GetConfig() const {
    return config_;
  }
};

void Configure(const std::string& config_file);
const boost::property_tree::ptree& GetConfig();

} // namespace config
} // namespace valhalla

#endif