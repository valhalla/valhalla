#ifndef VALHALLA_CONFIGURATION_H_
#define VALHALLA_CONFIGURATION_H_

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>
#include <string>

namespace valhalla {
namespace configuration {

class Configuration {
protected:
  Configuration() = delete;
  Configuration(const std::string& config_file_or_inline);

  boost::property_tree::ptree config_;

public:
  Configuration(Configuration const&) = delete;
  void operator=(const Configuration&) = delete;

  static Configuration& instance(const std::string& config_file_or_inline = "") {
    static Configuration instance(config_file_or_inline);
    return instance;
  }

  const boost::property_tree::ptree& config() const {
    return config_;
  }
};

void configure(const std::string& config_file_or_inline);

} // namespace configuration

const boost::property_tree::ptree& config();

} // namespace valhalla

#endif