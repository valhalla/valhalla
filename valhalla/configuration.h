#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>
#include <string>

namespace valhalla {
namespace config {

class Configuration {
protected:
  Configuration() = delete;
  Configuration(const std::string& path) : path_(path) {
    rapidjson::read_json(path, config_);
  }

  boost::property_tree::ptree config_;
  std::string path_;

public:
  Configuration(Configuration const&) = delete;
  void operator=(const Configuration&) = delete;

  static Configuration& GetInstance(const std::string& path = "") {
    static Configuration instance(path);
    return instance;
  }

  std::string GetPath() const {
    return path_;
  }

  boost::property_tree::ptree GetConfig() const {
    return config_;
  }
};

void Configure(const std::string& path){
  Configuration::GetInstance(path);
}

boost::property_tree::ptree GetConfig() {
  return Configuration::GetInstance().GetConfig();
}

} // namespace config
} // namespace valhalla
