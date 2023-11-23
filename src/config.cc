#include "config.h"
#include <mutex>
namespace valhalla {
const boost::property_tree::ptree& config(const std::string& config_file_or_inline) {
  static config_singleton_t instance(config_file_or_inline);
  return instance.config_;
}
}; // namespace valhalla