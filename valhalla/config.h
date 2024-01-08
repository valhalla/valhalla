#pragma once
#include <filesystem>
#include <stdexcept>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "baldr/rapidjson_utils.h"
#include "valhalla.h"

#define VALHALLA_STRINGIZE_NX(A) #A
#define VALHALLA_STRINGIZE(A) VALHALLA_STRINGIZE_NX(A)

/* Version number of package */

// clang-format off
#define VALHALLA_VERSION VALHALLA_STRINGIZE(VALHALLA_VERSION_MAJOR) "." VALHALLA_STRINGIZE(VALHALLA_VERSION_MINOR) "." VALHALLA_STRINGIZE(VALHALLA_VERSION_PATCH)
// clang-format on

/* Name of package */
#define PACKAGE "valhalla-" VALHALLA_VERSION

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT "https://github.com/valhalla/valhalla/issues"

/* Define to the full name of this package. */
#define PACKAGE_NAME "valhalla"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "valhalla " VALHALLA_VERSION

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "valhalla-" VALHALLA_VERSION

/* Define to the home page for this package. */
#define PACKAGE_URL "https://github.com/valhalla/valhalla"

/* Define to the version of this package. */
#define PACKAGE_VERSION VALHALLA_VERSION

namespace valhalla {
const boost::property_tree::ptree& config(const std::string&);

class ConfigUninitializedException : std::exception {
public:
  const char* what() const throw() {
    return "Config singleton was not initialized before usage";
  }
};
} // namespace valhalla

namespace {
struct config_singleton_t {
protected:
  boost::property_tree::ptree config_;

  config_singleton_t() = delete;
  config_singleton_t(const std::string& config_file_or_inline) {
    if (config_file_or_inline.empty()) {
      throw valhalla::ConfigUninitializedException();
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
inline const boost::property_tree::ptree& config(const std::string& config_file_or_inline = "") {
  static config_singleton_t instance(config_file_or_inline);
  return instance.config_;
}
} // namespace valhalla
