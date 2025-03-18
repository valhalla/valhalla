#pragma once
#include <filesystem>
#include <iostream>
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

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

inline std::string get_relative_file_path(std::string file_path) {
  std::string valhalla_dir_str = VALHALLA_STRINGIZE(VALHALLA_SOURCE_DIR);
  size_t len = valhalla_dir_str.length();
  return file_path.substr(len + 1);
}

#if defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_RELATIVE_FILE get_relative_file_path(__FILE__)
#else
#define VALHALLA_RELATIVE_FILE __FILE__
#endif

namespace valhalla {

const boost::property_tree::ptree& config(const std::string& config_file_or_inline = "");

} // namespace valhalla
