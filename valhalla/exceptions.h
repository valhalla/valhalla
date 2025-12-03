#ifndef __VALHALLA_EXCEPTIONS_H__
#define __VALHALLA_EXCEPTIONS_H__

#include <stdexcept>
#include <string>

namespace valhalla {
class Api;

/**
 * Project specific error messages and codes that can be converted to http responses
 */
struct valhalla_exception_t : public std::runtime_error {
  /**
   * Constructs the exception by looking up predefined ones by their codes. If unsuccessful the code
   * will be 0
   * @param code   the code to look up
   * @param extra  an extra string to append to the codes existing method
   */
  valhalla_exception_t(unsigned code, const std::string& extra = "");
  valhalla_exception_t(unsigned code,
                       const std::string& message,
                       unsigned http_code,
                       const std::string& http_message,
                       const std::string& osrm_error,
                       const std::string& statsd_key = "")
      : std::runtime_error(""), code(code), message(message), http_code(http_code),
        http_message(http_message), osrm_error(osrm_error), statsd_key(statsd_key) {
  }
  const char* what() const noexcept override {
    return message.c_str();
  }
  unsigned code;
  std::string message;
  unsigned http_code;
  std::string http_message;
  std::string osrm_error;
  std::string statsd_key;
};

/**
 * Adds a warning to the request PBF object.
 *
 * @param api   the full request
 * @param code  the warning code
 * @param extra an optional string to append to the hard-coded warning message
 */
void add_warning(valhalla::Api& api, unsigned code, const std::string& extra = "");
} // namespace valhalla

#endif //__VALHALLA_EXCEPTIONS_H__
