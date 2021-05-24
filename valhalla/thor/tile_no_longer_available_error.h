#ifndef VALHALLA_TILE_NO_LONGER_AVAILABLE_ERROR_H
#define VALHALLA_TILE_NO_LONGER_AVAILABLE_ERROR_H

#include "baldr/graphid.h"
#include <string>

namespace valhalla {
namespace thor {

struct tile_no_longer_available_error_t : public std::runtime_error {
  explicit tile_no_longer_available_error_t(const std::string& errormessage);
  tile_no_longer_available_error_t(std::string prefix, baldr::GraphId edgeid);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_TILE_NO_LONGER_AVAILABLE_ERROR_H
