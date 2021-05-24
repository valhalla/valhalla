#include "thor/tile_no_longer_available_error.h"

#include <utility>

namespace valhalla {
namespace thor {

tile_no_longer_available_error_t::tile_no_longer_available_error_t(const std::string& errormessage)
    : std::runtime_error(errormessage) {
}

tile_no_longer_available_error_t::tile_no_longer_available_error_t(std::string prefix,
                                                                   baldr::GraphId edgeid)
    : std::runtime_error(std::move(prefix) + ", tile no longer available " +
                         std::to_string(edgeid.Tile_Base())) {
}

} // namespace thor
} // namespace valhalla
