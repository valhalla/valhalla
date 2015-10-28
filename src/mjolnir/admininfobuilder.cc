#include "mjolnir/admininfobuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor with arguments
AdminInfoBuilder::AdminInfoBuilder(const uint32_t country_offset, const uint32_t state_offset,
                                   const std::string& country_iso, const std::string& state_iso)
    : Admin(country_offset, state_offset, country_iso, state_iso) {
}

}
}
