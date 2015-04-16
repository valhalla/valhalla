#ifndef VALHALLA_MJOLNIR_ADMINBUILDER_H_
#define VALHALLA_MJOLNIR_ADMINBUILDER_H_

#include <valhalla/baldr/admin.h>

namespace valhalla {
namespace mjolnir {

class AdminInfoBuilder : public baldr::Admin {
 public:


  AdminInfoBuilder(const uint32_t country_offset, const uint32_t state_offset,
                   const std::string& country_iso, const std::string& state_iso,
                   const std::string& start_dst, const std::string& end_dst);
};

}
}

#endif  // VALHALLA_MJOLNIR_ADMINBUILDER_H_
