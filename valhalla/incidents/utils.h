#ifndef __VALHALLA_INCIDENTS_UTILS_H__
#define __VALHALLA_INCIDENTS_UTILS_H__

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/rapidjson_utils.h>

namespace valhalla {
namespace incidents {

struct LocateEdge {};
struct RankEdge {
  uint8_t frc_diff;
  uint32_t dist_diff;
  uint16_t heading_diff;
};

// process the locate output
Api get_locate_req(const vb::OpenLR::LocationReferencePoint& lrp, bool flip_bearing);

} // namespace incidents
} // namespace valhalla

#endif // __VALHALLA_INCIDENTS_UTILS_H__
