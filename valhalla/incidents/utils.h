#ifndef __VALHALLA_INCIDENTS_UTILS_H__
#define __VALHALLA_INCIDENTS_UTILS_H__

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/api.pb.h>

namespace valhalla {
namespace incidents {

struct RankEdge {
  uint8_t frc_diff;
  uint32_t dist_diff;
  uint16_t heading_diff;
};

// process the locate output
void get_locate_req(Api& request,
                    const baldr::OpenLR::LocationReferencePoint& lrp,
                    const bool flip_bearing);

} // namespace incidents
} // namespace valhalla

#endif // __VALHALLA_INCIDENTS_UTILS_H__
