#ifndef VALHALLA_ODIN_TRANSIT_ROUTE_INFO_H_
#define VALHALLA_ODIN_TRANSIT_ROUTE_INFO_H_

#include <list>
#include <string>

#include <valhalla/proto/trip.pb.h>
#include <valhalla/proto/tripcommon.pb.h>

namespace valhalla {
namespace odin {

// TODO maybe rename later
struct TransitRouteInfo {

#ifdef LOGGING_LEVEL_TRACE
  std::string ToParameterString() const;
#endif

  std::string onestop_id;
  uint32_t block_id;
  uint32_t trip_id;
  std::string short_name;
  std::string long_name;
  std::string headsign;
  uint32_t color;
  uint32_t text_color;
  std::string description;
  std::string operator_onestop_id;
  std::string operator_name;
  std::string operator_url;
  std::list<TransitPlatformInfo> transit_stops;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_TRANSIT_ROUTE_INFO_H_
