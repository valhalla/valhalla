#ifndef VALHALLA_ODIN_TRANSIT_PLATFORM_H_
#define VALHALLA_ODIN_TRANSIT_PLATFORM_H_

#include <string>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>

namespace valhalla {
namespace odin {

struct TransitPlatform {

  TransitPlatform(TripDirections_TransitPlatform_Type type, std::string onestop_id,
                  std::string name, std::string arrival_date_time,
                  std::string departure_date_time, bool is_parent_stop,
                  bool assumed_schedule, float lat, float lng);

  TransitPlatform(TripPath_TransitPlatformInfo_Type type, std::string onestop_id,
                  std::string name, std::string arrival_date_time,
                  std::string departure_date_time, bool is_parent_stop,
                  bool assumed_schedule, float lat, float lng);

  std::string ToParameterString() const;

  TripDirections_TransitPlatform_Type type;
  std::string onestop_id;
  std::string name;
  std::string arrival_date_time;
  std::string departure_date_time;
  bool is_parent_stop;
  bool assumed_schedule;
  LatLng ll;
};

}
}

#endif  // VALHALLA_ODIN_TRANSIT_PLATFORM_H_
