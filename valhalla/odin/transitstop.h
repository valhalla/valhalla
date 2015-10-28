#ifndef VALHALLA_ODIN_TRANSIT_STOP_H_
#define VALHALLA_ODIN_TRANSIT_STOP_H_

#include <string>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>

namespace valhalla {
namespace odin {

struct TransitStop {

  TransitStop(TripPath_TransitStopInfo_Type type, std::string onestop_id,
              std::string name, std::string arrival_date_time,
              std::string departure_date_time, bool is_parent_stop);

  // TODO: do we need?
  std::string ToParameterString() const;

  void set_type(TripPath_TransitStopInfo_Type in_type);

  TripDirections_TransitStop_Type type;
  std::string onestop_id;
  std::string name;
  std::string arrival_date_time;
  std::string departure_date_time;
  bool is_parent_stop;
};

}
}

#endif  // VALHALLA_ODIN_TRANSIT_STOP_H_
