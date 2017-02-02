#include <unordered_map>

#include "proto/tripdirections.pb.h"
#include "proto/directions_options.pb.h"
#include "odin/transitstop.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

const std::unordered_map<int, TripDirections_TransitStop_Type> translate_transit_stop_type {
  { static_cast<int>(TripPath_TransitStopInfo_Type_kStop), TripDirections_TransitStop_Type_kStop },
  { static_cast<int>(TripPath_TransitStopInfo_Type_kStation), TripDirections_TransitStop_Type_kStation },
};

TransitStop::TransitStop(TripDirections_TransitStop_Type type,
                         std::string onestop_id,
                         std::string name,
                         std::string arrival_date_time,
                         std::string departure_date_time,
                         bool is_parent_stop,
                         bool assumed_schedule,
                         float lat,
                         float lng)
    : type(type),
      onestop_id(onestop_id),
      name(name),
      arrival_date_time(arrival_date_time),
      departure_date_time(departure_date_time),
      is_parent_stop(is_parent_stop),
      assumed_schedule(assumed_schedule) {
  ll.set_lat(lat);
  ll.set_lng(lng);
}

TransitStop::TransitStop(TripPath_TransitStopInfo_Type type,
                         std::string onestop_id,
                         std::string name,
                         std::string arrival_date_time,
                         std::string departure_date_time,
                         bool is_parent_stop,
                         bool assumed_schedule,
                         float lat,
                         float lng)
    : TransitStop(translate_transit_stop_type.find(type)->second, onestop_id,
                  name, arrival_date_time, departure_date_time, is_parent_stop,
                  assumed_schedule, lat, lng) {
}

std::string TransitStop::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str += "{ ";

  str += "TripDirections_TransitStop_Type_";
  str += TripDirections_TransitStop_Type_descriptor()->FindValueByNumber(type)->name();

  str += delim;
  str += GetQuotedString(onestop_id);

  str += delim;
  str += GetQuotedString(name);

  str += delim;
  str += GetQuotedString(arrival_date_time);

  str += delim;
  str += GetQuotedString(departure_date_time);

  str += delim;
  str += std::to_string(is_parent_stop);

  str += delim;
  str += std::to_string(assumed_schedule);

  str += delim;
  str += std::to_string(ll.lat());

  str += delim;
  str += std::to_string(ll.lng());

  str += " }";

  return str;
}

}
}
