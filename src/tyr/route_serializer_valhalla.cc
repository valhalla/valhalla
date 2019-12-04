#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "odin/util.h"
#include "tyr/serializers.h"

#include <valhalla/proto/options.pb.h>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::odin;
using namespace valhalla::baldr;
using namespace std;

namespace {

namespace valhalla_serializers {
/*
valhalla output looks like this:
{
    "trip":
{
    "status": 0,
    "locations": [
       {
        "longitude": -76.4791,
        "latitude": 40.4136,
         "stopType": 0
       },
       {
        "longitude": -76.5352,
        "latitude": 40.4029,
        "stopType": 0
       }
     ],
    "units": "kilometers"
    "summary":
{
    "distance": 4973,
    "time": 325
},
"legs":
[
  {
      "summary":
  {
      "distance": 4973,
      "time": 325
  },
  "maneuvers":
  [
    {
        "beginShapeIndex": 0,
        "distance": 633,
        "writtenInstruction": "Start out going west on West Market Street.",
        "streetNames":
        [
            "West Market Street"
        ],
        "type": 1,
        "time": 41
    },
    {
        "beginShapeIndex": 7,
        "distance": 4340,
        "writtenInstruction": "Continue onto Jonestown Road.",
        "streetNames":
        [
            "Jonestown Road"
        ],
        "type": 8,
        "time": 284
    },
    {
        "beginShapeIndex": 40,
        "distance": 0,
        "writtenInstruction": "You have arrived at your destination.",
        "type": 4,
        "time": 0
    }
],
"shape":
"gysalAlg|zpC~Clt@tDtx@hHfaBdKl{BrKbnApGro@tJrz@jBbQj@zVt@lTjFnnCrBz}BmFnoB]pHwCvm@eJxtATvXTnfAk@|^z@rGxGre@nTpnBhBbQvXduCrUr`Edd@naEja@~gAhk@nzBxf@byAfm@tuCvDtOvNzi@|jCvkKngAl`HlI|}@`N`{Adx@pjE??xB|J"
}
],
"status_message": "Found route between points"
},
"id": "work route"
}
*/
using namespace std;

json::MapPtr summary(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs) {

  uint64_t time = 0;
  long double length = 0;
  bool has_time_restrictions = false;
  AABB2<PointLL> bbox(10000.0f, 10000.0f, -10000.0f, -10000.0f);
  for (const auto& leg : legs) {
    time += static_cast<uint64_t>(leg.summary().time());
    length += leg.summary().length();

    AABB2<PointLL> leg_bbox(leg.summary().bbox().min_ll().lng(), leg.summary().bbox().min_ll().lat(),
                            leg.summary().bbox().max_ll().lng(), leg.summary().bbox().max_ll().lat());
    bbox.Expand(leg_bbox);
    has_time_restrictions = has_time_restrictions || leg.summary().has_time_restrictions();
  }

  auto route_summary = json::map({});
  route_summary->emplace("time", time);
  route_summary->emplace("length", json::fp_t{length, 3});
  route_summary->emplace("min_lat", json::fp_t{bbox.miny(), 6});
  route_summary->emplace("min_lon", json::fp_t{bbox.minx(), 6});
  route_summary->emplace("max_lat", json::fp_t{bbox.maxy(), 6});
  route_summary->emplace("max_lon", json::fp_t{bbox.maxx(), 6});
  route_summary->emplace("has_time_restrictions", json::Value{has_time_restrictions});
  LOG_DEBUG("trip_time::" + std::to_string(time) + "s");
  return route_summary;
}

json::ArrayPtr locations(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs) {
  auto locations = json::array({});

  int index = 0;
  for (auto leg = legs.begin(); leg != legs.end(); ++leg) {
    for (auto location = leg->location().begin() + index; location != leg->location().end();
         ++location) {
      index = 1;
      auto loc = json::map({});
      if (location->type() == valhalla::Location_Type_kThrough) {
        loc->emplace("type", std::string("through"));
      } else if (location->type() == valhalla::Location_Type_kVia) {
        loc->emplace("type", std::string("via"));
      } else if (location->type() == valhalla::Location_Type_kBreakThrough) {
        loc->emplace("type", std::string("break_through"));
      } else {
        loc->emplace("type", std::string("break"));
      }
      loc->emplace("lat", json::fp_t{location->ll().lat(), 6});
      loc->emplace("lon", json::fp_t{location->ll().lng(), 6});
      if (!location->name().empty()) {
        loc->emplace("name", location->name());
      }
      if (!location->street().empty()) {
        loc->emplace("street", location->street());
      }
      if (!location->city().empty()) {
        loc->emplace("city", location->city());
      }
      if (!location->state().empty()) {
        loc->emplace("state", location->state());
      }
      if (!location->postal_code().empty()) {
        loc->emplace("postal_code", location->postal_code());
      }
      if (!location->country().empty()) {
        loc->emplace("country", location->country());
      }
      if (location->has_heading()) {
        loc->emplace("heading", static_cast<uint64_t>(location->heading()));
      }
      if (!location->date_time().empty()) {
        loc->emplace("date_time", location->date_time());
      }
      if (location->has_side_of_street()) {
        if (location->side_of_street() == valhalla::Location::kLeft) {
          loc->emplace("side_of_street", std::string("left"));
        } else if (location->side_of_street() == valhalla::Location::kRight) {
          loc->emplace("side_of_street", std::string("right"));
        }
      }
      if (location->has_original_index()) {
        loc->emplace("original_index", static_cast<uint64_t>(location->original_index()));
      }

      // loc->emplace("sideOfStreet",location->side_of_street());

      locations->emplace_back(loc);
    }
  }

  return locations;
}

const std::unordered_map<int, std::string> vehicle_to_string{
    {static_cast<int>(DirectionsLeg_VehicleType_kCar), "car"},
    {static_cast<int>(DirectionsLeg_VehicleType_kMotorcycle), "motorcycle"},
    {static_cast<int>(DirectionsLeg_VehicleType_kAutoBus), "bus"},
    {static_cast<int>(DirectionsLeg_VehicleType_kTractorTrailer), "tractor_trailer"},
    {static_cast<int>(DirectionsLeg_VehicleType_kMotorScooter), "motor_scooter"},
};

std::unordered_map<int, std::string> pedestrian_to_string{
    {static_cast<int>(DirectionsLeg_PedestrianType_kFoot), "foot"},
    {static_cast<int>(DirectionsLeg_PedestrianType_kWheelchair), "wheelchair"},
    {static_cast<int>(DirectionsLeg_PedestrianType_kSegway), "segway"},
};

std::unordered_map<int, std::string> bicycle_to_string{
    {static_cast<int>(DirectionsLeg_BicycleType_kRoad), "road"},
    {static_cast<int>(DirectionsLeg_BicycleType_kCross), "cross"},
    {static_cast<int>(DirectionsLeg_BicycleType_kHybrid), "hybrid"},
    {static_cast<int>(DirectionsLeg_BicycleType_kMountain), "mountain"},
};

std::unordered_map<int, std::string> transit_to_string{
    {static_cast<int>(DirectionsLeg_TransitType_kTram), "tram"},
    {static_cast<int>(DirectionsLeg_TransitType_kMetro), "metro"},
    {static_cast<int>(DirectionsLeg_TransitType_kRail), "rail"},
    {static_cast<int>(DirectionsLeg_TransitType_kBus), "bus"},
    {static_cast<int>(DirectionsLeg_TransitType_kFerry), "ferry"},
    {static_cast<int>(DirectionsLeg_TransitType_kCableCar), "cable_car"},
    {static_cast<int>(DirectionsLeg_TransitType_kGondola), "gondola"},
    {static_cast<int>(DirectionsLeg_TransitType_kFunicular), "funicular"},
};

std::pair<std::string, std::string>
travel_mode_type(const valhalla::DirectionsLeg_Maneuver& maneuver) {
  switch (maneuver.travel_mode()) {
    case DirectionsLeg_TravelMode_kDrive: {
      auto i = maneuver.has_vehicle_type() ? vehicle_to_string.find(maneuver.vehicle_type())
                                           : vehicle_to_string.cend();
      return i == vehicle_to_string.cend() ? make_pair("drive", "car")
                                           : make_pair("drive", i->second);
    }
    case DirectionsLeg_TravelMode_kPedestrian: {
      auto i = maneuver.has_pedestrian_type() ? pedestrian_to_string.find(maneuver.pedestrian_type())
                                              : pedestrian_to_string.cend();
      return i == pedestrian_to_string.cend() ? make_pair("pedestrian", "foot")
                                              : make_pair("pedestrian", i->second);
    }
    case DirectionsLeg_TravelMode_kBicycle: {
      auto i = maneuver.has_bicycle_type() ? bicycle_to_string.find(maneuver.bicycle_type())
                                           : bicycle_to_string.cend();
      return i == bicycle_to_string.cend() ? make_pair("bicycle", "road")
                                           : make_pair("bicycle", i->second);
    }
    case DirectionsLeg_TravelMode_kTransit: {
      auto i = maneuver.has_transit_type() ? transit_to_string.find(maneuver.transit_type())
                                           : transit_to_string.cend();
      return i == transit_to_string.cend() ? make_pair("transit", "rail")
                                           : make_pair("transit", i->second);
    }
  }
  throw std::runtime_error("Unhandled case");
}

json::ArrayPtr
legs(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& directions_legs) {

  // TODO: multiple legs.
  auto legs = json::array({});
  for (const auto& directions_leg : directions_legs) {
    auto leg = json::map({});
    auto summary = json::map({});
    auto maneuvers = json::array({});
    bool has_time_restrictions = false;

    for (const auto& maneuver : directions_leg.maneuver()) {

      auto man = json::map({});

      // Maneuver type
      man->emplace("type", static_cast<uint64_t>(maneuver.type()));

      // Instruction and verbal instructions
      man->emplace("instruction", maneuver.text_instruction());
      if (maneuver.has_verbal_transition_alert_instruction()) {
        man->emplace("verbal_transition_alert_instruction",
                     maneuver.verbal_transition_alert_instruction());
      }
      if (maneuver.has_verbal_pre_transition_instruction()) {
        man->emplace("verbal_pre_transition_instruction",
                     maneuver.verbal_pre_transition_instruction());
      }
      if (maneuver.has_verbal_post_transition_instruction()) {
        man->emplace("verbal_post_transition_instruction",
                     maneuver.verbal_post_transition_instruction());
      }

      // Set street names
      if (maneuver.street_name_size() > 0) {
        auto street_names = json::array({});
        for (int i = 0; i < maneuver.street_name_size(); i++) {
          street_names->emplace_back(maneuver.street_name(i).value());
        }
        man->emplace("street_names", std::move(street_names));
      }

      // Set begin street names
      if (maneuver.begin_street_name_size() > 0) {
        auto begin_street_names = json::array({});
        for (int i = 0; i < maneuver.begin_street_name_size(); i++) {
          begin_street_names->emplace_back(maneuver.begin_street_name(i).value());
        }
        man->emplace("begin_street_names", std::move(begin_street_names));
      }

      // Time, length, and shape indexes
      man->emplace("time", static_cast<uint64_t>(maneuver.time()));
      man->emplace("length", json::fp_t{maneuver.length(), 3});
      man->emplace("begin_shape_index", static_cast<uint64_t>(maneuver.begin_shape_index()));
      man->emplace("end_shape_index", static_cast<uint64_t>(maneuver.end_shape_index()));

      // Portions toll and rough
      if (maneuver.portions_toll()) {
        man->emplace("toll", maneuver.portions_toll());
      }
      if (maneuver.portions_unpaved()) {
        man->emplace("rough", maneuver.portions_unpaved());
      }
      if (maneuver.has_time_restrictions()) {
        man->emplace("has_time_restrictions", maneuver.has_time_restrictions());
        has_time_restrictions = true;
      }

      // Process sign
      if (maneuver.has_sign()) {
        auto sign = json::map({});

        // Process exit number
        if (maneuver.sign().exit_numbers_size() > 0) {
          auto exit_number_elements = json::array({});
          for (int i = 0; i < maneuver.sign().exit_numbers_size(); ++i) {
            auto exit_number_element = json::map({});

            // Add the exit number text
            exit_number_element->emplace("text", maneuver.sign().exit_numbers(i).text());

            // Add the exit number consecutive count only if greater than zero
            if (maneuver.sign().exit_numbers(i).consecutive_count() > 0) {
              exit_number_element->emplace("consecutive_count",
                                           static_cast<uint64_t>(
                                               maneuver.sign().exit_numbers(i).consecutive_count()));
            }

            exit_number_elements->emplace_back(exit_number_element);
          }
          sign->emplace("exit_number_elements", std::move(exit_number_elements));
        }

        // Process exit branch
        if (maneuver.sign().exit_onto_streets_size() > 0) {
          auto exit_branch_elements = json::array({});
          for (int i = 0; i < maneuver.sign().exit_onto_streets_size(); ++i) {
            auto exit_branch_element = json::map({});

            // Add the exit branch text
            exit_branch_element->emplace("text", maneuver.sign().exit_onto_streets(i).text());

            // Add the exit branch consecutive count only if greater than zero
            if (maneuver.sign().exit_onto_streets(i).consecutive_count() > 0) {
              exit_branch_element
                  ->emplace("consecutive_count",
                            static_cast<uint64_t>(
                                maneuver.sign().exit_onto_streets(i).consecutive_count()));
            }

            exit_branch_elements->emplace_back(exit_branch_element);
          }
          sign->emplace("exit_branch_elements", std::move(exit_branch_elements));
        }

        // Process exit toward
        if (maneuver.sign().exit_toward_locations_size() > 0) {
          auto exit_toward_elements = json::array({});
          for (int i = 0; i < maneuver.sign().exit_toward_locations_size(); ++i) {
            auto exit_toward_element = json::map({});

            // Add the exit toward text
            exit_toward_element->emplace("text", maneuver.sign().exit_toward_locations(i).text());

            // Add the exit toward consecutive count only if greater than zero
            if (maneuver.sign().exit_toward_locations(i).consecutive_count() > 0) {
              exit_toward_element
                  ->emplace("consecutive_count",
                            static_cast<uint64_t>(
                                maneuver.sign().exit_toward_locations(i).consecutive_count()));
            }

            exit_toward_elements->emplace_back(exit_toward_element);
          }
          sign->emplace("exit_toward_elements", std::move(exit_toward_elements));
        }

        // Process exit name
        if (maneuver.sign().exit_names_size() > 0) {
          auto exit_name_elements = json::array({});
          for (int i = 0; i < maneuver.sign().exit_names_size(); ++i) {
            auto exit_name_element = json::map({});

            // Add the exit name text
            exit_name_element->emplace("text", maneuver.sign().exit_names(i).text());

            // Add the exit name consecutive count only if greater than zero
            if (maneuver.sign().exit_names(i).consecutive_count() > 0) {
              exit_name_element->emplace("consecutive_count",
                                         static_cast<uint64_t>(
                                             maneuver.sign().exit_names(i).consecutive_count()));
            }

            exit_name_elements->emplace_back(exit_name_element);
          }
          sign->emplace("exit_name_elements", std::move(exit_name_elements));
        }

        man->emplace("sign", std::move(sign));
      }

      // Roundabout count
      if (maneuver.has_roundabout_exit_count()) {
        man->emplace("roundabout_exit_count",
                     static_cast<uint64_t>(maneuver.roundabout_exit_count()));
      }

      // Depart and arrive instructions
      if (maneuver.has_depart_instruction()) {
        man->emplace("depart_instruction", maneuver.depart_instruction());
      }
      if (maneuver.has_verbal_depart_instruction()) {
        man->emplace("verbal_depart_instruction", maneuver.verbal_depart_instruction());
      }
      if (maneuver.has_arrive_instruction()) {
        man->emplace("arrive_instruction", maneuver.arrive_instruction());
      }
      if (maneuver.has_verbal_arrive_instruction()) {
        man->emplace("verbal_arrive_instruction", maneuver.verbal_arrive_instruction());
      }

      // Process transit route
      if (maneuver.has_transit_info()) {
        const auto& transit_info = maneuver.transit_info();
        auto json_transit_info = json::map({});

        if (transit_info.has_onestop_id()) {
          json_transit_info->emplace("onestop_id", transit_info.onestop_id());
          valhalla::midgard::logging::Log("transit_route_stopid::" + transit_info.onestop_id(),
                                          " [ANALYTICS] ");
        }
        if (transit_info.has_short_name()) {
          json_transit_info->emplace("short_name", transit_info.short_name());
        }
        if (transit_info.has_long_name()) {
          json_transit_info->emplace("long_name", transit_info.long_name());
        }
        if (transit_info.has_headsign()) {
          json_transit_info->emplace("headsign", transit_info.headsign());
        }
        if (transit_info.has_color()) {
          json_transit_info->emplace("color", static_cast<uint64_t>(transit_info.color()));
        }
        if (transit_info.has_text_color()) {
          json_transit_info->emplace("text_color", static_cast<uint64_t>(transit_info.text_color()));
        }
        if (transit_info.has_description()) {
          json_transit_info->emplace("description", transit_info.description());
        }
        if (transit_info.has_operator_onestop_id()) {
          json_transit_info->emplace("operator_onestop_id", transit_info.operator_onestop_id());
        }
        if (transit_info.has_operator_name()) {
          json_transit_info->emplace("operator_name", transit_info.operator_name());
        }
        if (transit_info.has_operator_url()) {
          json_transit_info->emplace("operator_url", transit_info.operator_url());
        }

        // Add transit stops
        if (transit_info.transit_stops().size() > 0) {
          auto json_transit_stops = json::array({});
          for (const auto& transit_stop : transit_info.transit_stops()) {
            auto json_transit_stop = json::map({});

            // type
            if (transit_stop.has_type()) {
              if (transit_stop.type() == TransitPlatformInfo_Type_kStation) {
                json_transit_stop->emplace("type", std::string("station"));
              } else {
                json_transit_stop->emplace("type", std::string("stop"));
              }
            }

            // onestop_id - using the station onestop_id
            if (transit_stop.has_station_onestop_id()) {
              json_transit_stop->emplace("onestop_id", transit_stop.station_onestop_id());
              valhalla::midgard::logging::Log("transit_stopid::" + transit_stop.station_onestop_id(),
                                              " [ANALYTICS] ");
            }

            // name - using the station name
            if (transit_stop.has_station_name()) {
              json_transit_stop->emplace("name", transit_stop.station_name());
            }

            // arrival_date_time
            if (transit_stop.has_arrival_date_time()) {
              json_transit_stop->emplace("arrival_date_time", transit_stop.arrival_date_time());
            }

            // departure_date_time
            if (transit_stop.has_departure_date_time()) {
              json_transit_stop->emplace("departure_date_time", transit_stop.departure_date_time());
            }

            // assumed_schedule
            if (transit_stop.has_assumed_schedule()) {
              json_transit_stop->emplace("assumed_schedule", transit_stop.assumed_schedule());
            }

            // latitude and longitude
            if (transit_stop.has_ll()) {
              json_transit_stop->emplace("lat", json::fp_t{transit_stop.ll().lat(), 6});
              json_transit_stop->emplace("lon", json::fp_t{transit_stop.ll().lng(), 6});
            }

            json_transit_stops->emplace_back(json_transit_stop);
          }
          json_transit_info->emplace("transit_stops", std::move(json_transit_stops));
        }

        man->emplace("transit_info", std::move(json_transit_info));
      }

      if (maneuver.verbal_multi_cue()) {
        man->emplace("verbal_multi_cue", maneuver.verbal_multi_cue());
      }

      // Travel mode
      auto mode_type = travel_mode_type(maneuver);
      man->emplace("travel_mode", mode_type.first);

      // Travel type
      man->emplace("travel_type", mode_type.second);

      //  man->emplace("hasGate", maneuver.);
      //  man->emplace("hasFerry", maneuver.);
      //“portionsTollNote” : “<portionsTollNote>”,
      //“portionsUnpavedNote” : “<portionsUnpavedNote>”,
      //“gateAccessRequiredNote” : “<gateAccessRequiredNote>”,
      //“checkFerryInfoNote” : “<checkFerryInfoNote>”
      maneuvers->emplace_back(man);
    }
    if (directions_leg.maneuver_size() > 0) {
      leg->emplace("maneuvers", maneuvers);
    }
    summary->emplace("time", static_cast<uint64_t>(directions_leg.summary().time()));
    summary->emplace("length", json::fp_t{directions_leg.summary().length(), 3});
    summary->emplace("min_lat", json::fp_t{directions_leg.summary().bbox().min_ll().lat(), 6});
    summary->emplace("min_lon", json::fp_t{directions_leg.summary().bbox().min_ll().lng(), 6});
    summary->emplace("max_lat", json::fp_t{directions_leg.summary().bbox().max_ll().lat(), 6});
    summary->emplace("max_lon", json::fp_t{directions_leg.summary().bbox().max_ll().lng(), 6});
    summary->emplace("has_time_restrictions", json::Value{has_time_restrictions});
    leg->emplace("summary", summary);
    leg->emplace("shape", directions_leg.shape());

    legs->emplace_back(leg);
  }
  return legs;
}

std::string serialize(const Api& api) {
  // build up the json object
  auto json = json::map(
      {{"trip", json::map({{"locations", locations(api.directions().routes(0).legs())},
                           {"summary", summary(api.directions().routes(0).legs())},
                           {"legs", legs(api.directions().routes(0).legs())},
                           {"status_message", string("Found route between points")},
                           {"status", static_cast<uint64_t>(0)}, // 0 success
                           {"units", valhalla::Options_Units_Enum_Name(api.options().units())},
                           {"language", api.options().language()}})}});
  if (api.options().has_id()) {
    json->emplace("id", api.options().id());
  }

  std::stringstream ss;
  ss << *json;
  return ss.str();
}
} // namespace valhalla_serializers
} // namespace
