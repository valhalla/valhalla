#include <unordered_map>
#include <vector>

#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "odin/util.h"
#include "proto_conversions.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::odin;
using namespace valhalla::baldr;

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
    "time": 325,
    "cost": 304
},
"legs":
[
  {
      "summary":
  {
      "distance": 4973,
      "time": 325,
      "cost": 304
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
        "time": 41,
        "cost": 23
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
        "time": 284,
        "cost": 281
    },
    {
        "beginShapeIndex": 40,
        "distance": 0,
        "writtenInstruction": "You have arrived at your destination.",
        "type": 4,
        "time": 0,
        "cost": 0
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

void summary(const valhalla::Api& api, int route_index, rapidjson::writer_wrapper_t& writer) {
  double route_time = 0;
  double route_length = 0;
  double route_cost = 0;
  bool has_time_restrictions = false;
  AABB2<PointLL> bbox(10000.0f, 10000.0f, -10000.0f, -10000.0f);
  std::vector<double> recost_times(api.options().recostings_size(), 0);
  for (int leg_index = 0; leg_index < api.directions().routes(route_index).legs_size(); ++leg_index) {
    const auto& leg = api.directions().routes(route_index).legs(leg_index);
    const auto& trip_leg = api.trip().routes(route_index).legs(leg_index);
    route_time += leg.summary().time();
    route_length += leg.summary().length();
    route_cost += trip_leg.node().rbegin()->cost().elapsed_cost().cost();

    // recostings
    const auto& recosts = trip_leg.node().rbegin()->recosts();
    auto recost_time_itr = recost_times.begin();
    for (const auto& recost : recosts) {
      if (!recost.has_elapsed_cost() || (*recost_time_itr) < 0)
        (*recost_time_itr) = -1;
      else
        (*recost_time_itr) += recost.elapsed_cost().seconds();
      ++recost_time_itr;
    }

    AABB2<PointLL> leg_bbox(leg.summary().bbox().min_ll().lng(), leg.summary().bbox().min_ll().lat(),
                            leg.summary().bbox().max_ll().lng(), leg.summary().bbox().max_ll().lat());
    bbox.Expand(leg_bbox);
    has_time_restrictions = has_time_restrictions || leg.summary().has_time_restrictions();
  }

  writer.start_object("summary");
  writer("has_time_restrictions", has_time_restrictions);
  writer.set_precision(6);
  writer("min_lat", bbox.miny());
  writer("min_lon", bbox.minx());
  writer("max_lat", bbox.maxy());
  writer("max_lon", bbox.maxx());
  writer.set_precision(3);
  writer("time", route_time);
  writer("length", route_length);
  writer("cost", route_cost);
  auto recost_itr = api.options().recostings().begin();
  for (auto recost : recost_times) {
    if (recost < 0)
      writer("time_" + recost_itr->name(), std::nullptr_t());
    else
      writer("time_" + recost_itr->name(), recost);
    ++recost_itr;
  }
  writer.end_object();

  writer("status_message", "Found route between points");
  writer("status", static_cast<uint64_t>(0)); // 0 success
  writer("units", valhalla::Options_Units_Enum_Name(api.options().units()));
  writer("language", api.options().language());

  LOG_DEBUG("trip_time::" + std::to_string(route_time) + "s");
}

void locations(const valhalla::Api& api, int route_index, rapidjson::writer_wrapper_t& writer) {

  int index = 0;
  writer.set_precision(6);
  writer.start_array("locations");
  for (const auto& leg : api.directions().routes(route_index).legs()) {
    for (auto location = leg.location().begin() + index; location != leg.location().end();
         ++location) {
      index = 1;
      writer.start_object();

      writer("type", Location_Type_Enum_Name(location->type()));
      writer("lat", location->ll().lat());
      writer("lon", location->ll().lng());
      if (!location->name().empty()) {
        writer("name", location->name());
      }

      if (!location->street().empty()) {
        writer("street", location->street());
      }

      if (location->has_heading_case()) {
        writer("heading", static_cast<uint64_t>(location->heading()));
      }

      if (!location->date_time().empty()) {
        writer("date_time", location->date_time());
      }

      if (location->side_of_street() != valhalla::Location::kNone) {
        writer("side_of_street", Location_SideOfStreet_Enum_Name(location->side_of_street()));
      }

      writer("original_index", static_cast<uint64_t>(location->correlation().original_index()));

      writer.end_object();
    }
  }

  writer.end_array();
}

void legs(const valhalla::Api& api, int route_index, rapidjson::writer_wrapper_t& writer) {
  writer.start_array("legs");
  const auto& directions_legs = api.directions().routes(route_index).legs();
  auto trip_leg_itr = api.trip().routes(route_index).legs().begin();
  for (const auto& directions_leg : directions_legs) {
    writer.start_object(); // leg
    bool has_time_restrictions = false;

    if (directions_leg.maneuver_size())
      writer.start_array("maneuvers");

    for (const auto& maneuver : directions_leg.maneuver()) {
      writer.start_object();

      // Maneuver type
      writer("type", static_cast<uint64_t>(maneuver.type()));

      // Instruction and verbal instructions
      writer("instruction", maneuver.text_instruction());
      if (!maneuver.verbal_transition_alert_instruction().empty()) {
        writer("verbal_transition_alert_instruction", maneuver.verbal_transition_alert_instruction());
      }
      if (!maneuver.verbal_succinct_transition_instruction().empty()) {
        writer("verbal_succinct_transition_instruction",
               maneuver.verbal_succinct_transition_instruction());
      }
      if (!maneuver.verbal_pre_transition_instruction().empty()) {
        writer("verbal_pre_transition_instruction", maneuver.verbal_pre_transition_instruction());
      }
      if (!maneuver.verbal_post_transition_instruction().empty()) {
        writer("verbal_post_transition_instruction", maneuver.verbal_post_transition_instruction());
      }

      // Set street names
      if (maneuver.street_name_size() > 0) {
        writer.start_array("street_names");
        for (int i = 0; i < maneuver.street_name_size(); i++) {
          writer(maneuver.street_name(i).value());
        }
        writer.end_array();
      }

      // Set begin street names
      if (maneuver.begin_street_name_size() > 0) {
        writer.start_array("begin_street_names");
        for (int i = 0; i < maneuver.begin_street_name_size(); i++) {
          writer(maneuver.begin_street_name(i).value());
        }
        writer.end_array();
      }

      // Time, length, cost, and shape indexes
      const auto& end_node = trip_leg_itr->node(maneuver.end_path_index());
      const auto& begin_node = trip_leg_itr->node(maneuver.begin_path_index());
      auto cost = end_node.cost().elapsed_cost().cost() - begin_node.cost().elapsed_cost().cost();

      writer.set_precision(3);
      writer("time", maneuver.time());
      writer("length", maneuver.length());
      writer("cost", cost);
      writer("begin_shape_index", static_cast<uint64_t>(maneuver.begin_shape_index()));
      writer("end_shape_index", static_cast<uint64_t>(maneuver.end_shape_index()));
      auto recost_itr = api.options().recostings().begin();
      auto begin_recost_itr = begin_node.recosts().begin();
      for (const auto& end_recost : end_node.recosts()) {
        if (end_recost.has_elapsed_cost())
          writer("time_" + recost_itr->name(),
                 end_recost.elapsed_cost().seconds() - begin_recost_itr->elapsed_cost().seconds());
        else
          writer("time_" + recost_itr->name(), std::nullptr_t());
        ++recost_itr;
      }

      // Portions toll and rough
      if (maneuver.portions_toll()) {
        writer("toll", maneuver.portions_toll());
      }
      if (maneuver.portions_unpaved()) {
        writer("rough", maneuver.portions_unpaved());
      }
      if (maneuver.has_time_restrictions()) {
        writer("has_time_restrictions", maneuver.has_time_restrictions());
        has_time_restrictions = true;
      }

      // Process sign
      if (maneuver.has_sign()) {
        writer.start_object("sign");

        // Process exit number
        if (maneuver.sign().exit_numbers_size() > 0) {
          writer.start_array("exit_number_elements");
          for (int i = 0; i < maneuver.sign().exit_numbers_size(); ++i) {
            writer.start_object();
            // Add the exit number text
            writer("text", maneuver.sign().exit_numbers(i).text());
            // Add the exit number consecutive count only if greater than zero
            if (maneuver.sign().exit_numbers(i).consecutive_count() > 0) {
              writer("consecutive_count",
                     static_cast<uint64_t>(maneuver.sign().exit_numbers(i).consecutive_count()));
            }
            writer.end_object();
          }
          writer.end_array();
        }

        // Process exit branch
        if (maneuver.sign().exit_onto_streets_size() > 0) {
          writer.start_array("exit_branch_elements");
          for (int i = 0; i < maneuver.sign().exit_onto_streets_size(); ++i) {
            writer.start_object();
            // Add the exit branch text
            writer("text", maneuver.sign().exit_onto_streets(i).text());
            // Add the exit branch consecutive count only if greater than zero
            if (maneuver.sign().exit_onto_streets(i).consecutive_count() > 0) {
              writer("consecutive_count",
                     static_cast<uint64_t>(maneuver.sign().exit_onto_streets(i).consecutive_count()));
            }
            writer.end_object();
          }
          writer.end_array();
        }

        // Process exit toward
        if (maneuver.sign().exit_toward_locations_size() > 0) {
          writer.start_array("exit_toward_elements");
          for (int i = 0; i < maneuver.sign().exit_toward_locations_size(); ++i) {
            writer.start_object();
            // Add the exit toward text
            writer("text", maneuver.sign().exit_toward_locations(i).text());
            // Add the exit toward consecutive count only if greater than zero
            if (maneuver.sign().exit_toward_locations(i).consecutive_count() > 0) {
              writer("consecutive_count",
                     static_cast<uint64_t>(
                         maneuver.sign().exit_toward_locations(i).consecutive_count()));
            }
            writer.end_object();
          }
          writer.end_array();
        }

        // Process exit name
        if (maneuver.sign().exit_names_size() > 0) {
          writer.start_array("exit_name_elements");
          for (int i = 0; i < maneuver.sign().exit_names_size(); ++i) {
            writer.start_object();
            // Add the exit name text
            writer("text", maneuver.sign().exit_names(i).text());
            // Add the exit name consecutive count only if greater than zero
            if (maneuver.sign().exit_names(i).consecutive_count() > 0) {
              writer("consecutive_count",
                     static_cast<uint64_t>(maneuver.sign().exit_names(i).consecutive_count()));
            }
            writer.end_object();
          }
          writer.end_array();
        }

        writer.end_object(); // sign
      }

      // Roundabout count
      if (maneuver.roundabout_exit_count() > 0) {
        writer("roundabout_exit_count", static_cast<uint64_t>(maneuver.roundabout_exit_count()));
      }

      // Depart and arrive instructions
      if (!maneuver.depart_instruction().empty()) {
        writer("depart_instruction", maneuver.depart_instruction());
      }
      if (!maneuver.verbal_depart_instruction().empty()) {
        writer("verbal_depart_instruction", maneuver.verbal_depart_instruction());
      }
      if (!maneuver.arrive_instruction().empty()) {
        writer("arrive_instruction", maneuver.arrive_instruction());
      }
      if (!maneuver.verbal_arrive_instruction().empty()) {
        writer("verbal_arrive_instruction", maneuver.verbal_arrive_instruction());
      }

      // Process transit route
      if (maneuver.has_transit_info()) {
        const auto& transit_info = maneuver.transit_info();
        writer.start_object("transit_info");

        if (!transit_info.onestop_id().empty()) {
          writer("onestop_id", transit_info.onestop_id());
        }
        if (!transit_info.short_name().empty()) {
          writer("short_name", transit_info.short_name());
        }
        if (!transit_info.long_name().empty()) {
          writer("long_name", transit_info.long_name());
        }
        if (!transit_info.headsign().empty()) {
          writer("headsign", transit_info.headsign());
        }
        writer("color", static_cast<uint64_t>(transit_info.color()));
        writer("text_color", static_cast<uint64_t>(transit_info.text_color()));
        if (!transit_info.description().empty()) {
          writer("description", transit_info.description());
        }
        if (!transit_info.operator_onestop_id().empty()) {
          writer("operator_onestop_id", transit_info.operator_onestop_id());
        }
        if (!transit_info.operator_name().empty()) {
          writer("operator_name", transit_info.operator_name());
        }
        if (!transit_info.operator_url().empty()) {
          writer("operator_url", transit_info.operator_url());
        }

        // Add transit stops
        if (transit_info.transit_stops().size() > 0) {
          writer.start_array("transit_stops");
          for (const auto& transit_stop : transit_info.transit_stops()) {
            writer.start_object("transit_stop");

            // type
            if (transit_stop.type() == TransitPlatformInfo_Type_kStation) {
              writer("type", std::string("station"));
            } else {
              writer("type", std::string("stop"));
            }

            // onestop_id - using the station onestop_id
            if (!transit_stop.station_onestop_id().empty()) {
              writer("onestop_id", transit_stop.station_onestop_id());
            }

            // name - using the station name
            if (!transit_stop.station_name().empty()) {
              writer("name", transit_stop.station_name());
            }

            // arrival_date_time
            if (!transit_stop.arrival_date_time().empty()) {
              writer("arrival_date_time", transit_stop.arrival_date_time());
            }

            // departure_date_time
            if (!transit_stop.departure_date_time().empty()) {
              writer("departure_date_time", transit_stop.departure_date_time());
            }

            // assumed_schedule
            writer("assumed_schedule", transit_stop.assumed_schedule());

            // latitude and longitude
            if (transit_stop.has_ll()) {
              writer.set_precision(6);
              writer("lat", transit_stop.ll().lat());
              writer("lon", transit_stop.ll().lng());
            }

            writer.end_object(); // transit_stop
          }
          writer.end_array(); // transit_stops
        }
        writer.end_object(); // transit_info
      }

      if (maneuver.verbal_multi_cue()) {
        writer("verbal_multi_cue", maneuver.verbal_multi_cue());
      }

      // Travel mode
      auto mode_type = travel_mode_type(maneuver);
      writer("travel_mode", mode_type.first);

      // Travel type
      writer("travel_type", mode_type.second);

      //  man->emplace("hasGate", maneuver.);
      //  man->emplace("hasFerry", maneuver.);
      //“portionsTollNote” : “<portionsTollNote>”,
      //“portionsUnpavedNote” : “<portionsUnpavedNote>”,
      //“gateAccessRequiredNote” : “<gateAccessRequiredNote>”,
      //“checkFerryInfoNote” : “<checkFerryInfoNote>”

      writer.end_object(); // maneuver
    }
    if (directions_leg.maneuver_size()) {
      writer.end_array(); // maneuvers
    }

    writer.start_object("summary");
    writer("has_time_restrictions", has_time_restrictions);
    writer.set_precision(6);
    writer("min_lat", directions_leg.summary().bbox().min_ll().lat());
    writer("min_lon", directions_leg.summary().bbox().min_ll().lng());
    writer("max_lat", directions_leg.summary().bbox().max_ll().lat());
    writer("max_lon", directions_leg.summary().bbox().max_ll().lng());
    writer.set_precision(3);
    writer("time", directions_leg.summary().time());
    writer("length", directions_leg.summary().length());
    writer("cost", trip_leg_itr->node().rbegin()->cost().elapsed_cost().cost());
    auto recost_itr = api.options().recostings().begin();
    for (const auto& recost : trip_leg_itr->node().rbegin()->recosts()) {
      if (recost.has_elapsed_cost())
        writer("time_" + recost_itr->name(), recost.elapsed_cost().seconds());
      else
        writer("time_" + recost_itr->name(), std::nullptr_t());
      ++recost_itr;
    }
    ++trip_leg_itr;
    writer.end_object();

    writer("shape", directions_leg.shape());

    writer.end_object(); // leg
  }
  writer.end_array(); // legs
}

std::string serialize(const Api& api) {
  // build up the json object, reserve 4k bytes
  rapidjson::writer_wrapper_t writer(4096);

  // for each route
  for (int i = 0; i < api.directions().routes_size(); ++i) {
    if (i == 1) {
      writer.start_array("alternates");
    }

    // the route itself
    writer.start_object();
    writer.start_object("trip");

    // the locations in the trip
    locations(api, i, writer);

    // the actual meat of the route
    legs(api, i, writer);

    // openlr references of the edges in the route
    valhalla::tyr::openlr(api, i, writer);

    // summary time/distance and other stats
    summary(api, i, writer);

    // get serialized warnings
    if (api.info().warnings_size() >= 1) {
      valhalla::tyr::serializeWarnings(api, writer);
    }

    writer.end_object(); // trip

    // leave space for alternates by closing this one outside the loop
    if (i > 0) {
      writer.end_object();
    }
  }

  if (api.directions().routes_size() > 1) {
    writer.end_array(); // alternates
  }

  if (api.options().has_id_case()) {
    writer("id", api.options().id());
  }

  writer.end_object(); // outer object

  return writer.get_buffer();
}
} // namespace valhalla_serializers
} // namespace
