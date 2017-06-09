#include <string>
#include "valhalla/tyr/util.h"

#include "proto/route.pb.h"
#include "baldr/rapidjson_utils.h"

namespace valhalla {
namespace tyr {

void jsonToProtoLocation (const rapidjson::Value& json_location, Route::Location* proto_location) {
  for (const auto& member : json_location.GetObject()) {
    // Set the lat
    if (member.name.GetString() == "lat") {
      proto_location->set_lat(member.value.GetFloat());
    }
    
    // Set the lon
    else if (member.name.GetString() == "lon") {
      proto_location->set_lon(member.value.GetFloat());
    }

    // Set the type
    else if (member.name.GetString() == "type") {
      proto_location->set_type(member.value.GetString());
    }

    // Set the heading
    else if (member.name.GetString() == "heading") {
      proto_location->set_heading(member.value.GetUint());
    }

    // Set the name
    else if (member.name.GetString() == "name") {
      proto_location->set_name(member.value.GetString());
    }

    // Set the street
    else if (member.name.GetString() == "street") {
      proto_location->set_street(member.value.GetString());
    }

    // Set the city
    else if (member.name.GetString() == "city") {
      proto_location->set_city(member.value.GetString());
    }

    // Set the state
    else if (member.name.GetString() == "state") {
      proto_location->set_state(member.value.GetString());
    }

    // Set the postal_code
    else if (member.name.GetString() == "postal_code") {
      proto_location->set_postal_code(member.value.GetString());
    }

    // Set the country
    else if (member.name.GetString() == "country") {
      proto_location->set_country(member.value.GetString());
    }

    // Set the date_time
    else if (member.name.GetString() == "date_time") {
      proto_location->set_date_time(member.value.GetString());
    }

    // Set the side_of_street
    else if (member.name.GetString() == "side_of_street") {
      proto_location->set_side_of_street(member.value.GetString());
    }

    // Set the original_index
    else if (member.name.GetString() == "original_index") {
      proto_location->set_original_index(member.value.GetUint());
    }
  }
}

void jsonToProtoSummary (const rapidjson::Value& json_summary, Route::Summary* proto_summary) {
  for (const auto& member : json_summary.GetObject()) {
    // Set the length
    if (member.name.GetString() == "length") {
      proto_summary->set_length(member.value.GetFloat());
    }

    // Set the time
    else if (member.name.GetString() == "time") {
      proto_summary->set_time(member.value.GetUint());
    }

    // Set the min_lat
    else if (member.name.GetString() == "min_lat") {
      proto_summary->set_min_lat(member.value.GetFloat());
    }

    // Set the min_lon
    else if (member.name.GetString() == "min_lon") {
      proto_summary->set_min_lon(member.value.GetFloat());
    }

    // Set the max_lat
    else if (member.name.GetString() == "max_lat") {
      proto_summary->set_max_lat(member.value.GetFloat());
    }

    // Set the max_lon
    else if (member.name.GetString() == "max_lon") {
      proto_summary->set_max_lon(member.value.GetFloat());
    }
  }
}

void jsonToProtoElement (const rapidjson::Value& json_element, Route::Maneuver::Sign::Element* proto_element) {

}

void jsonToProtoSign (const rapidjson::Value& json_sign, Route::Maneuver::Sign* proto_sign) {

}

void jsonToProtoTransitStop (const rapidjson::Value& json_transit_stop, Route::TransitStop* proto_transit_stop) {

}

void jsonToProtoTransitInfo (const rapidjson::Value& json_transit_info, Route::TransitInfo* proto_transit_info) {

}

void jsonToProtoManeuver (const rapidjson::Value& json_maneuver, Route::Maneuver* proto_maneuver) {
  for (const auto& member : json_maneuver.GetObject()) {
    // Set the type
    if (member.name.GetString() == "type") {
      proto_maneuver->set_type(member.value.GetUint());
    }

    // Set the instruction
    else if (member.name.GetString() == "instruction") {
      proto_maneuver->set_instruction(member.value.GetString());
    }
  }
}

void jsonToProtoLeg (const rapidjson::Value& json_leg, Route::Leg* proto_leg) {
  for (const auto& member : json_leg.GetObject()) {
    // Set the summary
    if (member.name.GetString() == "summary") {
      jsonToProtoSummary (member.value, proto_leg->mutable_summary());
    }

    // Set the maneuvers
    if(member.name.GetString() == "maneuvers") {
      auto proto_maneuvers = proto_leg->mutable_maneuvers();
      for(const auto& maneuver : member.value.GetArray()) {
        auto proto_maneuver = proto_maneuvers->Add();
        jsonToProtoManeuver (maneuver, proto_maneuver);
      }
    }

    // Set the shape
    if (member.name.GetString() == "shape") {
      proto_leg->set_shape(member.value.GetString());
    }
  }
}

void jsonToProtoRoute (const std::string& json_route, Route& proto_route) {
  rapidjson::Document d;
  d.Parse (json_route.c_str());

  // Grab the trip object from JSON
  auto json_trip = GetOptionalFromRapidJson<rapidjson::Value::Object> (d, "/trip");
  if (!json_trip) {
    return;
  }

  if (proto_route.has_trip())
    proto_route.clear_trip();

  // Get the empty trip object and start setting it's fields
  Route::Trip* proto_trip = proto_route.mutable_trip();

  for (const auto& member : *json_trip) {
    // Set the locations
    if (member.name.GetString() == "locations") {
      auto proto_locations = proto_trip->mutable_locations();
      for(const auto& loc : member.value.GetArray()) {
        auto proto_loc = proto_locations->Add();
        jsonToProtoLocation (loc, proto_loc);
      }
    }

    // Set the summary
    else if (member.name.GetString() == "summary") {
      jsonToProtoSummary (member.value, proto_trip->mutable_summary());
    }

    // Set the legs
    else if (member.name.GetString() == "legs") {
      auto proto_legs = proto_trip->mutable_legs();
      for (const auto& leg: member.value.GetArray()) {
        auto proto_leg = proto_legs->Add();
        jsonToProtoLeg (leg, proto_leg);
      }
    }

    // Set the status_message
    else if (member.name.GetString() == "status_message") {
      proto_trip->set_status_message(member.value.GetString());
    }

    // Set the status
    else if (member.name.GetString() == "status") {
      proto_trip->set_status(member.value.GetUint());
    }

    // Set the units
    else if (member.name.GetString() == "units") {
      proto_trip->set_units(member.value.GetString());
    }

    // Set the language
    else if (member.name.GetString() == "language") {
      proto_trip->set_language(member.value.GetString());
    }

    // Set the id
    else if (member.name.GetString() == "id") {
      proto_trip->set_id(member.value.GetString());
    }
  }
}

}
}
