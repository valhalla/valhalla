#include <string>
#include "valhalla/tyr/util.h"

#include "proto/route.pb.h"
#include "baldr/rapidjson_utils.h"

namespace valhalla {
namespace tyr {

void jsonToProtoLocation (const rapidjson::Value& json_location, Route::Location* proto_location) {
  for (const auto& member : json_location.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the lat
    if (member_name == "lat") {
      proto_location->set_lat(member.value.GetFloat());
    }
    
    // Set the lon
    else if (member_name == "lon") {
      proto_location->set_lon(member.value.GetFloat());
    }

    // Set the type
    else if (member_name == "type") {
      proto_location->set_type(member.value.GetString());
    }

    // Set the heading
    else if (member_name == "heading") {
      proto_location->set_heading(member.value.GetUint());
    }

    // Set the name
    else if (member_name == "name") {
      proto_location->set_name(member.value.GetString());
    }

    // Set the street
    else if (member_name == "street") {
      proto_location->set_street(member.value.GetString());
    }

    // Set the city
    else if (member_name == "city") {
      proto_location->set_city(member.value.GetString());
    }

    // Set the state
    else if (member_name == "state") {
      proto_location->set_state(member.value.GetString());
    }

    // Set the postal_code
    else if (member_name == "postal_code") {
      proto_location->set_postal_code(member.value.GetString());
    }

    // Set the country
    else if (member_name == "country") {
      proto_location->set_country(member.value.GetString());
    }

    // Set the date_time
    else if (member_name == "date_time") {
      proto_location->set_date_time(member.value.GetString());
    }

    // Set the side_of_street
    else if (member_name == "side_of_street") {
      proto_location->set_side_of_street(member.value.GetString());
    }

    // Set the original_index
    else if (member_name == "original_index") {
      proto_location->set_original_index(member.value.GetUint());
    }
  }
}

void jsonToProtoSummary (const rapidjson::Value& json_summary, Route::Summary* proto_summary) {
  for (const auto& member : json_summary.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the length
    if (member_name == "length") {
      proto_summary->set_length(member.value.GetFloat());
    }

    // Set the time
    else if (member_name == "time") {
      proto_summary->set_time(member.value.GetUint());
    }

    // Set the min_lat
    else if (member_name == "min_lat") {
      proto_summary->set_min_lat(member.value.GetFloat());
    }

    // Set the min_lon
    else if (member_name == "min_lon") {
      proto_summary->set_min_lon(member.value.GetFloat());
    }

    // Set the max_lat
    else if (member_name == "max_lat") {
      proto_summary->set_max_lat(member.value.GetFloat());
    }

    // Set the max_lon
    else if (member_name == "max_lon") {
      proto_summary->set_max_lon(member.value.GetFloat());
    }
  }
}

void jsonToProtoElement (const rapidjson::Value& json_element, Route::Maneuver::Sign::Element* proto_element) {
  for (const auto& member : json_element.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the text
    if (member_name == "text") {
      proto_element->set_text(member.value.GetString());
    }

    // Set the consecutive_count
    else if (member_name == "consecutive_count") {
      proto_element->set_consecutive_count(member.value.GetUint());
    }
  }
}

void jsonToProtoSign (const rapidjson::Value& json_sign, Route::Maneuver::Sign* proto_sign) {
  for (const auto& member : json_sign.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the exit_number_elements
    if (member_name == "exit_number_elements") {
      auto proto_exit_number_elements = proto_sign->mutable_exit_number_elements();
      for (const auto& exit_number_element : member.value.GetArray()) {
        auto proto_exit_number_element = proto_exit_number_elements->Add();
        jsonToProtoElement(exit_number_element, proto_exit_number_element);
      }
    }

    // Set the exit_branch_elements
    if (member_name == "exit_branch_elements") {
      auto proto_exit_branch_elements = proto_sign->mutable_exit_branch_elements();
      for (const auto& exit_branch_element : member.value.GetArray()) {
        auto proto_exit_branch_element = proto_exit_branch_elements->Add();
        jsonToProtoElement(exit_branch_element, proto_exit_branch_element);
      }
    }

    // Set the exit_toward_elements
    if (member_name == "exit_toward_elements") {
      auto proto_exit_toward_elements = proto_sign->mutable_exit_toward_elements();
      for (const auto& exit_toward_element : member.value.GetArray()) {
        auto proto_exit_toward_element = proto_exit_toward_elements->Add();
        jsonToProtoElement(exit_toward_element, proto_exit_toward_element);
      }
    }

    // Set the exit_name_elements
    if (member_name == "exit_name_elements") {
      auto proto_exit_name_elements = proto_sign->mutable_exit_name_elements();
      for (const auto& exit_name_element : member.value.GetArray()) {
        auto proto_exit_name_element = proto_exit_name_elements->Add();
        jsonToProtoElement(exit_name_element, proto_exit_name_element);
      }
    }
  }
}

void jsonToProtoTransitStop (const rapidjson::Value& json_transit_stop, Route::TransitStop* proto_transit_stop) {
  for (const auto& member : json_transit_stop.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the type
    if (member_name == "type") {
      proto_transit_stop->set_type(member.value.GetString());
    }

    // Set the onestop_id
    else if (member_name == "onestop_id") {
      proto_transit_stop->set_onestop_id(member.value.GetString());
    }

    // Set the name
    else if (member_name == "name") {
      proto_transit_stop->set_name(member.value.GetString());
    }

    // Set the arrival_date_time
    else if (member_name == "arrival_date_time") {
      proto_transit_stop->set_arrival_date_time(member.value.GetString());
    }

    // Set the departure_date_time
    else if (member_name == "departure_date_time") {
      proto_transit_stop->set_departure_date_time(member.value.GetString());
    }

    // Set is_parent_stop
    else if (member_name == "is_parent_stop") {
      proto_transit_stop->set_is_parent_stop(member.value.GetBool());
    }

    // Set assumed_schedule
    else if (member_name == "assumed_schedule") {
      proto_transit_stop->set_assumed_schedule(member.value.GetBool());
    }

    // Set the lat
    else if (member_name == "lat") {
      proto_transit_stop->set_lat(member.value.GetFloat());
    }

    // Set the lon
    else if (member_name == "lon") {
      proto_transit_stop->set_lon(member.value.GetFloat());
    }
  }
}

void jsonToProtoTransitInfo (const rapidjson::Value& json_transit_info, Route::TransitInfo* proto_transit_info) {
  for (const auto& member : json_transit_info.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the onestop_id
    if (member_name == "onestop_id") {
      proto_transit_info->set_onestop_id(member.value.GetString());
    }

    // Set the short_name
    else if (member_name == "short_name") {
      proto_transit_info->set_short_name(member.value.GetString());
    }

    // Set the long_name
    else if (member_name == "long_name") {
      proto_transit_info->set_long_name(member.value.GetString());
    }

    // Set the headsign
    else if (member_name == "headsign") {
      proto_transit_info->set_headsign(member.value.GetString());
    }

    // Set the color
    else if (member_name == "color") {
      proto_transit_info->set_color(member.value.GetUint());
    }

    // Set the text_color
    else if (member_name == "text_color") {
      proto_transit_info->set_text_color(member.value.GetUint());
    }

    // Set the description
    else if (member_name == "description") {
      proto_transit_info->set_description(member.value.GetString());
    }

    // Set the operator_onestop_id
    else if (member_name == "operator_onestop_id") {
      proto_transit_info->set_operator_onestop_id(member.value.GetString());
    }

    // Set the operator_name
    else if (member_name == "operator_name") {
      proto_transit_info->set_operator_name(member.value.GetString());
    }

    // Set the operator_url
    else if (member_name == "operator_url") {
      proto_transit_info->set_operator_url(member.value.GetString());
    }

    // Set the transit_stops
    else if (member_name == "transit_stops") {
      auto proto_transit_stops = proto_transit_info->mutable_transit_stops();
      for (const auto& transit_stop : member.value.GetArray()) {
        auto proto_transit_stop = proto_transit_stops->Add();
        jsonToProtoTransitStop(transit_stop, proto_transit_stop);
      }
    }
  }
}

void jsonToProtoManeuver (const rapidjson::Value& json_maneuver, Route::Maneuver* proto_maneuver) {
  for (const auto& member : json_maneuver.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the type
    if (member_name == "type") {
      proto_maneuver->set_type(member.value.GetUint());
    }

    // Set the instruction
    else if (member_name == "instruction") {
      proto_maneuver->set_instruction(member.value.GetString());
    }

    // Set the street_names
    else if (member_name == "street_names") {
      auto proto_street_names = proto_maneuver->mutable_street_names();
      for (const auto& street_name : member.value.GetArray()) {
        auto proto_street_name = proto_street_names->Add();
        *proto_street_name = street_name.GetString();
      }
    }

    // Set the length
    else if (member_name == "length") {
      proto_maneuver->set_length(member.value.GetFloat());
    }

    // Set the time
    else if (member_name == "time") {
      proto_maneuver->set_time(member.value.GetUint());
    }

    // Set the begin_cardinal_direction
    else if (member_name == "begin_cardinal_direction") {
      proto_maneuver->set_begin_cardinal_direction(member.value.GetString());
    }

    // Set the begin_heading
    else if (member_name == "begin_heading") {
      proto_maneuver->set_begin_heading(member.value.GetUint());
    }

    // Set the begin_shape_index
    else if (member_name == "begin_shape_index") {
      proto_maneuver->set_begin_shape_index(member.value.GetUint());
    }

    // Set the end_shape_index
    else if (member_name == "end_shape_index") {
      proto_maneuver->set_end_shape_index(member.value.GetUint());
    }

    // Set toll
    else if (member_name == "toll") {
      proto_maneuver->set_toll(member.value.GetBool());
    }

    // Set rough
    else if (member_name == "rough") {
      proto_maneuver->set_rough(member.value.GetBool());
    }

    // Set the verbal_transition_alert_instruction
    else if (member_name == "verbal_transition_alert_instruction") {
      proto_maneuver->set_verbal_transition_alert_instruction(member.value.GetString());
    }

    // Set the verbal_pre_transition_instruction
    else if (member_name == "verbal_pre_transition_instruction") {
      proto_maneuver->set_verbal_pre_transition_instruction(member.value.GetString());
    }

    // Set the verbal_post_transition_instruction
    else if (member_name == "verbal_post_transition_instruction") {
      proto_maneuver->set_verbal_post_transition_instruction(member.value.GetString());
    }

    // Set the begin_street_names
    else if (member_name == "begin_street_names") {
      auto proto_begin_street_names = proto_maneuver->mutable_begin_street_names();
      for (const auto& begin_street_name : member.value.GetArray()) {
        auto proto_begin_street_name = proto_begin_street_names->Add();
        *proto_begin_street_name = begin_street_name.GetString();
      }
    }

    // Set the sign
    else if (member_name == "sign") {
      jsonToProtoSign (member.value, proto_maneuver->mutable_sign());
    }

    // Set the roundabout_exit_count
    else if (member_name == "roundabout_exit_count") {
      proto_maneuver->set_roundabout_exit_count(member.value.GetUint());
    }

    // Set the depart_instruction
    else if (member_name == "depart_instruction") {
      proto_maneuver->set_depart_instruction(member.value.GetString());
    }

    // Set the verbal_depart_instruction
    else if (member_name == "verbal_depart_instruction") {
      proto_maneuver->set_verbal_depart_instruction(member.value.GetString());
    }

    // Set the arrive_instruction
    else if (member_name == "arrive_instruction") {
      proto_maneuver->set_arrive_instruction(member.value.GetString());
    }

    // Set the verbal_arrive_instruction
    else if (member_name == "verbal_arrive_instruction") {
      proto_maneuver->set_verbal_arrive_instruction(member.value.GetString());
    }

    // Set the transit_info
    else if (member_name == "transit_info") {
      jsonToProtoTransitInfo (member.value, proto_maneuver->mutable_transit_info());
    }

    // Set verbal_multi_cue
    else if (member_name == "verbal_multi_cue") {
      proto_maneuver->set_verbal_multi_cue(member.value.GetBool());
    }

    // Set the travel_mode
    else if (member_name == "travel_mode") {
      proto_maneuver->set_travel_mode(member.value.GetString());
    }

    // Set the travel_type
    else if (member_name == "travel_type") {
      proto_maneuver->set_travel_type(member.value.GetString());
    }
  }
}

void jsonToProtoLeg (const rapidjson::Value& json_leg, Route::Leg* proto_leg) {
  for (const auto& member : json_leg.GetObject()) {
    std::string member_name = member.name.GetString();

    // Set the summary
    if (member_name == "summary") {
      jsonToProtoSummary (member.value, proto_leg->mutable_summary());
    }

    // Set the maneuvers
    if(member_name == "maneuvers") {
      auto proto_maneuvers = proto_leg->mutable_maneuvers();
      for (const auto& maneuver : member.value.GetArray()) {
        auto proto_maneuver = proto_maneuvers->Add();
        jsonToProtoManeuver (maneuver, proto_maneuver);
      }
    }

    // Set the shape
    if (member_name == "shape") {
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
    std::string member_name = member.name.GetString();

    // Set the locations
    if (member_name == "locations") {
      auto proto_locations = proto_trip->mutable_locations();
      for (const auto& loc : member.value.GetArray()) {
        auto proto_loc = proto_locations->Add();
        jsonToProtoLocation (loc, proto_loc);
      }
    }

    // Set the summary
    else if (member_name == "summary") {
      jsonToProtoSummary (member.value, proto_trip->mutable_summary());
    }

    // Set the legs
    else if (member_name == "legs") {
      auto proto_legs = proto_trip->mutable_legs();
      for (const auto& leg: member.value.GetArray()) {
        auto proto_leg = proto_legs->Add();
        jsonToProtoLeg (leg, proto_leg);
      }
    }

    // Set the status_message
    else if (member_name == "status_message") {
      proto_trip->set_status_message(member.value.GetString());
    }

    // Set the status
    else if (member_name == "status") {
      proto_trip->set_status(member.value.GetUint());
    }

    // Set the units
    else if (member_name == "units") {
      proto_trip->set_units(member.value.GetString());
    }

    // Set the language
    else if (member_name == "language") {
      proto_trip->set_language(member.value.GetString());
    }

    // Set the id
    else if (member_name == "id") {
      proto_trip->set_id(member.value.GetString());
    }
  }
}

}
}
