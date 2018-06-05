#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/turn.h"
#include "exception.h"
#include "midgard/aabb2.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "odin/util.h"
#include "route_serializer_osrm.cc"
#include "route_serializer_valhalla.cc"
#include "tyr/serializers.h"

#include <valhalla/proto/directions_options.pb.h>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

namespace {

void jsonToProtoLocation(const rapidjson::Value& json_location, Route::Location* proto_location) {
  // Set the lat
  auto lat_iter = json_location.FindMember("lat");
  if (lat_iter != json_location.MemberEnd()) {
    if (!lat_iter->value.IsFloat()) {
      throw std::runtime_error("lat is not a float.");
    }
    proto_location->set_lat(lat_iter->value.GetFloat());
  }

  // Set the lon
  auto lon_iter = json_location.FindMember("lon");
  if (lon_iter != json_location.MemberEnd()) {
    if (!lon_iter->value.IsFloat()) {
      throw std::runtime_error("lon is not a float.");
    }
    proto_location->set_lon(lon_iter->value.GetFloat());
  }

  // Set the type
  auto type_iter = json_location.FindMember("type");
  if (type_iter != json_location.MemberEnd()) {
    if (!type_iter->value.IsString()) {
      throw std::runtime_error("type is not a string.");
    }
    proto_location->set_type(type_iter->value.GetString());
  }

  // Set the heading
  auto heading_iter = json_location.FindMember("heading");
  if (heading_iter != json_location.MemberEnd()) {
    if (!heading_iter->value.IsUint()) {
      throw std::runtime_error("heading is not a Uint.");
    }
    proto_location->set_heading(heading_iter->value.GetUint());
  }

  // Set the name
  auto name_iter = json_location.FindMember("name");
  if (name_iter != json_location.MemberEnd()) {
    if (!name_iter->value.IsString()) {
      throw std::runtime_error("name is not a string.");
    }
    proto_location->set_name(name_iter->value.GetString());
  }

  // Set the street
  auto street_iter = json_location.FindMember("street");
  if (street_iter != json_location.MemberEnd()) {
    if (!street_iter->value.IsString()) {
      throw std::runtime_error("street is not a string.");
    }
    proto_location->set_street(street_iter->value.GetString());
  }

  // Set the city
  auto city_iter = json_location.FindMember("city");
  if (city_iter != json_location.MemberEnd()) {
    if (!city_iter->value.IsString()) {
      throw std::runtime_error("city is not a string.");
    }
    proto_location->set_city(city_iter->value.GetString());
  }

  // Set the state
  auto state_iter = json_location.FindMember("state");
  if (state_iter != json_location.MemberEnd()) {
    if (!state_iter->value.IsString()) {
      throw std::runtime_error("state is not a string.");
    }
    proto_location->set_state(state_iter->value.GetString());
  }

  // Set the postal_code
  auto postal_code_iter = json_location.FindMember("postal_code");
  if (postal_code_iter != json_location.MemberEnd()) {
    if (!postal_code_iter->value.IsString()) {
      throw std::runtime_error("postal_code is not a string.");
    }
    proto_location->set_postal_code(postal_code_iter->value.GetString());
  }

  // Set the country
  auto country_iter = json_location.FindMember("country");
  if (country_iter != json_location.MemberEnd()) {
    if (!country_iter->value.IsString()) {
      throw std::runtime_error("country is not a string.");
    }
    proto_location->set_country(country_iter->value.GetString());
  }

  // Set the date_time
  auto date_time_iter = json_location.FindMember("date_time");
  if (date_time_iter != json_location.MemberEnd()) {
    if (!date_time_iter->value.IsString()) {
      throw std::runtime_error("date_time is not a string.");
    }
    proto_location->set_date_time(date_time_iter->value.GetString());
  }

  // Set the side_of_street
  auto side_of_street_iter = json_location.FindMember("side_of_street");
  if (side_of_street_iter != json_location.MemberEnd()) {
    if (!side_of_street_iter->value.IsString()) {
      throw std::runtime_error("side_of_street is not a string.");
    }
    proto_location->set_side_of_street(side_of_street_iter->value.GetString());
  }

  // Set the original_index
  auto original_index_iter = json_location.FindMember("original_index");
  if (original_index_iter != json_location.MemberEnd()) {
    if (!original_index_iter->value.IsUint()) {
      throw std::runtime_error("original_index is not a Uint.");
    }
    proto_location->set_original_index(original_index_iter->value.GetUint());
  }
}

void jsonToProtoSummary(const rapidjson::Value& json_summary, Route::Summary* proto_summary) {
  // Set the length
  auto length_iter = json_summary.FindMember("length");
  if (length_iter != json_summary.MemberEnd()) {
    if (!length_iter->value.IsFloat()) {
      throw std::runtime_error("length is not a float.");
    }
    proto_summary->set_length(length_iter->value.GetFloat());
  }

  // Set the time
  auto time_iter = json_summary.FindMember("time");
  if (time_iter != json_summary.MemberEnd()) {
    if (!time_iter->value.IsUint()) {
      throw std::runtime_error("time is not a Uint.");
    }
    proto_summary->set_time(time_iter->value.GetUint());
  }

  // Set the min_lat
  auto min_lat_iter = json_summary.FindMember("min_lat");
  if (min_lat_iter != json_summary.MemberEnd()) {
    if (!min_lat_iter->value.IsFloat()) {
      throw std::runtime_error("min_lat is not a float.");
    }
    proto_summary->set_min_lat(min_lat_iter->value.GetFloat());
  }

  // Set the min_lon
  auto min_lon_iter = json_summary.FindMember("min_lon");
  if (min_lon_iter != json_summary.MemberEnd()) {
    if (!min_lon_iter->value.IsFloat()) {
      throw std::runtime_error("min_lon is not a float.");
    }
    proto_summary->set_min_lon(min_lon_iter->value.GetFloat());
  }

  // Set the max_lat
  auto max_lat_iter = json_summary.FindMember("max_lat");
  if (max_lat_iter != json_summary.MemberEnd()) {
    if (!max_lat_iter->value.IsFloat()) {
      throw std::runtime_error("max_lat is not a float.");
    }
    proto_summary->set_max_lat(max_lat_iter->value.GetFloat());
  }

  // Set the max_lon
  auto max_lon_iter = json_summary.FindMember("max_lon");
  if (max_lon_iter != json_summary.MemberEnd()) {
    if (!max_lon_iter->value.IsFloat()) {
      throw std::runtime_error("max_lon is not a float.");
    }
    proto_summary->set_max_lon(max_lon_iter->value.GetFloat());
  }
}

void jsonToProtoElement(const rapidjson::Value& json_element,
                        Route::Maneuver::Sign::Element* proto_element) {
  // Set the text
  auto text_iter = json_element.FindMember("text");
  if (text_iter != json_element.MemberEnd()) {
    if (!text_iter->value.IsString()) {
      throw std::runtime_error("text is not a string.");
    }
    proto_element->set_text(text_iter->value.GetString());
  }

  // Set the consecutive_count
  auto consecutive_count_iter = json_element.FindMember("consecutive_count");
  if (consecutive_count_iter != json_element.MemberEnd()) {
    if (!consecutive_count_iter->value.IsUint()) {
      throw std::runtime_error("consecutive_count is not a Uint.");
    }
    proto_element->set_consecutive_count(consecutive_count_iter->value.GetUint());
  }
}

void jsonToProtoSign(const rapidjson::Value& json_sign, Route::Maneuver::Sign* proto_sign) {
  // Set the exit_number_elements
  auto exit_number_elements_iter = json_sign.FindMember("exit_number_elements");
  if (exit_number_elements_iter != json_sign.MemberEnd()) {
    if (!exit_number_elements_iter->value.IsArray()) {
      throw std::runtime_error("exit_number_elements is not an array.");
    }
    auto proto_exit_number_elements = proto_sign->mutable_exit_number_elements();
    for (const auto& exit_number_element : exit_number_elements_iter->value.GetArray()) {
      if (!exit_number_element.IsObject()) {
        throw std::runtime_error("exit_number_element is not an object.");
      }
      auto proto_exit_number_element = proto_exit_number_elements->Add();
      jsonToProtoElement(exit_number_element, proto_exit_number_element);
    }
  }

  // Set the exit_branch_elements
  auto exit_branch_elements_iter = json_sign.FindMember("exit_branch_elements");
  if (exit_branch_elements_iter != json_sign.MemberEnd()) {
    if (!exit_branch_elements_iter->value.IsArray()) {
      throw std::runtime_error("exit_branch_element is not an array.");
    }
    auto proto_exit_branch_elements = proto_sign->mutable_exit_branch_elements();
    for (const auto& exit_branch_element : exit_branch_elements_iter->value.GetArray()) {
      if (!exit_branch_element.IsObject()) {
        throw std::runtime_error("exit_branch_element is not an object.");
      }
      auto proto_exit_branch_element = proto_exit_branch_elements->Add();
      jsonToProtoElement(exit_branch_element, proto_exit_branch_element);
    }
  }

  // Set the exit_toward_elements
  auto exit_toward_elements_iter = json_sign.FindMember("exit_toward_elements");
  if (exit_toward_elements_iter != json_sign.MemberEnd()) {
    if (!exit_toward_elements_iter->value.IsArray()) {
      throw std::runtime_error("exit_toward_element is not an array.");
    }
    auto proto_exit_toward_elements = proto_sign->mutable_exit_toward_elements();
    for (const auto& exit_toward_element : exit_toward_elements_iter->value.GetArray()) {
      if (!exit_toward_element.IsObject()) {
        throw std::runtime_error("exit_toward_element is not an object.");
      }
      auto proto_exit_toward_element = proto_exit_toward_elements->Add();
      jsonToProtoElement(exit_toward_element, proto_exit_toward_element);
    }
  }

  // Set the exit_name_elements
  auto exit_name_elements_iter = json_sign.FindMember("exit_name_elements");
  if (exit_name_elements_iter != json_sign.MemberEnd()) {
    if (!exit_name_elements_iter->value.IsArray()) {
      throw std::runtime_error("exit_name_element is not an array.");
    }
    auto proto_exit_name_elements = proto_sign->mutable_exit_name_elements();
    for (const auto& exit_name_element : exit_name_elements_iter->value.GetArray()) {
      if (!exit_name_element.IsObject()) {
        throw std::runtime_error("exit_name_element is not an object.");
      }
      auto proto_exit_name_element = proto_exit_name_elements->Add();
      jsonToProtoElement(exit_name_element, proto_exit_name_element);
    }
  }
}

void jsonToProtoTransitStop(const rapidjson::Value& json_transit_stop,
                            Route::TransitStop* proto_transit_stop) {
  // Set the type
  auto type_iter = json_transit_stop.FindMember("type");
  if (type_iter != json_transit_stop.MemberEnd()) {
    if (!type_iter->value.IsString()) {
      throw std::runtime_error("type is not a string.");
    }
    proto_transit_stop->set_type(type_iter->value.GetString());
  }

  // Set the onestop_id
  auto onestop_id_iter = json_transit_stop.FindMember("onestop_id");
  if (onestop_id_iter != json_transit_stop.MemberEnd()) {
    if (!onestop_id_iter->value.IsString()) {
      throw std::runtime_error("onestop_id is not a string.");
    }
    proto_transit_stop->set_onestop_id(onestop_id_iter->value.GetString());
  }

  // Set the name
  auto name_iter = json_transit_stop.FindMember("name");
  if (name_iter != json_transit_stop.MemberEnd()) {
    if (!name_iter->value.IsString()) {
      throw std::runtime_error("name is not a string.");
    }
    proto_transit_stop->set_name(name_iter->value.GetString());
  }

  // Set the arrival_date_time
  auto arrival_date_time_iter = json_transit_stop.FindMember("arrival_date_time");
  if (arrival_date_time_iter != json_transit_stop.MemberEnd()) {
    if (!arrival_date_time_iter->value.IsString()) {
      throw std::runtime_error("arrival_date_time is not a string.");
    }
    proto_transit_stop->set_arrival_date_time(arrival_date_time_iter->value.GetString());
  }

  // Set the departure_date_time
  auto departure_date_time_iter = json_transit_stop.FindMember("departure_date_time");
  if (departure_date_time_iter != json_transit_stop.MemberEnd()) {
    if (!departure_date_time_iter->value.IsString()) {
      throw std::runtime_error("departure_date_time is not a string.");
    }
    proto_transit_stop->set_departure_date_time(departure_date_time_iter->value.GetString());
  }

  // Set is_parent_stop
  auto is_parent_stop_iter = json_transit_stop.FindMember("is_parent_stop");
  if (is_parent_stop_iter != json_transit_stop.MemberEnd()) {
    if (!is_parent_stop_iter->value.IsBool()) {
      throw std::runtime_error("is_parent_stop is not a bool.");
    }
    proto_transit_stop->set_is_parent_stop(is_parent_stop_iter->value.GetBool());
  }

  // Set assumed_schedule
  auto assumed_schedule_iter = json_transit_stop.FindMember("assumed_schedule");
  if (assumed_schedule_iter != json_transit_stop.MemberEnd()) {
    if (!assumed_schedule_iter->value.IsBool()) {
      throw std::runtime_error("assumed_schedule is not a bool.");
    }
    proto_transit_stop->set_assumed_schedule(assumed_schedule_iter->value.GetBool());
  }

  // Set the lat
  auto lat_iter = json_transit_stop.FindMember("lat");
  if (lat_iter != json_transit_stop.MemberEnd()) {
    if (!lat_iter->value.IsFloat()) {
      throw std::runtime_error("lat is not a float.");
    }
    proto_transit_stop->set_lat(lat_iter->value.GetFloat());
  }

  // Set the lon
  auto lon_iter = json_transit_stop.FindMember("lon");
  if (lon_iter != json_transit_stop.MemberEnd()) {
    if (!lon_iter->value.IsFloat()) {
      throw std::runtime_error("lon is not a float.");
    }
    proto_transit_stop->set_lon(lon_iter->value.GetFloat());
  }
}

void jsonToProtoTransitInfo(const rapidjson::Value& json_transit_info,
                            Route::TransitInfo* proto_transit_info) {
  // Set the onestop_id
  auto onestop_id_iter = json_transit_info.FindMember("onestop_id");
  if (onestop_id_iter != json_transit_info.MemberEnd()) {
    if (!onestop_id_iter->value.IsString()) {
      throw std::runtime_error("onestop_id is not a string.");
    }
    proto_transit_info->set_onestop_id(onestop_id_iter->value.GetString());
  }

  // Set the short_name
  auto short_name_iter = json_transit_info.FindMember("short_name");
  if (short_name_iter != json_transit_info.MemberEnd()) {
    if (!short_name_iter->value.IsString()) {
      throw std::runtime_error("short_name is not a string.");
    }
    proto_transit_info->set_short_name(short_name_iter->value.GetString());
  }

  // Set the long_name
  auto long_name_iter = json_transit_info.FindMember("long_name");
  if (long_name_iter != json_transit_info.MemberEnd()) {
    if (!long_name_iter->value.IsString()) {
      throw std::runtime_error("long_name is not a string.");
    }
    proto_transit_info->set_long_name(long_name_iter->value.GetString());
  }

  // Set the headsign
  auto headsign_iter = json_transit_info.FindMember("headsign");
  if (headsign_iter != json_transit_info.MemberEnd()) {
    if (!headsign_iter->value.IsString()) {
      throw std::runtime_error("headsign is not a string.");
    }
    proto_transit_info->set_headsign(headsign_iter->value.GetString());
  }

  // Set the color
  auto color_iter = json_transit_info.FindMember("color");
  if (color_iter != json_transit_info.MemberEnd()) {
    if (!color_iter->value.IsUint()) {
      throw std::runtime_error("color is not a Uint.");
    }
    proto_transit_info->set_color(color_iter->value.GetUint());
  }

  // Set the text_color
  auto text_color_iter = json_transit_info.FindMember("text_color");
  if (text_color_iter != json_transit_info.MemberEnd()) {
    if (!text_color_iter->value.IsUint()) {
      throw std::runtime_error("text_color is not a Uint.");
    }
    proto_transit_info->set_text_color(text_color_iter->value.GetUint());
  }

  // Set the description
  auto description_iter = json_transit_info.FindMember("description");
  if (description_iter != json_transit_info.MemberEnd()) {
    if (!description_iter->value.IsString()) {
      throw std::runtime_error("description is not a string.");
    }
    proto_transit_info->set_description(description_iter->value.GetString());
  }

  // Set the operator_onestop_id
  auto operator_onestop_id_iter = json_transit_info.FindMember("operator_onestop_id");
  if (operator_onestop_id_iter != json_transit_info.MemberEnd()) {
    if (!operator_onestop_id_iter->value.IsString()) {
      throw std::runtime_error("operator_onestop_id is not a string.");
    }
    proto_transit_info->set_operator_onestop_id(operator_onestop_id_iter->value.GetString());
  }

  // Set the operator_name
  auto operator_name_iter = json_transit_info.FindMember("operator_name");
  if (operator_name_iter != json_transit_info.MemberEnd()) {
    if (!operator_name_iter->value.IsString()) {
      throw std::runtime_error("operator_name is not a string.");
    }
    proto_transit_info->set_operator_name(operator_name_iter->value.GetString());
  }

  // Set the operator_url
  auto operator_url_iter = json_transit_info.FindMember("operator_url");
  if (operator_url_iter != json_transit_info.MemberEnd()) {
    if (!operator_url_iter->value.IsString()) {
      throw std::runtime_error("operator_url is not a string.");
    }
    proto_transit_info->set_operator_url(operator_url_iter->value.GetString());
  }

  // Set the transit_stops
  auto transit_stops_iter = json_transit_info.FindMember("transit_stops");
  if (transit_stops_iter != json_transit_info.MemberEnd()) {
    if (!transit_stops_iter->value.IsArray()) {
      throw std::runtime_error("transit_stops is not an array.");
    }
    auto proto_transit_stops = proto_transit_info->mutable_transit_stops();
    for (const auto& transit_stop : transit_stops_iter->value.GetArray()) {
      if (!transit_stop.IsObject()) {
        throw std::runtime_error("transit_stop is not an object.");
      }
      auto proto_transit_stop = proto_transit_stops->Add();
      jsonToProtoTransitStop(transit_stop, proto_transit_stop);
    }
  }
}

void jsonToProtoManeuver(const rapidjson::Value& json_maneuver, Route::Maneuver* proto_maneuver) {
  // Set the type
  auto type_iter = json_maneuver.FindMember("type");
  if (type_iter != json_maneuver.MemberEnd()) {
    if (!type_iter->value.IsUint()) {
      throw std::runtime_error("type is not a Uint");
    }
    proto_maneuver->set_type(type_iter->value.GetUint());
  }

  // Set the instruction
  auto instruction_iter = json_maneuver.FindMember("instruction");
  if (instruction_iter != json_maneuver.MemberEnd()) {
    if (!instruction_iter->value.IsString()) {
      throw std::runtime_error("instruction is not a string.");
    }
    proto_maneuver->set_instruction(instruction_iter->value.GetString());
  }

  // Set the street_names
  auto street_names_iter = json_maneuver.FindMember("street_names");
  if (street_names_iter != json_maneuver.MemberEnd()) {
    if (!street_names_iter->value.IsArray()) {
      throw std::runtime_error("street_names is not an array.");
    }
    auto proto_street_names = proto_maneuver->mutable_street_names();
    for (const auto& street_name : street_names_iter->value.GetArray()) {
      if (!street_name.IsString()) {
        throw std::runtime_error("street_name is not a string.");
      }
      auto proto_street_name = proto_street_names->Add();
      *proto_street_name = street_name.GetString();
    }
  }

  // Set the length
  auto length_iter = json_maneuver.FindMember("length");
  if (length_iter != json_maneuver.MemberEnd()) {
    if (!length_iter->value.IsFloat()) {
      throw std::runtime_error("length is not a float.");
    }
    proto_maneuver->set_length(length_iter->value.GetFloat());
  }

  // Set the time
  auto time_iter = json_maneuver.FindMember("time");
  if (time_iter != json_maneuver.MemberEnd()) {
    if (!time_iter->value.IsUint()) {
      throw std::runtime_error("time is not a Uint.");
    }
    proto_maneuver->set_time(time_iter->value.GetUint());
  }

  // Set the begin_cardinal_direction
  auto begin_cardinal_direction_iter = json_maneuver.FindMember("begin_cardinal_direction");
  if (begin_cardinal_direction_iter != json_maneuver.MemberEnd()) {
    if (!begin_cardinal_direction_iter->value.IsString()) {
      throw std::runtime_error("begin_cardinal_direction is not a string.");
    }
    proto_maneuver->set_begin_cardinal_direction(begin_cardinal_direction_iter->value.GetString());
  }

  // Set the begin_heading
  auto begin_heading_iter = json_maneuver.FindMember("begin_heading");
  if (begin_heading_iter != json_maneuver.MemberEnd()) {
    if (!begin_heading_iter->value.IsUint()) {
      throw std::runtime_error("begin_heading is not a Uint.");
    }
    proto_maneuver->set_begin_heading(begin_heading_iter->value.GetUint());
  }

  // Set the begin_shape_index
  auto begin_shape_index_iter = json_maneuver.FindMember("begin_shape_index");
  if (begin_shape_index_iter != json_maneuver.MemberEnd()) {
    if (!begin_shape_index_iter->value.IsUint()) {
      throw std::runtime_error("begin_shape_index is not a Uint.");
    }
    proto_maneuver->set_begin_shape_index(begin_shape_index_iter->value.GetUint());
  }

  // Set the end_shape_index
  auto end_shape_index_iter = json_maneuver.FindMember("end_shape_index");
  if (end_shape_index_iter != json_maneuver.MemberEnd()) {
    if (!end_shape_index_iter->value.IsUint()) {
      throw std::runtime_error("end_shape_index is not a Uint.");
    }
    proto_maneuver->set_end_shape_index(end_shape_index_iter->value.GetUint());
  }

  // Set toll
  auto toll_iter = json_maneuver.FindMember("toll");
  if (toll_iter != json_maneuver.MemberEnd()) {
    if (!toll_iter->value.IsBool()) {
      throw std::runtime_error("toll is not a bool.");
    }
    proto_maneuver->set_toll(toll_iter->value.GetBool());
  }

  // Set rough
  auto rough_iter = json_maneuver.FindMember("rough");
  if (rough_iter != json_maneuver.MemberEnd()) {
    if (!rough_iter->value.IsBool()) {
      throw std::runtime_error("rough is not a bool.");
    }
    proto_maneuver->set_rough(rough_iter->value.GetBool());
  }

  // Set the verbal_transition_alert_instruction
  auto verbal_transition_alert_instruction_iter =
      json_maneuver.FindMember("verbal_transition_alert_instruction");
  if (verbal_transition_alert_instruction_iter != json_maneuver.MemberEnd()) {
    if (!verbal_transition_alert_instruction_iter->value.IsString()) {
      throw std::runtime_error("verbal_transition_alert_instruction is not a string.");
    }
    proto_maneuver->set_verbal_transition_alert_instruction(
        verbal_transition_alert_instruction_iter->value.GetString());
  }

  // Set the verbal_pre_transition_instruction
  auto verbal_pre_transition_instruction_iter =
      json_maneuver.FindMember("verbal_pre_transition_instruction");
  if (verbal_pre_transition_instruction_iter != json_maneuver.MemberEnd()) {
    if (!verbal_pre_transition_instruction_iter->value.IsString()) {
      throw std::runtime_error("verbal_pre_transition_instruction is not a string.");
    }
    proto_maneuver->set_verbal_pre_transition_instruction(
        verbal_pre_transition_instruction_iter->value.GetString());
  }

  // Set the verbal_post_transition_instruction
  auto verbal_post_transition_instruction_iter =
      json_maneuver.FindMember("verbal_post_transition_instruction");
  if (verbal_post_transition_instruction_iter != json_maneuver.MemberEnd()) {
    if (!verbal_post_transition_instruction_iter->value.IsString()) {
      throw std::runtime_error("verbal_post_transition_instruction is not a string.");
    }
    proto_maneuver->set_verbal_post_transition_instruction(
        verbal_post_transition_instruction_iter->value.GetString());
  }

  // Set the begin_street_names
  auto begin_street_names_iter = json_maneuver.FindMember("begin_street_names");
  if (begin_street_names_iter != json_maneuver.MemberEnd()) {
    if (!begin_street_names_iter->value.IsArray()) {
      throw std::runtime_error("begin_street_names is not an array.");
    }
    auto proto_begin_street_names = proto_maneuver->mutable_begin_street_names();
    for (const auto& begin_street_name : begin_street_names_iter->value.GetArray()) {
      if (!begin_street_name.IsString()) {
        throw std::runtime_error("begin_street_name is not a string.");
      }
      auto proto_begin_street_name = proto_begin_street_names->Add();
      *proto_begin_street_name = begin_street_name.GetString();
    }
  }

  // Set the sign
  auto sign_iter = json_maneuver.FindMember("sign");
  if (sign_iter != json_maneuver.MemberEnd()) {
    if (!sign_iter->value.IsObject()) {
      throw std::runtime_error("sign is not an object.");
    }
    jsonToProtoSign(sign_iter->value, proto_maneuver->mutable_sign());
  }

  // Set the roundabout_exit_count
  auto roundabout_exit_count_iter = json_maneuver.FindMember("roundabout_exit_count");
  if (roundabout_exit_count_iter != json_maneuver.MemberEnd()) {
    if (!roundabout_exit_count_iter->value.IsUint()) {
      throw std::runtime_error("roundabout_exit_count is not a Uint.");
    }
    proto_maneuver->set_roundabout_exit_count(roundabout_exit_count_iter->value.GetUint());
  }

  // Set the depart_instruction
  auto depart_instruction_iter = json_maneuver.FindMember("depart_instruction");
  if (depart_instruction_iter != json_maneuver.MemberEnd()) {
    if (!depart_instruction_iter->value.IsString()) {
      throw std::runtime_error("depart_instruction is not a string.");
    }
    proto_maneuver->set_depart_instruction(depart_instruction_iter->value.GetString());
  }

  // Set the verbal_depart_instruction
  auto verbal_depart_instruction_iter = json_maneuver.FindMember("verbal_depart_instruction");
  if (verbal_depart_instruction_iter != json_maneuver.MemberEnd()) {
    if (!verbal_depart_instruction_iter->value.IsString()) {
      throw std::runtime_error("verbal_depart_instruction is not a string.");
    }
    proto_maneuver->set_verbal_depart_instruction(verbal_depart_instruction_iter->value.GetString());
  }

  // Set the arrive_instruction
  auto arrive_instruction_iter = json_maneuver.FindMember("arrive_instruction");
  if (arrive_instruction_iter != json_maneuver.MemberEnd()) {
    if (!arrive_instruction_iter->value.IsString()) {
      throw std::runtime_error("arrive_instruction is not a string.");
    }
    proto_maneuver->set_arrive_instruction(arrive_instruction_iter->value.GetString());
  }

  // Set the verbal_arrive_instruction
  auto verbal_arrive_instruction_iter = json_maneuver.FindMember("verbal_arrive_instruction");
  if (verbal_arrive_instruction_iter != json_maneuver.MemberEnd()) {
    if (!verbal_arrive_instruction_iter->value.IsString()) {
      throw std::runtime_error("verbal_arrive_instruction is not a string.");
    }
    proto_maneuver->set_verbal_arrive_instruction(verbal_arrive_instruction_iter->value.GetString());
  }

  // Set the transit_info
  auto transit_info_iter = json_maneuver.FindMember("transit_info");
  if (transit_info_iter != json_maneuver.MemberEnd()) {
    if (!transit_info_iter->value.IsObject()) {
      throw std::runtime_error("transit_info is not an object.");
    }
    jsonToProtoTransitInfo(transit_info_iter->value, proto_maneuver->mutable_transit_info());
  }

  // Set verbal_multi_cue
  auto verbal_multi_cue_iter = json_maneuver.FindMember("verbal_multi_cue");
  if (verbal_multi_cue_iter != json_maneuver.MemberEnd()) {
    if (!verbal_multi_cue_iter->value.IsBool()) {
      throw std::runtime_error("verbal_multi_cue is not a bool.");
    }
    proto_maneuver->set_verbal_multi_cue(verbal_multi_cue_iter->value.GetBool());
  }

  // Set the travel_mode
  auto travel_mode_iter = json_maneuver.FindMember("travel_mode");
  if (travel_mode_iter != json_maneuver.MemberEnd()) {
    if (!travel_mode_iter->value.IsString()) {
      throw std::runtime_error("travel_mode is not a string.");
    }
    proto_maneuver->set_travel_mode(travel_mode_iter->value.GetString());
  }

  // Set the travel_type
  auto travel_type_iter = json_maneuver.FindMember("travel_type");
  if (travel_type_iter != json_maneuver.MemberEnd()) {
    if (!travel_type_iter->value.IsString()) {
      throw std::runtime_error("travel_type is not a string.");
    }
    proto_maneuver->set_travel_type(travel_type_iter->value.GetString());
  }
}

void jsonToProtoLeg(const rapidjson::Value& json_leg, Route::Leg* proto_leg) {
  // Set the summary
  auto summary_iter = json_leg.FindMember("summary");
  if (summary_iter != json_leg.MemberEnd()) {
    if (!summary_iter->value.IsObject()) {
      throw std::runtime_error("summary is not an object.");
    }
    jsonToProtoSummary(summary_iter->value, proto_leg->mutable_summary());
  }

  // Set the maneuvers
  auto maneuvers_iter = json_leg.FindMember("maneuvers");
  if (maneuvers_iter != json_leg.MemberEnd()) {
    if (!maneuvers_iter->value.IsArray()) {
      throw std::runtime_error("maneuvers is not an array.");
    }
    auto proto_maneuvers = proto_leg->mutable_maneuvers();
    for (const auto& maneuver : maneuvers_iter->value.GetArray()) {
      if (!maneuver.IsObject()) {
        throw std::runtime_error("maneuver is not an object.");
      }
      auto proto_maneuver = proto_maneuvers->Add();
      jsonToProtoManeuver(maneuver, proto_maneuver);
    }
  }

  // Set the shape
  auto shape_iter = json_leg.FindMember("shape");
  if (shape_iter != json_leg.MemberEnd()) {
    if (!shape_iter->value.IsString()) {
      throw std::runtime_error("shape is not a string.");
    }
    proto_leg->set_shape(shape_iter->value.GetString());
  }
}

/**
 * Returns GPX formatted route responses given the legs of the route
 * @param  legs  The legs of the route
 * @return the gpx string
 */
std::string pathToGPX(const std::list<odin::TripPath>& legs) {
  // start the gpx, we'll use 6 digits of precision
  std::stringstream gpx;
  gpx << std::setprecision(6) << std::fixed;
  gpx << R"(<?xml version="1.0" encoding="UTF-8" standalone="no"?><gpx version="1.1" creator="libvalhalla"><metadata/>)";

  // for each leg
  for (const auto& leg : legs) {
    // decode the shape for this leg
    auto wpts = midgard::decode<std::vector<PointLL>>(leg.shape());

    // throw the shape points in as way points
    // TODO: add time to each, need transition time at nodes
    for (const auto& wpt : wpts) {
      gpx << R"(<wpt lon=")" << wpt.first << R"(" lat=")" << wpt.second << R"("></wpt>)";
    }

    // throw the intersections in as route points
    // TODO: add time to each, need transition time at nodes
    gpx << "<rte>";
    uint64_t last_id = -1;
    for (const auto& node : leg.node()) {
      // if this isnt the last node we want the begin shape index of the edge
      size_t shape_idx = wpts.size() - 1;
      if (node.has_edge()) {
        last_id = node.edge().way_id();
        shape_idx = node.edge().begin_shape_index();
      }

      // output this intersection (note that begin and end points may not be intersections)
      const auto& rtept = wpts[shape_idx];
      gpx << R"(<rtept lon=")" << rtept.first << R"(" lat=")" << rtept.second << R"(">)"
          << "<name>" << last_id << "</name></rtept>";
    }
    gpx << "</rte>";
  }

  // give it back as a string
  gpx << "</gpx>";
  return gpx.str();
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeDirections(const valhalla_request_t& request,
                                const std::list<TripPath>& path_legs,
                                const std::list<TripDirections>& directions_legs) {
  // serialize them
  switch (request.options.format()) {
    case DirectionsOptions_Format_osrm:
      return osrm_serializers::serialize(request.options, path_legs, directions_legs);
    case DirectionsOptions_Format_gpx:
      return pathToGPX(path_legs);
    case DirectionsOptions_Format_json:
      return valhalla_serializers::serialize(request.options, directions_legs);
    default:
      throw;
  }
}

void jsonToProtoRoute(const std::string& json_route, Route& proto_route) {
  rapidjson::Document d;
  d.Parse(json_route.c_str());
  if (d.HasParseError()) {
    throw std::runtime_error("String to document parsing failed.");
  }

  // Grab the trip object from JSON
  auto json_trip_iter = d.FindMember("trip");
  if (json_trip_iter == d.MemberEnd()) {
    return;
  } else if (!json_trip_iter->value.IsObject()) {
    throw std::runtime_error("trip is not an object.");
  }

  if (proto_route.has_trip()) {
    proto_route.clear_trip();
  }

  // Get the empty trip object and start setting it's fields
  Route::Trip* proto_trip = proto_route.mutable_trip();

  // Set the locations
  auto locations_iter = json_trip_iter->value.FindMember("locations");
  if (locations_iter != json_trip_iter->value.MemberEnd()) {
    if (!locations_iter->value.IsArray()) {
      throw std::runtime_error("locations is not an array.");
    }
    auto proto_locations = proto_trip->mutable_locations();
    for (const auto& loc : locations_iter->value.GetArray()) {
      if (!loc.IsObject()) {
        throw std::runtime_error("location is not an object.");
      }
      auto proto_loc = proto_locations->Add();
      jsonToProtoLocation(loc, proto_loc);
    }
  }

  // Set the summary
  auto summary_iter = json_trip_iter->value.FindMember("summary");
  if (summary_iter != json_trip_iter->value.MemberEnd()) {
    if (!summary_iter->value.IsObject()) {
      throw std::runtime_error("summary is not an object.");
    }
    jsonToProtoSummary(summary_iter->value, proto_trip->mutable_summary());
  }

  // Set the legs
  auto legs_iter = json_trip_iter->value.FindMember("legs");
  if (legs_iter != json_trip_iter->value.MemberEnd()) {
    if (!legs_iter->value.IsArray()) {
      throw std::runtime_error("legs is not an array.");
    }
    auto proto_legs = proto_trip->mutable_legs();
    for (const auto& leg : legs_iter->value.GetArray()) {
      if (!leg.IsObject()) {
        throw std::runtime_error("leg is not an object.");
      }
      auto proto_leg = proto_legs->Add();
      jsonToProtoLeg(leg, proto_leg);
    }
  }

  // Set the status_message
  auto status_message_iter = json_trip_iter->value.FindMember("status_message");
  if (status_message_iter != json_trip_iter->value.MemberEnd()) {
    if (!status_message_iter->value.IsString()) {
      throw std::runtime_error("status_message is not a string.");
    }
    proto_trip->set_status_message(status_message_iter->value.GetString());
  }

  // Set the status
  auto status_iter = json_trip_iter->value.FindMember("status");
  if (status_iter != json_trip_iter->value.MemberEnd()) {
    if (!status_iter->value.IsUint()) {
      throw std::runtime_error("status is not a Uint.");
    }
    proto_trip->set_status(status_iter->value.GetUint());
  }

  // Set the units
  auto units_iter = json_trip_iter->value.FindMember("units");
  if (units_iter != json_trip_iter->value.MemberEnd()) {
    if (!units_iter->value.IsString()) {
      throw std::runtime_error("units is not a string.");
    }
    proto_trip->set_units(units_iter->value.GetString());
  }

  // Set the language
  auto language_iter = json_trip_iter->value.FindMember("language");
  if (language_iter != json_trip_iter->value.MemberEnd()) {
    if (!language_iter->value.IsString()) {
      throw std::runtime_error("language is not a string.");
    }
    proto_trip->set_language(language_iter->value.GetString());
  }

  // Set the id
  auto id_iter = json_trip_iter->value.FindMember("id");
  if (id_iter != json_trip_iter->value.MemberEnd()) {
    if (!id_iter->value.IsString()) {
      throw std::runtime_error("id is not a string.");
    }
    proto_trip->set_id(id_iter->value.GetString());
  }
}

} // namespace tyr
} // namespace valhalla
