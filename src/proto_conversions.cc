#include "proto_conversions.h"

using namespace valhalla;

namespace valhalla {

std::string incidentTypeToString(const TripLeg_Node_Incident_Type& incident_type) {
  switch (incident_type) {
    case TripLeg_Node_Incident_Type_NOT_SET:
      return "not_set";
      break;
    case TripLeg_Node_Incident_Type_ACCIDENT:
      return "accident";
      break;
    case TripLeg_Node_Incident_Type_CONGESTION:
      return "congestion";
      break;
    case TripLeg_Node_Incident_Type_CONSTRUCTION:
      return "construction";
      break;
    case TripLeg_Node_Incident_Type_DISABLED_VEHICLE:
      return "disabled_vehicle";
      break;
    case TripLeg_Node_Incident_Type_LANE_RESTRICTION:
      return "lane_restriction";
      break;
    case TripLeg_Node_Incident_Type_MASS_TRANSIT:
      return "mass_transit";
      break;
    case TripLeg_Node_Incident_Type_MISCELLANEOUS:
      return "miscellaneous";
      break;
    case TripLeg_Node_Incident_Type_OTHER_NEWS:
      return "other_news";
      break;
    case TripLeg_Node_Incident_Type_PLANNED_EVENT:
      return "planned_event";
      break;
    case TripLeg_Node_Incident_Type_ROAD_CLOSURE:
      return "road_closure";
      break;
    case TripLeg_Node_Incident_Type_ROAD_HAZARD:
      return "road_hazard";
      break;
    case TripLeg_Node_Incident_Type_WEATHER:
      return "weather";
      break;
  };
  throw std::runtime_error("Unhandled case in incidentTypeToString: " +
                           std::to_string(incident_type));
}

bool Options_Action_Enum_Parse(const std::string& action, Options::Action* a) {
  static const std::unordered_map<std::string, Options::Action> actions{
      {"route", Options::route},
      {"locate", Options::locate},
      {"sources_to_targets", Options::sources_to_targets},
      {"optimized_route", Options::optimized_route},
      {"isochrone", Options::isochrone},
      {"trace_route", Options::trace_route},
      {"trace_attributes", Options::trace_attributes},
      {"height", Options::height},
      {"transit_available", Options::transit_available},
      {"expansion", Options::expansion},
  };
  auto i = actions.find(action);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

const std::string& Options_Action_Enum_Name(const Options::Action action) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> actions{
      {Options::route, "route"},
      {Options::locate, "locate"},
      {Options::sources_to_targets, "sources_to_targets"},
      {Options::optimized_route, "optimized_route"},
      {Options::isochrone, "isochrone"},
      {Options::trace_route, "trace_route"},
      {Options::trace_attributes, "trace_attributes"},
      {Options::height, "height"},
      {Options::transit_available, "transit_available"},
      {Options::expansion, "expansion"},
  };
  auto i = actions.find(action);
  return i == actions.cend() ? empty : i->second;
}

bool Costing_Enum_Parse(const std::string& costing, Costing* c) {
  static const std::unordered_map<std::string, Costing> costings{
      {"auto", Costing::auto_},
      {"auto_shorter", Costing::auto_shorter},
      {"bicycle", Costing::bicycle},
      {"bus", Costing::bus},
      {"hov", Costing::hov},
      {"taxi", Costing::taxi},
      {"motor_scooter", Costing::motor_scooter},
      {"multimodal", Costing::multimodal},
      {"pedestrian", Costing::pedestrian},
      {"transit", Costing::transit},
      {"truck", Costing::truck},
      {"motorcycle", Costing::motorcycle},
      {"auto_data_fix", Costing::auto_data_fix},
      {"none", Costing::none_},
      {"", Costing::none_},
      {"bikeshare", Costing::bikeshare},
  };
  auto i = costings.find(costing);
  if (i == costings.cend())
    return false;
  *c = i->second;
  return true;
}

const std::string& Costing_Enum_Name(const Costing costing) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> costings{
      {Costing::auto_, "auto"},
      {Costing::auto_shorter, "auto_shorter"},
      {Costing::bicycle, "bicycle"},
      {Costing::bus, "bus"},
      {Costing::hov, "hov"},
      {Costing::taxi, "taxi"},
      {Costing::motor_scooter, "motor_scooter"},
      {Costing::multimodal, "multimodal"},
      {Costing::pedestrian, "pedestrian"},
      {Costing::transit, "transit"},
      {Costing::truck, "truck"},
      {Costing::motorcycle, "motorcycle"},
      {Costing::auto_data_fix, "auto_data_fix"},
      {Costing::none_, "none"},
      {Costing::bikeshare, "bikeshare"},
  };
  auto i = costings.find(costing);
  return i == costings.cend() ? empty : i->second;
}

bool ShapeMatch_Enum_Parse(const std::string& match, ShapeMatch* s) {
  static const std::unordered_map<std::string, ShapeMatch> matches{
      {"edge_walk", ShapeMatch::edge_walk},
      {"map_snap", ShapeMatch::map_snap},
      {"walk_or_snap", ShapeMatch::walk_or_snap},
  };
  auto i = matches.find(match);
  if (i == matches.cend())
    return false;
  *s = i->second;
  return true;
}

const std::string& ShapeMatch_Enum_Name(const ShapeMatch match) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> matches{
      {ShapeMatch::edge_walk, "edge_walk"},
      {ShapeMatch::map_snap, "map_snap"},
      {ShapeMatch::walk_or_snap, "walk_or_snap"},
  };
  auto i = matches.find(match);
  return i == matches.cend() ? empty : i->second;
}

bool Options_Format_Enum_Parse(const std::string& format, Options::Format* f) {
  static const std::unordered_map<std::string, Options::Format> formats{
      {"json", Options::json},
      {"gpx", Options::gpx},
      {"osrm", Options::osrm},
  };
  auto i = formats.find(format);
  if (i == formats.cend())
    return false;
  *f = i->second;
  return true;
}

const std::string& Options_Format_Enum_Name(const Options::Format match) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> formats{
      {Options::json, "json"},
      {Options::gpx, "gpx"},
      {Options::osrm, "osrm"},
  };
  auto i = formats.find(match);
  return i == formats.cend() ? empty : i->second;
}

const std::string& Options_Units_Enum_Name(const Options::Units unit) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> units{
      {Options::kilometers, "kilometers"},
      {Options::miles, "miles"},
  };
  auto i = units.find(unit);
  return i == units.cend() ? empty : i->second;
}

bool FilterAction_Enum_Parse(const std::string& action, FilterAction* a) {
  static const std::unordered_map<std::string, FilterAction> actions{
      {"exclude", FilterAction::exclude},
      {"include", FilterAction::include},
  };
  auto i = actions.find(action);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

const std::string& FilterAction_Enum_Name(const FilterAction action) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> actions{
      {FilterAction::exclude, "exclude"},
      {FilterAction::include, "include"},
  };
  auto i = actions.find(action);
  return i == actions.cend() ? empty : i->second;
}

bool DirectionsType_Enum_Parse(const std::string& dtype, DirectionsType* t) {
  static const std::unordered_map<std::string, DirectionsType> types{
      {"none", DirectionsType::none},
      {"maneuvers", DirectionsType::maneuvers},
      {"instructions", DirectionsType::instructions},
  };
  auto i = types.find(dtype);
  if (i == types.cend())
    return false;
  *t = i->second;
  return true;
}

bool PreferredSide_Enum_Parse(const std::string& pside, valhalla::Location::PreferredSide* p) {
  static const std::unordered_map<std::string, valhalla::Location::PreferredSide> types{
      {"either", valhalla::Location::either},
      {"same", valhalla::Location::same},
      {"opposite", valhalla::Location::opposite},
  };
  auto i = types.find(pside);
  if (i == types.cend())
    return false;
  *p = i->second;
  return true;
}

bool RoadClass_Enum_Parse(const std::string& rc_name, valhalla::RoadClass* rc) {
  static const std::unordered_map<std::string, valhalla::RoadClass> types{
      {"motorway", valhalla::RoadClass::kMotorway},
      {"trunk", valhalla::RoadClass::kTrunk},
      {"primary", valhalla::RoadClass::kPrimary},
      {"secondary", valhalla::RoadClass::kSecondary},
      {"tertiary", valhalla::RoadClass::kTertiary},
      {"unclassified", valhalla::RoadClass::kUnclassified},
      {"residential", valhalla::RoadClass::kResidential},
      {"service_other", valhalla::RoadClass::kServiceOther},
  };
  auto i = types.find(rc_name);
  if (i == types.cend())
    return false;
  *rc = i->second;
  return true;
}
} // namespace valhalla
