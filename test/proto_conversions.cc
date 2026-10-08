#include "proto_conversions.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

TEST(ProtoConversions, IncidentType) {
  const std::vector<std::pair<IncidentsTile::Metadata::Type, std::string>> types = {
      {IncidentsTile::Metadata::ACCIDENT, "accident"},
      {IncidentsTile::Metadata::CONGESTION, "congestion"},
      {IncidentsTile::Metadata::CONSTRUCTION, "construction"},
      {IncidentsTile::Metadata::DISABLED_VEHICLE, "disabled_vehicle"},
      {IncidentsTile::Metadata::LANE_RESTRICTION, "lane_restriction"},
      {IncidentsTile::Metadata::MASS_TRANSIT, "mass_transit"},
      {IncidentsTile::Metadata::MISCELLANEOUS, "miscellaneous"},
      {IncidentsTile::Metadata::OTHER_NEWS, "other_news"},
      {IncidentsTile::Metadata::PLANNED_EVENT, "planned_event"},
      {IncidentsTile::Metadata::ROAD_CLOSURE, "road_closure"},
      {IncidentsTile::Metadata::ROAD_HAZARD, "road_hazard"},
      {IncidentsTile::Metadata::WEATHER, "weather"},
  };
  for (const auto& [type, name] : types)
    EXPECT_EQ(incidentTypeToString(type), name);

  EXPECT_THROW(incidentTypeToString(static_cast<IncidentsTile::Metadata::Type>(1000)),
               std::runtime_error);
}

TEST(ProtoConversions, IncidentImpact) {
  EXPECT_EQ(incidentImpactToString(IncidentsTile::Metadata::UNKNOWN), "unknown");
  EXPECT_EQ(incidentImpactToString(IncidentsTile::Metadata::CRITICAL), "critical");
  EXPECT_EQ(incidentImpactToString(IncidentsTile::Metadata::MAJOR), "major");
  EXPECT_EQ(incidentImpactToString(IncidentsTile::Metadata::MINOR), "minor");
  EXPECT_EQ(incidentImpactToString(IncidentsTile::Metadata::LOW), "low");
  EXPECT_EQ(incidentImpactToString(static_cast<IncidentsTile::Metadata::Impact>(1000)),
            "UNHANDLED_CASE");
}

TEST(ProtoConversions, ShapeMatch) {
  for (const auto* name : {"edge_walk", "map_snap", "walk_or_snap"}) {
    ShapeMatch match;
    ASSERT_TRUE(ShapeMatch_Enum_Parse(name, &match)) << name;
    EXPECT_EQ(ShapeMatch_Enum_Name(match), name);
  }
  ShapeMatch match;
  EXPECT_FALSE(ShapeMatch_Enum_Parse("snap_or_walk", &match));
  EXPECT_EQ(ShapeMatch_Enum_Name(static_cast<ShapeMatch>(1000)), "");
}

TEST(ProtoConversions, FilterAction) {
  for (const auto* name : {"exclude", "include"}) {
    FilterAction action;
    ASSERT_TRUE(FilterAction_Enum_Parse(name, &action)) << name;
    EXPECT_EQ(FilterAction_Enum_Name(action), name);
  }
  FilterAction action;
  EXPECT_FALSE(FilterAction_Enum_Parse("ignore", &action));
  EXPECT_EQ(FilterAction_Enum_Name(static_cast<FilterAction>(1000)), "");
}

TEST(ProtoConversions, DirectionsType) {
  DirectionsType type;
  ASSERT_TRUE(DirectionsType_Enum_Parse("none", &type));
  EXPECT_EQ(type, DirectionsType::none);
  ASSERT_TRUE(DirectionsType_Enum_Parse("maneuvers", &type));
  EXPECT_EQ(type, DirectionsType::maneuvers);
  ASSERT_TRUE(DirectionsType_Enum_Parse("instructions", &type));
  EXPECT_EQ(type, DirectionsType::instructions);
  EXPECT_FALSE(DirectionsType_Enum_Parse("narrative", &type));
}

TEST(ProtoConversions, RoadClass) {
  const std::vector<std::pair<std::string, RoadClass>> classes = {
      {"motorway", RoadClass::kMotorway},       {"trunk", RoadClass::kTrunk},
      {"primary", RoadClass::kPrimary},         {"secondary", RoadClass::kSecondary},
      {"tertiary", RoadClass::kTertiary},       {"unclassified", RoadClass::kUnclassified},
      {"residential", RoadClass::kResidential}, {"service_other", RoadClass::kServiceOther},
  };
  for (const auto& [name, expected] : classes) {
    RoadClass rc;
    ASSERT_TRUE(RoadClass_Enum_Parse(name, &rc)) << name;
    EXPECT_EQ(rc, expected);
  }
  RoadClass rc;
  EXPECT_FALSE(RoadClass_Enum_Parse("Motorway", &rc));
}

TEST(ProtoConversions, ExpansionProperties) {
  const std::vector<std::pair<std::string, Options::ExpansionProperties>> props = {
      {"cost", Options::cost},
      {"duration", Options::duration},
      {"distance", Options::distance},
      {"edge_status", Options::edge_status},
      {"edge_id", Options::edge_id},
      {"pred_edge_id", Options::pred_edge_id},
      {"expansion_type", Options::expansion_type},
      {"flow_sources", Options::flow_sources},
      {"travel_mode", Options::travel_mode},
      {"expansion_index", Options::expansion_index},
  };
  for (const auto& [name, expected] : props) {
    Options::ExpansionProperties prop;
    ASSERT_TRUE(Options_ExpansionProperties_Enum_Parse(name, &prop)) << name;
    EXPECT_EQ(prop, expected);
  }
  Options::ExpansionProperties prop;
  EXPECT_FALSE(Options_ExpansionProperties_Enum_Parse("speed", &prop));
}

TEST(ProtoConversions, ExpansionEdgeStatus) {
  EXPECT_EQ(Expansion_EdgeStatus_Enum_Name(Expansion_EdgeStatus_reached), "r");
  EXPECT_EQ(Expansion_EdgeStatus_Enum_Name(Expansion_EdgeStatus_settled), "s");
  EXPECT_EQ(Expansion_EdgeStatus_Enum_Name(Expansion_EdgeStatus_connected), "c");
  EXPECT_EQ(Expansion_EdgeStatus_Enum_Name(static_cast<Expansion_EdgeStatus>(1000)), "");
}

TEST(ProtoConversions, TravelModeType) {
  DirectionsLeg_Maneuver maneuver;
  maneuver.set_travel_mode(TravelMode::kDrive);
  maneuver.set_vehicle_type(VehicleType::kTruck);
  EXPECT_EQ(travel_mode_type(maneuver), std::make_pair(std::string("drive"), std::string("truck")));

  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_pedestrian_type(PedestrianType::kWheelchair);
  EXPECT_EQ(travel_mode_type(maneuver).second, "wheelchair");

  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_bicycle_type(BicycleType::kMountain);
  EXPECT_EQ(travel_mode_type(maneuver).second, "mountain");

  maneuver.set_travel_mode(TravelMode::kTransit);
  maneuver.set_transit_type(TransitType::kFunicular);
  EXPECT_EQ(travel_mode_type(maneuver).second, "funicular");

  maneuver.set_travel_mode(static_cast<TravelMode>(100));
  EXPECT_THROW(travel_mode_type(maneuver), std::logic_error);
}

} // namespace
