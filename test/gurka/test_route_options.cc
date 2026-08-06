#include "gurka.h"
#include "loki/worker.h"
#include "microtar.h"
#include "mjolnir/adminbuilder.h"
#include "test/test.h"

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/traffictile.h>

#ifndef _WIN32
#include <sys/mman.h>
#endif

#include <sys/stat.h>

#include <cmath>
#include <filesystem>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

void expected_options(valhalla::Api& result1) {
  rapidjson::Document response_json = gurka::convert_to_json(result1, valhalla::Options_Format_json);
  if (response_json.HasParseError()) {
    FAIL() << "Error converting route response to JSON";
  }
  rapidjson::Value* options = (rapidjson::GetValueByPointer(response_json, "/options"));

  if (!(options != nullptr && options->IsObject())) {
    FAIL() << "options not found";
  }

  rapidjson::Value& optionsObj = options->GetObject();

  rapidjson::Value* unit_pointer = rapidjson::GetValueByPointer(optionsObj, "/units");
  if (unit_pointer != nullptr && unit_pointer->IsString()) {
    valhalla::Options::Units unit;
    valhalla::Options_Units_Enum_Parse(unit_pointer->GetString(), &unit);
    EXPECT_EQ(unit, valhalla::Options_Units_kilometers);
  } else {
    FAIL() << "options.units not found";
  }

  rapidjson::Value* direction_type_pointer =
      rapidjson::GetValueByPointer(optionsObj, "/directions_type");
  if (direction_type_pointer != nullptr && direction_type_pointer->IsString()) {
    valhalla::DirectionsType direction_type;
    valhalla::DirectionsType_Enum_Parse(direction_type_pointer->GetString(), &direction_type);
    EXPECT_EQ(direction_type, valhalla::DirectionsType::instructions);
  } else {
    FAIL() << "options.directions_type not found";
  }

  rapidjson::Value* format_pointer = rapidjson::GetValueByPointer(optionsObj, "/format");
  if (format_pointer != nullptr && format_pointer->IsString()) {
    valhalla::Options::Format format;
    valhalla::Options_Format_Enum_Parse(format_pointer->GetString(), &format);
    EXPECT_EQ(format, valhalla::Options::Format::Options_Format_json);
  } else {
    FAIL() << "options.format not found";
  }

  rapidjson::Value* action_pointer = rapidjson::GetValueByPointer(optionsObj, "/action");
  if (action_pointer != nullptr && action_pointer->IsString()) {
    valhalla::Options::Action action;
    valhalla::Options_Action_Enum_Parse(action_pointer->GetString(), &action);
    EXPECT_EQ(action, valhalla::Options::Action::Options_Action_route);
  } else {
    FAIL() << "options.action not found";
  }

  rapidjson::Value* costing_type_pointer = rapidjson::GetValueByPointer(optionsObj, "/costing_type");
  if (costing_type_pointer != nullptr && costing_type_pointer->IsString()) {
    valhalla::Costing::Type costing_type;
    valhalla::Costing_Enum_Parse(costing_type_pointer->GetString(), &costing_type);
    EXPECT_EQ(costing_type, valhalla::Costing::bicycle);
  } else {
    FAIL() << "options.costing_type not found";
  }

  rapidjson::Value* costings_ponter = rapidjson::GetValueByPointer(optionsObj, "/costings");
  if (!(costings_ponter != nullptr && costings_ponter->IsObject())) {
    FAIL() << "options.costings not found";
  }

  rapidjson::Value& costing_obj = costings_ponter->GetObject();
  rapidjson::Value* bicycle_costing_pointer = rapidjson::GetValueByPointer(costing_obj, "/bicycle");

  if (!(bicycle_costing_pointer != nullptr && bicycle_costing_pointer->IsObject())) {
    FAIL() << "options.costings.bicycle not found";
  }

  rapidjson::Value& bicycle_costing_obj = bicycle_costing_pointer->GetObject();

  rapidjson::Value* name_pointer = rapidjson::GetValueByPointer(bicycle_costing_obj, "/name");
  if (name_pointer != nullptr && name_pointer->IsString()) {
    EXPECT_STREQ(name_pointer->GetString(), "bicycle");
  } else {
    FAIL() << "options.costings.bicycle.name not found";
  }

  rapidjson::Value* costing_options_pointer =
      rapidjson::GetValueByPointer(bicycle_costing_obj, "/options");
  if (!(costing_options_pointer != nullptr && costing_options_pointer->IsObject())) {
    FAIL() << "options.costings.bicycle.options not found";
  }

  rapidjson::Value& costing_options_obj = costing_options_pointer->GetObject();

  auto fixed_speed_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/fixed_speed");
  EXPECT_EQ(fixed_speed_ptr->GetUint(), 0);

  auto axle_count_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/axle_count");
  EXPECT_EQ(axle_count_ptr->GetUint(), 0);

  auto use_lit_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/use_lit");
  EXPECT_FLOAT_EQ(use_lit_ptr->GetFloat(), 0);

  auto ignore_non_vehicular_restrictions_ptr =
      rapidjson::GetValueByPointer(costing_options_obj, "/ignore_non_vehicular_restrictions");
  EXPECT_FALSE(ignore_non_vehicular_restrictions_ptr->GetBool());

  auto use_truck_route_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/use_truck_route");
  EXPECT_FLOAT_EQ(use_truck_route_ptr->GetFloat(), 0);

  auto exclude_bridges_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/exclude_bridges");
  EXPECT_FALSE(exclude_bridges_ptr->GetBool());

  auto exclude_tunnels_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/exclude_tunnels");
  EXPECT_FALSE(exclude_tunnels_ptr->GetBool());

  auto exclude_highways_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/exclude_highways");
  EXPECT_FALSE(exclude_highways_ptr->GetBool());

  auto exclude_ferries_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/exclude_ferries");
  EXPECT_FALSE(exclude_ferries_ptr->GetBool());

  auto exclude_tolls_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/exclude_tolls");
  EXPECT_FALSE(exclude_tolls_ptr->GetBool());

  auto ignore_construction_ptr =
      rapidjson::GetValueByPointer(costing_options_obj, "/ignore_construction");
  EXPECT_FALSE(ignore_construction_ptr->GetBool());

  auto disable_hierarchy_pruning_ptr =
      rapidjson::GetValueByPointer(costing_options_obj, "/disable_hierarchy_pruning");
  EXPECT_FALSE(disable_hierarchy_pruning_ptr->GetBool());

  auto filter_stop_action_ptr =
      rapidjson::GetValueByPointer(costing_options_obj, "/filter_stop_action");
  EXPECT_STREQ(filter_stop_action_ptr->GetString(), "");

  auto filter_operator_action_ptr =
      rapidjson::GetValueByPointer(costing_options_obj, "/filter_operator_action");
  EXPECT_STREQ(filter_operator_action_ptr->GetString(), "");

  auto filter_route_action_ptr =
      rapidjson::GetValueByPointer(costing_options_obj, "/filter_route_action");
  EXPECT_STREQ(filter_route_action_ptr->GetString(), "");

  auto maneuver_penalty_ptr = rapidjson::GetValueByPointer(costing_options_obj, "/maneuver_penalty");
  EXPECT_FLOAT_EQ(maneuver_penalty_ptr->GetFloat(), 5.0);

  auto destination_only_penalty_ptr =
      rapidjson::GetValueByPointer(costing_options_obj, "/destination_only_penalty");
  EXPECT_FLOAT_EQ(destination_only_penalty_ptr->GetFloat(), 600.0);
}

TEST(TestRouteOptions, GetOptions) {
  const std::string ascii_map = R"(
                                C--------D
                                |        |
      A-------------------------B        E-----------------------------------------F
                                |\      /|
                                | I----J |
                                |        |
                                G--------H
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"BIJE", {{"highway", "motorway"}, {"name", "RT 2"}}},
      {"BCDE", {{"highway", "primary"}, {"name", "RT 3"}, {"foot", "yes"}, {"bicycle", "no"}}},
      {"BGHE", {{"highway", "primary"}, {"name", "RT 4"}, {"foot", "no"}, {"bicycle", "yes"}}},
      {"EF", {{"highway", "primary"}, {"name", "RT 5"}}},
  };

  const auto node_layout = gurka::detail::map_to_coordinates(ascii_map, 100, PointLL{5.108, 52.01});

  std::unordered_map<std::string, std::string> config_map =
      {{"mjolnir.data_processing.use_direction_on_ways", "true"},
       {"mjolnir.admin", VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}};

  std::string workdir = "test/data/gurka_test_route_options";
  valhalla::gurka::map map = gurka::buildtiles(node_layout, ways, {}, {}, workdir, config_map);

  map.nodes = node_layout;

  // Bikes avoid motorways, so the shortest route is not an option.
  // ABCDEF is the next shortest route, but BCDE is marked bicycle=no.
  // The only remaining option is the southernmost route: ABGHEF
  valhalla::Api result1 = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "bicycle");
  EXPECT_EQ(result1.trip().routes_size(), 1);
  EXPECT_EQ(result1.trip().routes(0).legs_size(), 1);
  gurka::assert::osrm::expect_steps(result1, {"RT 1", "RT 4", "RT 5"});
  gurka::assert::osrm::expect_summaries(result1, {"RT 1, RT 5"});
  expected_options(result1);

  std::filesystem::remove_all(workdir);
}