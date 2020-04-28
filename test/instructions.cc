#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "odin/directionsbuilder.h"
#include "tyr/serializers.h"

#include <valhalla/proto/api.pb.h>
#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace std;
using namespace valhalla::baldr;

namespace {

void test_instructions(const std::string& filename,
                       int expected_routes_size,
                       int expected_legs_size,
                       int expected_maneuvers_size,
                       int maneuver_index,
                       const std::string& expected_text_instruction,
                       const std::string& expected_verbal_transition_alert_instruction = "",
                       const std::string& expected_verbal_pre_transition_instruction = "",
                       const std::string& expected_verbal_post_transition_instruction = "") {
  // Load pinpoint test
  std::string path_bytes = test::load_binary_file(filename);
  EXPECT_NE(path_bytes.size(), 0);

  // Create the request from the path bytes
  valhalla::Api request;
  request.ParseFromString(path_bytes);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request);

  // Validate routes size
  int found_routes_size = request.directions().routes_size();
  EXPECT_EQ(found_routes_size, expected_routes_size);

  // Validate legs size
  int found_legs_size = request.directions().routes(0).legs_size();
  EXPECT_EQ(found_legs_size, expected_legs_size);

  // Validate maneuvers size
  int found_maneuvers_size = request.directions().routes(0).legs(0).maneuver_size();
  EXPECT_EQ(found_maneuvers_size, expected_maneuvers_size);

  // Validate the text instruction for the specified maneuver index
  std::string found_text_instruction =
      request.directions().routes(0).legs(0).maneuver(maneuver_index).text_instruction();
  EXPECT_EQ(found_text_instruction, expected_text_instruction);

  // Validate the verbal_transition_alert_instruction for the specified maneuver index, if requested
  if (!expected_verbal_transition_alert_instruction.empty()) {
    std::string found_verbal_transition_alert_instruction =
        request.directions()
            .routes(0)
            .legs(0)
            .maneuver(maneuver_index)
            .verbal_transition_alert_instruction();
    EXPECT_EQ(found_verbal_transition_alert_instruction,
              expected_verbal_transition_alert_instruction);
  }

  // Validate the verbal_pre_transition_instruction for the specified maneuver index, if requested
  if (!expected_verbal_pre_transition_instruction.empty()) {
    std::string found_verbal_pre_transition_instruction = request.directions()
                                                              .routes(0)
                                                              .legs(0)
                                                              .maneuver(maneuver_index)
                                                              .verbal_pre_transition_instruction();
    EXPECT_EQ(found_verbal_pre_transition_instruction, expected_verbal_pre_transition_instruction);
  }

  // Validate the verbal_post_transition_instruction for the specified maneuver index, if requested
  if (!expected_verbal_post_transition_instruction.empty()) {
    std::string found_verbal_post_transition_instruction = request.directions()
                                                               .routes(0)
                                                               .legs(0)
                                                               .maneuver(maneuver_index)
                                                               .verbal_post_transition_instruction();
    EXPECT_EQ(found_verbal_post_transition_instruction, expected_verbal_post_transition_instruction);
  }
}

void test_osrm_maneuver(const std::string& filename,
                        int routes_index,
                        int legs_index,
                        int steps_index,
                        const std::string& expected_maneuver_type,
                        const std::string& expected_maneuver_modifier) {
  // Load pinpoint test
  std::string path_bytes = test::load_binary_file(filename);
  EXPECT_NE(path_bytes.size(), 0);

  // Create the request from the path bytes
  valhalla::Api request;
  request.ParseFromString(path_bytes);

  // Set osrm format
  request.mutable_options()->set_format(valhalla::Options_Format_osrm);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request);

  // Serialize to osrm json string
  auto json_str = valhalla::tyr::serializeDirections(request);

  rapidjson::Document doc;
  doc.Parse(json_str.c_str());

  ASSERT_FALSE(doc.HasParseError()) << "Parse JSON error";

  // Set the maneuver path
  std::string maneuver_path = "/routes/" + std::to_string(routes_index) + "/legs/" +
                              std::to_string(legs_index) + "/steps/" + std::to_string(steps_index) +
                              "/maneuver";

  // Validate maneuver type
  std::string maneuver_type_path = maneuver_path + "/type";
  std::string found_maneuver_type = rapidjson::get<std::string>(doc, maneuver_type_path.c_str());
  EXPECT_EQ(found_maneuver_type, expected_maneuver_type);

  // Validate maneuver modifier
  std::string maneuver_midifier_path = maneuver_path + "/modifier";
  std::string found_maneuver_modifier =
      rapidjson::get<std::string>(doc, maneuver_midifier_path.c_str());
  EXPECT_EQ(found_maneuver_modifier, expected_maneuver_modifier);
}

void test_osrm_destinations(const std::string& filename,
                            int routes_index,
                            int legs_index,
                            int steps_index,
                            const std::string& expected_destinations) {
  // Load pinpoint test
  std::string path_bytes = test::load_binary_file(filename);

  EXPECT_NE(path_bytes.size(), 0);

  // Create the request from the path bytes
  valhalla::Api request;
  request.ParseFromString(path_bytes);

  // Set osrm format
  request.mutable_options()->set_format(valhalla::Options_Format_osrm);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request);

  // Serialize to osrm json string
  auto json_str = valhalla::tyr::serializeDirections(request);

  rapidjson::Document doc;
  doc.Parse(json_str.c_str());
  ASSERT_FALSE(doc.HasParseError()) << "Parse JSON error";

  // Set the destination path
  std::string destinations_path = "/routes/" + std::to_string(routes_index) + "/legs/" +
                                  std::to_string(legs_index) + "/steps/" +
                                  std::to_string(steps_index) + "/destinations";

  // Validate destinations
  std::string found_destinations = rapidjson::get<std::string>(doc, destinations_path.c_str());
  EXPECT_EQ(found_destinations, expected_destinations);
}

TEST(Instructions, validate_merge_instructions) {

  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 4;
  int maneuver_index = 2;

  // Test merge right
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/merge_right.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Merge right onto I 695 West/Baltimore Beltway.", "",
                    "Merge right onto Interstate 6 95 West, Baltimore Beltway.",
                    "Continue for a quarter mile.");

  // Test merge left
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/merge_left.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Merge left onto US 322 East.", "", "Merge left onto U.S. 3 22 East.",
                    "Continue for 1,000 feet.");
}

TEST(Instructions, validate_osrm_merge_maneuver) {

  int routes_index = 0;
  int legs_index = 0;
  int steps_index = 2;

  // Test osrm merge right
  test_osrm_maneuver({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/merge_right.pbf"},
                     routes_index, legs_index, steps_index, "merge", "slight right");

  // Test osrm merge left
  test_osrm_maneuver({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/merge_left.pbf"}, routes_index,
                     legs_index, steps_index, "merge", "slight left");
}

TEST(Instructions, validate_osrm_turn_destinations) {

  int routes_index = 0;
  int legs_index = 0;
  int steps_index = 1;

  // Test osrm turn right guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/turn_right_guide_sign.pbf"},
                         routes_index, legs_index, steps_index, "A 95: München");

  // Test osrm turn left guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/turn_left_guide_sign.pbf"},
                         routes_index, legs_index, steps_index, "Germering, Planegg");

  // Test osrm bear right guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/bear_right_guide_sign.pbf"},
                         routes_index, legs_index, steps_index,
                         "Hersheypark, Arena, Stadium, Giant Center");

  // Test osrm bear left guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/bear_left_guide_sign.pbf"},
                         routes_index, legs_index, steps_index, "US 50 West: Fairfax");

  steps_index = 3;

  // Test osrm bear right guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_and_bear_right_guide_sign.pbf"},
                         routes_index, legs_index, steps_index, "A20: Dover, Channel Tunnel");
}

TEST(Instructions, validate_osrm_roundabout_destinations) {

  int routes_index = 0;
  int legs_index = 0;
  int enter_steps_index = 1;
  int exit_steps_index = 2;

  // Test osrm roundabout enter guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_guide_sign_1.pbf"},
                         routes_index, legs_index, enter_steps_index, "Kürten, Dhünn");

  // Test osrm roundabout exit guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_guide_sign_1.pbf"},
                         routes_index, legs_index, exit_steps_index, "Kürten, Dhünn");

  // Test osrm roundabout enter guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_guide_sign_2.pbf"},
                         routes_index, legs_index, enter_steps_index, "Hückeswagen");

  // Test osrm roundabout exit guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_guide_sign_2.pbf"},
                         routes_index, legs_index, exit_steps_index, "Hückeswagen");

  // Test osrm roundabout enter guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_guide_sign_3.pbf"},
                         routes_index, legs_index, enter_steps_index,
                         "A 1: Remscheid, Wermelskirchen");

  // Test osrm roundabout exit guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_guide_sign_3.pbf"},
                         routes_index, legs_index, exit_steps_index,
                         "A 1: Remscheid, Wermelskirchen");

  // Test osrm roundabout enter guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_and_bear_right_guide_sign.pbf"},
                         routes_index, legs_index, enter_steps_index, "Bexley");

  // Test osrm roundabout exit guide sign
  test_osrm_destinations({VALHALLA_SOURCE_DIR
                          "test/pinpoints/instructions/roundabout_and_bear_right_guide_sign.pbf"},
                         routes_index, legs_index, exit_steps_index, "Bexley");
}

TEST(Instructions, validate_ramp_instructions) {
  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 4;
  int maneuver_index = 1;

  // Test take toward driving side right
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/ramp_take_toward_driving_side_right.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the PA 283 West ramp toward Harrisburg.",
                    "Take the Pennsylvania 2 83 West ramp.",
                    "Take the Pennsylvania 2 83 West ramp toward Harrisburg.");

  // Test take toward driving side left
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/ramp_take_toward_driving_side_left.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the M11 ramp toward London.", "", "Take the M11 ramp toward London.");
}

TEST(Instructions, validate_osrm_ramp_maneuver) {

  int routes_index = 0;
  int legs_index = 0;
  int steps_index = 1;

  // Test take toward driving side right
  test_osrm_maneuver({VALHALLA_SOURCE_DIR
                      "test/pinpoints/instructions/ramp_take_toward_driving_side_right.pbf"},
                     routes_index, legs_index, steps_index, "on ramp", "slight right");

  // Test take toward driving side left
  test_osrm_maneuver({VALHALLA_SOURCE_DIR
                      "test/pinpoints/instructions/ramp_take_toward_driving_side_left.pbf"},
                     routes_index, legs_index, steps_index, "on ramp", "slight left");
}

TEST(Instructions, validate_exit_instructions) {
  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 3;
  int maneuver_index = 1;

  // Test exit left on right driving side
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/exit_left_driving_side_right.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the I 66 East exit on the left toward Washington.", "",
                    "Take the Interstate 66 East exit on the left toward Washington.");

  // Test exit left on left driving side
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/exit_left_driving_side_left.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take exit 8 onto A120(W)|A120(W).", "", "Take exit 8 onto A1 20(W)|A1 20(W).");

  expected_maneuvers_size = 4;
  // Test exit non-motorway in PA
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/exit_right_nonmotorway_pa.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the PA 934 exit toward I 81/Fort Indiantown Gap/Annville.",
                    "Take the Pennsylvania 9 34 exit.",
                    "Take the Pennsylvania 9 34 exit toward Interstate 81, Fort Indiantown Gap.");

  expected_maneuvers_size = 5;
  // Test exit non-motorway in VA
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/exit_right_nonmotorway_va.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the US 15 North exit toward Frederick Maryland.",
                    "Take the U.S. 15 North exit.",
                    "Take the U.S. 15 North exit toward Frederick Maryland.");
}

TEST(Instructions, validate_osrm_exit_maneuver) {

  int routes_index = 0;
  int legs_index = 0;
  int steps_index = 1;

  // Test exit left on right driving side
  test_osrm_maneuver({VALHALLA_SOURCE_DIR
                      "test/pinpoints/instructions/exit_left_driving_side_right.pbf"},
                     routes_index, legs_index, steps_index, "off ramp", "slight left");

  // Test exit left on left driving side
  test_osrm_maneuver({VALHALLA_SOURCE_DIR
                      "test/pinpoints/instructions/exit_left_driving_side_left.pbf"},
                     routes_index, legs_index, steps_index, "off ramp", "slight left");

  // Test exit non-motorway in PA
  test_osrm_maneuver({VALHALLA_SOURCE_DIR
                      "test/pinpoints/instructions/exit_right_nonmotorway_pa.pbf"},
                     routes_index, legs_index, steps_index, "off ramp", "slight right");

  // Test exit non-motorway in VA
  test_osrm_maneuver({VALHALLA_SOURCE_DIR
                      "test/pinpoints/instructions/exit_right_nonmotorway_va.pbf"},
                     routes_index, legs_index, steps_index, "off ramp", "slight right");
}

TEST(Instructions, validate_multi_cue_instructions) {
  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 4;
  int maneuver_index = 0;

  // Test the start verbal multi-cue instruction
  test_instructions(
      {VALHALLA_SOURCE_DIR "test/pinpoints/instructions/multi_cue_start_turn_destination.pbf"},
      expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
      "Drive north on Hartman Bridge Road/PA 896.", "",
      "Drive north on Hartman Bridge Road, Pennsylvania 8 96 for 200 feet. Then Turn left onto U.S. 30.");

  maneuver_index = 1;
  // Test the turn verbal multi-cue instruction
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/multi_cue_start_turn_destination.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn left onto US 30/Lincoln Highway East.", "Turn left onto U.S. 30.",
                    "Turn left onto U.S. 30, Lincoln Highway East. Then Turn right.",
                    "Continue for 500 feet.");

  maneuver_index = 2;
  // Test the destination verbal multi-cue instruction
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/multi_cue_start_turn_destination.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn right.", "Turn right.",
                    "Turn right. Then Adidas Outlet will be on the left.", "Continue for 100 feet.");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
