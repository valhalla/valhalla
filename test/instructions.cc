#include <fstream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "odin/directionsbuilder.h"
#include "odin/markup_formatter.h"
#include "tyr/serializers.h"

#include "proto/api.pb.h"
#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"

#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

// this is useful when you modify the Options proto and need to restore it
//#include "worker.h"
// void fix_request(const std::string& filename, valhalla::Api& request) {
//  auto txt = filename;
//  txt.replace(txt.size() - 3, 3, "txt");
//  std::string req_txt = test::load_binary_file(txt);
//  req_txt.pop_back();
//  req_txt.pop_back();
//  req_txt = req_txt.substr(4);
//
//  // valhalla::Api api;
//  // valhalla::ParseApi(req_txt, valhalla::Options::route, api);
//  // request.mutable_options()->CopyFrom(api.options());
//
//  for (auto& loc : *request.mutable_options()->mutable_locations()) {
//    loc.mutable_correlation()->mutable_edges()->CopyFrom(loc.correlation().old_edges());
//    loc.mutable_correlation()->set_original_index(loc.correlation().old_original_index());
//    loc.mutable_correlation()->mutable_projected_ll()->CopyFrom(loc.correlation().old_projected_ll());
//    loc.mutable_correlation()->set_leg_shape_index(loc.correlation().old_leg_shape_index());
//    loc.mutable_correlation()->set_distance_from_leg_origin(
//        loc.correlation().old_distance_from_leg_origin());
//  }
//
//  for (auto& loc : *request.mutable_trip()->mutable_routes(0)->mutable_legs(0)->mutable_location())
//  {
//    loc.mutable_correlation()->mutable_edges()->CopyFrom(loc.correlation().old_edges());
//    loc.mutable_correlation()->set_original_index(loc.correlation().old_original_index());
//    loc.mutable_correlation()->mutable_projected_ll()->CopyFrom(loc.correlation().old_projected_ll());
//    loc.mutable_correlation()->set_leg_shape_index(loc.correlation().old_leg_shape_index());
//    loc.mutable_correlation()->set_distance_from_leg_origin(
//        loc.correlation().old_distance_from_leg_origin());
//  }
//
//  std::ofstream f(filename);
//  auto buf = request.SerializeAsString();
//  f.write(buf.data(), buf.size());
//}

using namespace valhalla::baldr;

namespace {

void test_instructions(const std::string& filename,
                       int expected_routes_size,
                       int expected_legs_size,
                       int expected_maneuvers_size,
                       int maneuver_index,
                       const std::string& expected_text_instruction,
                       const std::string& expected_verbal_succinct_transition_instruction,
                       const std::string& expected_verbal_transition_alert_instruction,
                       const std::string& expected_verbal_pre_transition_instruction,
                       const std::string& expected_verbal_post_transition_instruction) {
  // Load pinpoint test
  std::string path_bytes = test::load_binary_file(filename);
  EXPECT_NE(path_bytes.size(), 0);

  // Create the request from the path bytes
  valhalla::Api request;
  request.ParseFromString(path_bytes);

  // fix_request(filename, request);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request, valhalla::odin::MarkupFormatter());

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

  // Validate the verbal_succinct_transition_instruction for the specified maneuver index
  std::string found_verbal_succinct_transition_instruction =
      request.directions()
          .routes(0)
          .legs(0)
          .maneuver(maneuver_index)
          .verbal_succinct_transition_instruction();
  EXPECT_EQ(found_verbal_succinct_transition_instruction,
            expected_verbal_succinct_transition_instruction);

  // Validate the verbal_transition_alert_instruction for the specified maneuver index
  std::string found_verbal_transition_alert_instruction = request.directions()
                                                              .routes(0)
                                                              .legs(0)
                                                              .maneuver(maneuver_index)
                                                              .verbal_transition_alert_instruction();
  EXPECT_EQ(found_verbal_transition_alert_instruction, expected_verbal_transition_alert_instruction);

  // Validate the verbal_pre_transition_instruction for the specified maneuver index
  std::string found_verbal_pre_transition_instruction = request.directions()
                                                            .routes(0)
                                                            .legs(0)
                                                            .maneuver(maneuver_index)
                                                            .verbal_pre_transition_instruction();
  EXPECT_EQ(found_verbal_pre_transition_instruction, expected_verbal_pre_transition_instruction);

  // Validate the verbal_post_transition_instruction for the specified maneuver index
  std::string found_verbal_post_transition_instruction = request.directions()
                                                             .routes(0)
                                                             .legs(0)
                                                             .maneuver(maneuver_index)
                                                             .verbal_post_transition_instruction();
  EXPECT_EQ(found_verbal_post_transition_instruction, expected_verbal_post_transition_instruction);
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

  // fix_request(filename, request);

  // Set osrm format
  request.mutable_options()->set_format(valhalla::Options_Format_osrm);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request, valhalla::odin::MarkupFormatter());

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

  // fix_request(filename, request);

  // Set osrm format
  request.mutable_options()->set_format(valhalla::Options_Format_osrm);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request, valhalla::odin::MarkupFormatter());

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
  int expected_maneuvers_size = 3;
  int maneuver_index = 1;

  // Test take toward driving side right
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/ramp_take_toward_driving_side_right.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the PA 283 West ramp toward Harrisburg.", "",
                    "Take the Pennsylvania 2 83 West ramp.",
                    "Take the Pennsylvania 2 83 West ramp toward Harrisburg.",
                    "Continue for a quarter mile.");

  // Test take toward driving side left
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/ramp_take_toward_driving_side_left.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the M11 ramp toward London.", "", "Take the M11 ramp.",
                    "Take the M11 ramp toward London.", "Continue for a half mile.");
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
                    "Take the Interstate 66 East exit on the left.",
                    "Take the Interstate 66 East exit on the left toward Washington.", "");

  // Test exit left on left driving side
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/exit_left_driving_side_left.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take exit 8 onto A120(W)|A120(W).", "", "Take exit 8.",
                    "Take exit 8 onto A120(W)|A120(W).", "");
  expected_maneuvers_size = 4;
  // Test exit non-motorway in PA
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/exit_right_nonmotorway_pa.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the PA 934 exit toward I 81/Fort Indiantown Gap/Annville.", "",
                    "Take the Pennsylvania 9 34 exit.",
                    "Take the Pennsylvania 9 34 exit toward Interstate 81, Fort Indiantown Gap.", "");

  expected_maneuvers_size = 4;
  // Test exit non-motorway in VA
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/exit_right_nonmotorway_va.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the US 15 North exit toward Frederick Maryland.", "",
                    "Take the U.S. 15 North exit.",
                    "Take the U.S. 15 North exit toward Frederick Maryland.", "");
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
  int maneuver_index = 1;

  // Test imminent turn
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/multi_cue_imminent_turn.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn right onto Linden Road.",
                    "Turn right. Then Turn left onto West Caracas Avenue.",
                    "Turn right onto Linden Road.",
                    "Turn right onto Linden Road. Then Turn left onto West Caracas Avenue.",
                    "Continue for 400 feet.");

  maneuver_index = 0;
  // Test the imminent start verbal multi-cue instruction
  test_instructions(
      {VALHALLA_SOURCE_DIR "test/pinpoints/instructions/multi_cue_start_turn_destination.pbf"},
      expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
      "Drive north on Hartman Bridge Road/PA 896.", "Drive north. Then Turn left onto U.S. 30.", "",
      "Drive north on Hartman Bridge Road, Pennsylvania 8 96. Then Turn left onto U.S. 30.",
      "Continue for 200 feet.");

  maneuver_index = 1;
  // Test the distant turn verbal multi-cue instruction
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/multi_cue_start_turn_destination.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn left onto US 30/Lincoln Highway East.",
                    "Turn left. Then, in 500 feet, Turn right.", "Turn left onto U.S. 30.",
                    "Turn left onto U.S. 30, Lincoln Highway East. Then, in 500 feet, Turn right.",
                    "Continue for 500 feet.");

  maneuver_index = 2;
  // Test the distant destination verbal multi-cue instruction
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/multi_cue_start_turn_destination.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn right.",
                    "Turn right. Then, in 100 feet, Adidas Outlet will be on the left.",
                    "Turn right.",
                    "Turn right. Then, in 100 feet, Adidas Outlet will be on the left.",
                    "Continue for 100 feet.");
}

TEST(Instructions, validate_roundabout_unnamed_cycleway_instructions) {
  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 4;
  int maneuver_index = 0;

  // Test start maneuver on unnamed cycleway prior to roundabout
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/roundabout_unnamed_cycleway.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Bike east on the cycleway.",
                    "Bike east. Then Enter the roundabout and take the 2nd exit.", "",
                    "Bike east on the cycleway. Then Enter the roundabout and take the 2nd exit.",
                    "Continue for 200 feet.");

  maneuver_index = 1;
  // Test enter roundabout with unnamed cycleway
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/roundabout_unnamed_cycleway.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Enter the roundabout and take the 2nd exit.",
                    "Enter the roundabout and take the 2nd exit.",
                    "Enter the roundabout and take the 2nd exit.",
                    "Enter the roundabout and take the 2nd exit.", "");

  maneuver_index = 2;
  // Test exit roundabout onto unnamed cycleway
  test_instructions(
      {VALHALLA_SOURCE_DIR "test/pinpoints/instructions/roundabout_unnamed_cycleway.pbf"},
      expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
      "Exit the roundabout onto the cycleway.",
      "Exit the roundabout. Then, in 200 feet, You will arrive at your destination.", "",
      "Exit the roundabout onto the cycleway. Then, in 200 feet, You will arrive at your destination.",
      "Continue for 200 feet.");
}

TEST(Instructions, validate_turn_at_instructions) {

  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 3;
  int maneuver_index = 1;

  // Turn right at
  test_instructions(
      {VALHALLA_SOURCE_DIR "test/pinpoints/instructions/turn_right_at.pbf"}, expected_routes_size,
      expected_legs_size, expected_maneuvers_size, maneuver_index, "Turn right at 新橋三丁目交番前.",
      "Turn right at 新橋三丁目交番前. Then, in 50 meters, You will arrive at your destination.",
      "Turn right at 新橋三丁目交番前.",
      "Turn right at 新橋三丁目交番前. Then, in 50 meters, You will arrive at your destination.",
      "Continue for 50 meters.");

  // Turn right at using internal edge
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/turn_right_at_using_internal_edge.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn right at 万年橋東.",
                    "Turn right at 万年橋東. Then You will arrive at your destination.",
                    "Turn right at 万年橋東.",
                    "Turn right at 万年橋東. Then You will arrive at your destination.",
                    "Continue for 50 meters.");

  // Make a right U-turn at
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/uturn_right_at.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Make a right U-turn at 銀座五丁目.",
                    "Make a right U-turn at 銀座五丁目. Then You will arrive at 15.",
                    "Make a right U-turn at 銀座五丁目.",
                    "Make a right U-turn at 銀座五丁目. Then You will arrive at 15.",
                    "Continue for 30 meters.");

  // Turn left at
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/turn_left_at.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn left at 銀座七丁目.",
                    "Turn left at 銀座七丁目. Then You will arrive at 花椿通り.",
                    "Turn left at 銀座七丁目.",
                    "Turn left at 銀座七丁目. Then You will arrive at 花椿通り.",
                    "Continue for 20 meters.");
}

TEST(Instructions, validate_obvious_maneuver_instructions) {

  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 4;
  int maneuver_index = 1;

  // Simple name change collapsed
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/obvious_maneuver_simple_name_change.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn left onto Vine Street.", "Turn left.", "Turn left onto Vine Street.",
                    "Turn left onto Vine Street.", "Continue for 5 miles.");

  // Turn channel to continue collapsed
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/obvious_maneuver_turn_channel.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Bear right toward 8th Street.", "Bear right toward 8th Street.",
                    "Bear right toward 8th Street.", "Bear right toward 8th Street.",
                    "Continue for a quarter mile.");

  // Suppress use of the begin street name when the step contains an obvious maneuver
  expected_maneuvers_size = 5;
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/obvious_maneuver_begin_name.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn left onto Hershey Road/PA 743/PA 341 Truck.", "Turn left.",
                    "Turn left onto Hershey Road.", "Turn left onto Hershey Road, Pennsylvania 7 43.",
                    "Continue for 8 miles.");

  // Short continue collapsed
  // no continue on US 422; US 322 and then the PA 39 West/Hersheypark Drive Exit
  expected_maneuvers_size = 5;
  maneuver_index = 1;
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/obvious_maneuver_short_continue.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Turn right to take the US 322 East ramp.", "",
                    "Turn right to take the U.S. 3 22 East ramp.",
                    "Turn right to take the U.S. 3 22 East ramp.", "Continue for 1 mile.");

  // Exit onto PA 39 West/Hersheypark Drive
  maneuver_index = 2;
  test_instructions({VALHALLA_SOURCE_DIR
                     "test/pinpoints/instructions/obvious_maneuver_short_continue.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Take the PA 39 West/Hersheypark Drive exit toward Attractions.", "",
                    "Take the Pennsylvania 39 West exit.",
                    "Take the Pennsylvania 39 West, Hersheypark Drive exit toward Attractions.",
                    "Continue for a half mile.");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
