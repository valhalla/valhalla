#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "baldr/turnlanes.h"
#include "odin/directionsbuilder.h"
#include "odin/enhancedtrippath.h"

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

void test_instructions(const std::string filename,
                       int expected_routes_size,
                       int expected_legs_size,
                       int expected_maneuvers_size,
                       int maneuver_index,
                       const std::string expected_text_instruction) {
  // Load pinpoint test
  std::string path_bytes = test::load_binary_file(filename);
  if (path_bytes.size() == 0) {
    throw std::runtime_error("path_bytes is empty");
  }

  // Create the request from the path bytes
  valhalla::Api request;
  request.ParseFromString(path_bytes);

  // Build the directions
  valhalla::odin::DirectionsBuilder().Build(request);

  // Validate routes size
  int found_routes_size = request.directions().routes_size();
  if (found_routes_size != expected_routes_size) {
    throw std::runtime_error("Invalid routes size - found: " + std::to_string(found_routes_size) +
                             " | expected: " + std::to_string(expected_routes_size));
  }

  // Validate legs size
  int found_legs_size = request.directions().routes(0).legs_size();
  if (found_legs_size != expected_legs_size) {
    throw std::runtime_error("Invalid legs size - found: " + std::to_string(found_legs_size) +
                             " | expected: " + std::to_string(expected_legs_size));
  }

  // Validate maneuvers size
  int found_maneuvers_size = request.directions().routes(0).legs(0).maneuver_size();
  if (found_maneuvers_size != expected_maneuvers_size) {
    throw std::runtime_error(
        "Invalid maneuvers size - found: " + std::to_string(found_maneuvers_size) +
        " | expected: " + std::to_string(expected_maneuvers_size));
  }

  // Validate the text instruction for the specified maneuver index
  std::string found_text_instruction =
      request.directions().routes(0).legs(0).maneuver(maneuver_index).text_instruction();
  if (found_text_instruction != expected_text_instruction) {
    throw std::runtime_error("Invalid text instruction - found: " + found_text_instruction +
                             " | expected: " + expected_text_instruction);
  }
}

void validate_merge_instructions() {

  int expected_routes_size = 1;
  int expected_legs_size = 1;
  int expected_maneuvers_size = 4;
  int maneuver_index = 2;

  // Test merge right
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/merge_right.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Merge right onto I 695 West/Baltimore Beltway.");

  // Test merge left
  test_instructions({VALHALLA_SOURCE_DIR "test/pinpoints/instructions/merge_left.pbf"},
                    expected_routes_size, expected_legs_size, expected_maneuvers_size, maneuver_index,
                    "Merge left onto US 322 East.");
}

} // namespace

int main() {
  test::suite suite("instructions");

  // Test turn lanes
  suite.test(TEST_CASE(validate_merge_instructions));

  return suite.tear_down();
}
