#include <regex>

#include "test.h"
#include "valhalla/odin/maneuver.h"
#include "valhalla/odin/sign.h"
#include "valhalla/odin/signs.h"
#include "valhalla/odin/narrativebuilder.h"

using namespace std;
using namespace valhalla::odin;

namespace {

// Sub class to test protected methods
class NarrativeBuilderTest : public NarrativeBuilder {
 public:

  static std::string FormRampStraightInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormRampStraightInstruction(maneuver);
  }

  static std::string FormRampInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormRampInstruction(maneuver);
  }

  static std::string FormExitInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormExitInstruction(maneuver);
  }

  static std::string FormVerbalPostTransitionInstruction(
      Maneuver& maneuver, DirectionsOptions_Units units,
      bool include_street_names = false,
      uint32_t element_max_count = kVerbalPostElementMaxCount,
      std::string delim = kVerbalDelim) {
    return NarrativeBuilder::FormVerbalPostTransitionInstruction(
        maneuver, units, include_street_names, element_max_count, delim);
  }

};

Maneuver CreateVerbalPostManeuver(vector<std::string> street_names,
                                  float kilometers,
                                  TripDirections_Maneuver_Type type =
                                      TripDirections_Maneuver_Type_kRight) {

  Maneuver maneuver;
  maneuver.set_street_names(street_names);
  maneuver.set_length(kilometers);
  maneuver.set_type(type);

  return maneuver;
}

void TryFormVerbalPostTransitionInstruction(Maneuver maneuver,
                                            DirectionsOptions_Units units,
                                            bool include_street_names,
                                            std::string expected) {
  std::string instruction =
      NarrativeBuilderTest::FormVerbalPostTransitionInstruction(
          maneuver, units, include_street_names);
  if (instruction != expected) {
    throw std::runtime_error(
        "Incorrect FormVerbalPostTransitionInstruction - EXPECTED: "
            + expected + "  |  FORMED: " + instruction);
  }
}

void TestFormVerbalPostTransitionInstruction() {
  // Verify kilometers round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.54056f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 3.5 kilometers.");

  // Verify kilometers round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.86243f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 3.9 kilometers.");

  // Verify kilometers street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.86243f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 3.9 kilometers.");

  // Verify 1 kilometer round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.04f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 1 kilometer.");

  // Verify 1 kilometer round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.95f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 1 kilometer.");

  // Verify 1 kilometer street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.0f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 1 kilometer.");

  // Verify a half kilometer round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.54f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for a half kilometer.");

  // Verify a half kilometer round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.45f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for a half kilometer.");

  // Verify a half kilometer street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.5f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for a half kilometer.");

  // Verify 900 meters round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.94f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 900 meters.");

  // Verify 900 meters round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.85f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 900 meters.");

  // Verify 900 meters street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.9f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 900 meters.");

  // Verify 400 meters round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.44f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 400 meters.");

  // Verify 400 meters round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.35f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 400 meters.");

  // Verify 400 meters street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.4f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 400 meters.");

  // Verify 100 meters round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.14f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 100 meters.");

  // Verify 100 meters round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.095f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 100 meters.");

  // Verify 100 meters street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.1f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 100 meters.");

  // Verify 90 meters round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.094f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 90 meters.");

  // Verify 90 meters round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.085f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 90 meters.");

  // Verify 90 meters street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.09f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 90 meters.");

  // Verify 30 meters round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.034f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 30 meters.");

  // Verify 30 meters round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.025f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 30 meters.");

  // Verify 30 meters street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.03f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 30 meters.");

  // Verify 10 meters round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.012f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 10 meters.");

  // Verify 10 meters round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.0096f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 10 meters.");

  // Verify 10 meters street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.01f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 10 meters.");

  // Verify less than 10 meters round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" },  0.0094f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for less than 10 meters.");

  // Verify less than 10 meters
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.0088f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for less than 10 meters.");

  // Verify less than 10 meters street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.005f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for less than 10 meters.");

  /////////////////////////////////////////////////////////////////////////////

  // Verify miles round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.604931f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 2.2 miles.");

  // Verify miles round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.637117f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 2.3 miles.");

  // Verify miles street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.637117f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 2.3 miles.");

  // Verify 1 mile round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.657624f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 mile.");

  // Verify 1 mile round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.561064f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 mile.");

  // Verify 1 mile street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.60934f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 1 mile.");

  // Verify half mile round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.8368589f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for a half mile.");

  // Verify half mile round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.7724851f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for a half mile.");

  // Verify half mile street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.804672f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for a half mile.");

  // Verify 9 tenths of a mile round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.480596f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 9 tenths of a mile.");

  // Verify 9 tenths of a mile round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.416223f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 9 tenths of a mile.");

  // Verify 9 tenths of a mile street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 1.44841f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 9 tenths of a mile.");

  // Verify 4 tenths of a mile round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.675924f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 4 tenths of a mile.");

  // Verify 4 tenths of a mile round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.611551f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 4 tenths of a mile.");

  // Verify 4 tenths of a mile street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.643738f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 4 tenths of a mile.");

  // Verify 1 tenth of a mile round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.193121f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 tenth of a mile.");

  // Verify 1 tenth of a mile round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.158496f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 tenth of a mile.");

  // Verify 1 tenth of a mile street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.160934f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 1 tenth of a mile.");

  // Verify 500 feet round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.155448f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 500 feet.");

  // Verify 500 feet round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.149352f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 500 feet.");

  // Verify 500 feet street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.1524f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 500 feet.");

  // Verify 100 feet round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.036576f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 100 feet.");

  // Verify 100 feet round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.028956f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 100 feet.");

  // Verify 100 feet street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.03048f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 100 feet.");

  // Verify 90 feet round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.0283464f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 90 feet.");

  // Verify 90 feet round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.0268224f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 90 feet.");

  // Verify 90 feet street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.027432f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 90 feet.");

  // Verify 10 feet round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.0036576f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 10 feet.");

  // Verify 10 feet round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.00292608f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 10 feet.");

  // Verify 10 feet street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.003048f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 10 feet.");

  // Verify less than 10 feet round down
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.00280416f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for less than 10 feet.");

  // Verify less than 10 feet round up
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.00268224f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for less than 10 feet.");

  // Verify less than 10 feet street name
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 0.001524f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for less than 10 feet.");

}

Maneuver CreateSignManeuver(TripDirections_Maneuver_Type type,
                            Maneuver::RelativeDirection relative_direction,
                            vector<std::string> exit_numbers,
                            vector<std::string> exit_branches,
                            vector<std::string> exit_towards,
                            vector<std::string> exit_names) {

  Maneuver maneuver;
  maneuver.set_type(type);
  maneuver.set_begin_relative_direction(relative_direction);
  auto* signs = maneuver.mutable_signs();
  auto* exit_number_list = signs->mutable_exit_number_list();
  auto* exit_branch_list = signs->mutable_exit_branch_list();
  auto* exit_toward_list = signs->mutable_exit_toward_list();
  auto* exit_name_list = signs->mutable_exit_name_list();

  // Process exit numbers
  for (auto& exit_number : exit_numbers) {
    exit_number_list->emplace_back(exit_number);
  }

  // Process exit branches
  for (auto& exit_branch : exit_branches) {
    exit_branch_list->emplace_back(exit_branch);
  }

  // Process exit numbers
  for (auto& exit_toward : exit_towards) {
    exit_toward_list->emplace_back(exit_toward);
  }

  // Process exit numbers
  for (auto& exit_name : exit_names) {
    exit_name_list->emplace_back(exit_name);
  }

  return maneuver;
}

void TryFormRampStraightInstruction(Maneuver maneuver, std::string expected) {
  std::string instruction = NarrativeBuilderTest::FormRampStraightInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormRampStraightInstruction");
}

void TestFormRampStraightInstruction() {
  // phrase_id = 0
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, { },
                         { }, { }),
      "Stay straight to take the ramp.");

  // phrase_id = 1
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { }, { }),
      "Stay straight to take the I 95 South ramp.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { }, { "Gettysburg Pike" }),
      "Stay straight to take the I 95 South ramp.");

  // phrase_id = 2
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { }, { "Baltimore" }, { }),
      "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { }, { "Baltimore" }, { "Gettysburg Pike" }),
      "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 3
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward exists
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { "Baltimore" }, { "Gettysburg Pike" }),
      "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 4
  TryFormRampStraightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { }, { }, { "Gettysburg Pike" }),
      "Stay straight to take the Gettysburg Pike ramp.");

}

void TryFormRampRightInstruction(Maneuver maneuver, std::string expected) {
  std::string instruction = NarrativeBuilderTest::FormRampInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormRampRightInstruction");
}

void TestFormRampRightInstruction() {
  // phrase_id = 0
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { }),
      "Take the ramp on the right.");

  // phrase_id = 1
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South ramp on the right.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the right.");

  // phrase_id = 2
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the ramp on the right toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { "Gettysburg Pike" }),
      "Take the ramp on the right toward Baltimore.");

  // phrase_id = 3
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 4
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike ramp on the right.");

  // phrase_id = 5
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, { }, { },
                         { }),
      "Turn right to take the ramp.");

  // phrase_id = 6
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { },
                         { "I 95 South" }, { }, { }),
      "Turn right to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { },
                         { }, { "Baltimore" }, { }),
      "Turn right to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Turn right to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Turn right to take the Gettysburg Pike ramp.");

}

void TryFormRampLeftInstruction(Maneuver maneuver, std::string expected) {
  std::string instruction = NarrativeBuilderTest::FormRampInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormRampLeftInstruction");
}

void TestFormRampLeftInstruction() {
  // phrase_id = 0
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { }),
      "Take the ramp on the left.");

  // phrase_id = 1
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South ramp on the left.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the left.");

  // phrase_id = 2
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the ramp on the left toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { "Gettysburg Pike" }),
      "Take the ramp on the left toward Baltimore.");

  // phrase_id = 3
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 4
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike ramp on the left.");

  // phrase_id = 5
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { }, { }, { },
                         { }),
      "Turn left to take the ramp.");

  // phrase_id = 6
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { "I 95 South" }, { }, { }),
      "Turn left to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { }, { "Baltimore" }, { }),
      "Turn left to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Turn left to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Turn left to take the Gettysburg Pike ramp.");

}

void TryFormExitRightInstruction(Maneuver maneuver, std::string expected) {
  std::string instruction = NarrativeBuilderTest::FormExitInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormExitRightInstruction");
}

void TestFormExitRightInstruction() {
  // phrase_id = 0
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { }),
      "Take the exit on the right.");

  // phrase_id = 1
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { }, { }),
      "Take exit 67A on the right.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { }, { "Gettysburg Pike" }),
      "Take exit 67A on the right.");

  // phrase_id = 2
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South exit on the right.");

  // phrase_id = 3
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" }, {
                             "I 95 South" },
                         { }, { }),
      "Take exit 67A on the right onto I 95 South.");

  // phrase_id = 4
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the exit on the right toward Baltimore.");

  // phrase_id = 5
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { "Baltimore" }, { }),
      "Take exit 67A on the right toward Baltimore.");

  // phrase_id = 6
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South exit on the right toward Baltimore.");

  // phrase_id = 7
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take exit 67A on the right onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right.");

  // phrase_id = 10
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right onto US 15.");

  // phrase_id = 12
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { },
                         { "US 15" }, { "Harrisburg", "Gettysburg" }, {
                             "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right onto US 15 toward Harrisburg/Gettysburg.");

}

void TryFormExitLeftInstruction(Maneuver maneuver, std::string expected) {
  std::string instruction = NarrativeBuilderTest::FormExitInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormExitLeftInstruction");
}

void TestFormExitLeftInstruction() {
  // phrase_id = 0
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { }),
      "Take the exit on the left.");

  // phrase_id = 1
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { }, { }),
      "Take exit 67A on the left.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { }, { "Gettysburg Pike" }),
      "Take exit 67A on the left.");

  // phrase_id = 2
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South exit on the left.");

  // phrase_id = 3
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, {
                             "I 95 South" },
                         { }, { }),
      "Take exit 67A on the left onto I 95 South.");

  // phrase_id = 4
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the exit on the left toward Baltimore.");

  // phrase_id = 5
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { "Baltimore" }, { }),
      "Take exit 67A on the left toward Baltimore.");

  // phrase_id = 6
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South exit on the left toward Baltimore.");

  // phrase_id = 7
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take exit 67A on the left onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left.");

  // phrase_id = 10
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15.");

  // phrase_id = 12
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { },
                         { "US 15" }, { "Harrisburg", "Gettysburg" }, {
                             "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15 toward Harrisburg/Gettysburg.");

}

string ProcessNumberSplitMatch(const smatch& m) {
  string tts;
  if (m[1].matched) {
    tts += m[1].str();
  }

  string num = m[2].str();
  const size_t step = 2;
  const char space = ' ';
  for (size_t i = (num.size() % 2 == 0) ? step : (step - 1); i < num.size();
      i += step + 1) {
    num.insert(num.begin() + i, space);
  }
  tts += num;

  if (m[3].matched) {
    tts += m[3].str();
  }

  return tts;
}

string FormNumberSplitTts(string source) {
  const string number_split_pattern = "(\\D*)(\\d+)(\\D*)";
  const regex r(number_split_pattern);

  string tts;
  for (sregex_iterator it(source.begin(), source.end(), r), end_it;
      it != end_it; ++it) {
    tts += ProcessNumberSplitMatch(*it);
  }
  return tts.empty() ? source : tts;
}

void TryFormNumberSplitTtsString(string source, string expected) {
  string tts = FormNumberSplitTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormNumberSplitTtsString() {
  TryFormNumberSplitTtsString("1", "1");
  TryFormNumberSplitTtsString("12", "12");
  TryFormNumberSplitTtsString("123", "1 23");
  TryFormNumberSplitTtsString("1234", "12 34");
  TryFormNumberSplitTtsString("12345", "1 23 45");
  TryFormNumberSplitTtsString("123456", "12 34 56");
  TryFormNumberSplitTtsString("1234567", "1 23 45 67");
  TryFormNumberSplitTtsString("12345678", "12 34 56 78");
  TryFormNumberSplitTtsString("123456789", "1 23 45 67 89");
  TryFormNumberSplitTtsString("1234567890", "12 34 56 78 90");
  TryFormNumberSplitTtsString("105", "1 05");
  TryFormNumberSplitTtsString("20304", "2 03 04");
  TryFormNumberSplitTtsString("3010203", "3 01 02 03");
  TryFormNumberSplitTtsString("PA 220", "PA 2 20");
  TryFormNumberSplitTtsString("PA 283 West", "PA 2 83 West");
  TryFormNumberSplitTtsString("CR 1155 North", "CR 11 55 North");
  TryFormNumberSplitTtsString("146B", "1 46B");
  TryFormNumberSplitTtsString("1 123456", "1 12 34 56");
  TryFormNumberSplitTtsString("12 123456", "12 12 34 56");
  TryFormNumberSplitTtsString("1 123456 1", "1 12 34 56 1");
  TryFormNumberSplitTtsString("12 123456 1", "12 12 34 56 1");
  TryFormNumberSplitTtsString("123 123456", "1 23 12 34 56");
  TryFormNumberSplitTtsString("123456, 123456", "12 34 56, 12 34 56");
  TryFormNumberSplitTtsString("123456, 123456, 123456", "12 34 56, 12 34 56, 12 34 56");
  TryFormNumberSplitTtsString("County Road 00-122", "County Road 00-1 22");
  TryFormNumberSplitTtsString("Road 0110", "Road 01 10");
}

string FormLeadingZeroTts(string source) {
  const string leading_zero_pattern = "( )(0)([1-9])";
  const regex r(leading_zero_pattern);
  const string out_pattern = "$1o$3";

  string tts;
  tts = regex_replace(source, r, out_pattern);
  return tts.empty() ? source : tts;
}

void TryFormLeadingZeroTtsString(string source, string expected) {
  string tts = FormLeadingZeroTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormLeadingZeroTtsString() {
  TryFormLeadingZeroTtsString("1", "1");
  TryFormLeadingZeroTtsString("1 05", "1 o5");
  TryFormLeadingZeroTtsString("West 1 05 1st Avenue", "West 1 o5 1st Avenue");
  TryFormLeadingZeroTtsString("10 05 03", "10 o5 o3");
  TryFormLeadingZeroTtsString("County Road 00-1 22", "County Road 00-1 22");
  TryFormLeadingZeroTtsString("County Road 0-30", "County Road 0-30");
  TryFormLeadingZeroTtsString("State Highway 0", "State Highway 0");
}

string FormHundredTts(string source) {
  const string hundred_pattern = "(^| )([1-9]{1,2})(00)($| )";
  const regex r(hundred_pattern);
  const string out_pattern = "$1$2 hundred$4";

  string tts;
  tts = regex_replace(source, r, out_pattern);
  return tts.empty() ? source : tts;
}

void TryFormHundredTtsString(string source, string expected) {
  string tts = FormHundredTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormHundredTtsString() {
  TryFormHundredTtsString("1", "1");
  TryFormHundredTtsString("10", "10");
  TryFormHundredTtsString("100", "1 hundred");
  TryFormHundredTtsString("MD 100", "MD 1 hundred");
  TryFormHundredTtsString("MD 100 West", "MD 1 hundred West");
  TryFormHundredTtsString("2400", "24 hundred");
  TryFormHundredTtsString("MD 2400", "MD 24 hundred");
  TryFormHundredTtsString("MD 2400 West", "MD 24 hundred West");
}

string FormThousandTts(string source) {
  const string thousand_pattern = "(^| )([1-9]{1,2})(000)($| )";
  const regex r(thousand_pattern);
  const string out_pattern = "$1$2 thousand$4";

  string tts;
  tts = regex_replace(source, r, out_pattern);
  return tts.empty() ? source : tts;
}

void TryFormThousandTtsString(string source, string expected) {
  string tts = FormThousandTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormThousandTtsString() {
  TryFormThousandTtsString("1", "1");
  TryFormThousandTtsString("10", "10");
  TryFormThousandTtsString("1000", "1 thousand");
  TryFormThousandTtsString("MD 1000", "MD 1 thousand");
  TryFormThousandTtsString("MD 1000 West", "MD 1 thousand West");
  TryFormThousandTtsString("24000", "24 thousand");
  TryFormThousandTtsString("MD 24000", "MD 24 thousand");
  TryFormThousandTtsString("MD 24000 West", "MD 24 thousand West");
}

}

int main() {
  test::suite suite("narrativebuilder");

  // FormRampStraightInstruction
  suite.test(TEST_CASE(TestFormRampStraightInstruction));

  // FormRampRightInstruction
  suite.test(TEST_CASE(TestFormRampRightInstruction));

  // FormRampLeftInstruction
  suite.test(TEST_CASE(TestFormRampLeftInstruction));

  // FormExitRightInstruction
  suite.test(TEST_CASE(TestFormExitRightInstruction));

  // FormExitLeftInstruction
  suite.test(TEST_CASE(TestFormExitLeftInstruction));

  // FormVerbalPostTransitionInstruction
  suite.test(TEST_CASE(TestFormVerbalPostTransitionInstruction));

  // FormNumberSplitTtsString
  suite.test(TEST_CASE(TestFormNumberSplitTtsString));

  // FormLeadingZeroTtsString
  suite.test(TEST_CASE(TestFormLeadingZeroTtsString));

  // FormHundredTtsString
  suite.test(TEST_CASE(TestFormHundredTtsString));

  // FormThousandTtsString
  suite.test(TEST_CASE(TestFormThousandTtsString));

  return suite.tear_down();
}
