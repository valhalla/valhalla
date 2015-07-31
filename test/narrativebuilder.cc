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
                                  float length,
                                  TripDirections_Maneuver_Type type =
                                      TripDirections_Maneuver_Type_kRight) {

  Maneuver maneuver;
  maneuver.set_street_names(street_names);
  maneuver.set_length(length);
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
  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.54056f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 3.5 kilometers.");

  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.86243f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 3.9 kilometers.");

  TryFormVerbalPostTransitionInstruction(
      CreateVerbalPostManeuver( { "Main Street" }, 3.86243f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 3.9 kilometers.");
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

  return suite.tear_down();
}
