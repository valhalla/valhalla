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

  static void FormExitRightInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormExitRightInstruction(maneuver);
  }

  static void FormExitLeftInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormExitLeftInstruction(maneuver);
  }

};

Maneuver CreateSignManeuver(TripDirections_Maneuver_Type type,
                            vector<std::string> exit_numbers,
                            vector<std::string> exit_branches,
                            vector<std::string> exit_towards,
                            vector<std::string> exit_names) {

  Maneuver maneuver;
  maneuver.set_type(type);
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

void TryFormExitRightInstruction(Maneuver maneuver, std::string expected) {
  NarrativeBuilderTest nbTest;
  nbTest.FormExitRightInstruction(maneuver);
  if (maneuver.instruction() != expected)
    throw std::runtime_error("Incorrect FormExitRightInstruction");
}

void TestFormExitRightInstruction() {
  // phrase_id = 0
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { }, { }, { },
                         { }),
      "Take the exit on the right.");

  // phrase_id = 1
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { "67A" },
                         { }, { }, { }),
      "Take exit 67A on the right.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { "67A" },
                         { }, { }, { "Gettysburg Pike" }),
      "Take exit 67A on the right.");

  // phrase_id = 2
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { },
                         { "I 95 South" }, { }, { }),
      "Take the I 95 South exit on the right.");

  // phrase_id = 3
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { "67A" },
                         { "I 95 South" }, { }, { }),
      "Take exit 67A on the right onto I 95 South.");

  // phrase_id = 4
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { },
                         { }, { "Baltimore" }, { }),
      "Take the exit on the right toward Baltimore.");

  // phrase_id = 5
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { "67A" },
                         { }, { "Baltimore" }, { }),
      "Take exit 67A on the right toward Baltimore.");

  // phrase_id = 6
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Take the I 95 South exit on the right toward Baltimore.");

  // phrase_id = 7
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { "67A" },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Take exit 67A on the right onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { },
                         { }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right.");

  // phrase_id = 10
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right onto US 15.");

  // phrase_id = 12
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { }, { },
                         { "Harrisburg", "Gettysburg" }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitRightInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight, { },
                         { "US 15" }, { "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right onto US 15 toward Harrisburg/Gettysburg.");

}

void TryFormExitLeftInstruction(Maneuver maneuver, std::string expected) {
  NarrativeBuilderTest nbTest;
  nbTest.FormExitLeftInstruction(maneuver);
  if (maneuver.instruction() != expected)
    throw std::runtime_error("Incorrect FormExitLeftInstruction");
}

void TestFormExitLeftInstruction() {
  // phrase_id = 0
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { }, { }, { },
                         { }),
      "Take the exit on the left.");

  // phrase_id = 1
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { "67A" },
                         { }, { }, { }),
      "Take exit 67A on the left.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { "67A" },
                         { }, { }, { "Gettysburg Pike" }),
      "Take exit 67A on the left.");

  // phrase_id = 2
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { },
                         { "I 95 South" }, { }, { }),
      "Take the I 95 South exit on the left.");

  // phrase_id = 3
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { "67A" },
                         { "I 95 South" }, { }, { }),
      "Take exit 67A on the left onto I 95 South.");

  // phrase_id = 4
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { },
                         { }, { "Baltimore" }, { }),
      "Take the exit on the left toward Baltimore.");

  // phrase_id = 5
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { "67A" },
                         { }, { "Baltimore" }, { }),
      "Take exit 67A on the left toward Baltimore.");

  // phrase_id = 6
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Take the I 95 South exit on the left toward Baltimore.");

  // phrase_id = 7
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { "67A" },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Take exit 67A on the left onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { },
                         { }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left.");

  // phrase_id = 10
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15.");

  // phrase_id = 12
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { }, { },
                         { "Harrisburg", "Gettysburg" }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitLeftInstruction(
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft, { },
                         { "US 15" }, { "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15 toward Harrisburg/Gettysburg.");

}

}

int main() {
  test::suite suite("maneuversbuilder");

  // FormExitRightInstruction
  suite.test(TEST_CASE(TestFormExitRightInstruction));

  // FormExitLeftInstruction
  suite.test(TEST_CASE(TestFormExitLeftInstruction));

  return suite.tear_down();
}
