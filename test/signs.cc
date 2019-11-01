#include <algorithm>
#include <cstdint>
#include <vector>

#include "odin/sign.h"
#include "odin/signs.h"

#include "test.h"

using namespace std;
using namespace valhalla::odin;

namespace {

constexpr size_t TEXT = 0;
constexpr size_t IS_ROUTE_NUMBER = 1;
constexpr size_t CONSECUTIVE_COUNT = 2;

void PopulateSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items,
                   std::vector<Sign>* sign_list) {
  for (auto& sign_item : sign_items) {
    sign_list->emplace_back(std::get<TEXT>(sign_item), std::get<IS_ROUTE_NUMBER>(sign_item));
    sign_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(sign_item));
  }
}

Signs GetExitNumberSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  Signs signs;
  PopulateSigns(sign_items, signs.mutable_exit_number_list());

  return signs;
}

Signs GetExitBranchSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  Signs signs;
  PopulateSigns(sign_items, signs.mutable_exit_branch_list());

  return signs;
}

Signs GetExitTowardSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  Signs signs;
  PopulateSigns(sign_items, signs.mutable_exit_toward_list());

  return signs;
}

Signs GetExitNameSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  Signs signs;
  PopulateSigns(sign_items, signs.mutable_exit_name_list());

  return signs;
}

Signs GetGuideBranchSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  Signs signs;
  PopulateSigns(sign_items, signs.mutable_guide_branch_list());

  return signs;
}

Signs GetGuideTowardSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  Signs signs;
  PopulateSigns(sign_items, signs.mutable_guide_toward_list());

  return signs;
}

void TryGetExitNumberString(const Signs& signs,
                            uint32_t max_count,
                            bool limit_by_consecutive_count,
                            const std::string& expectedString) {

  if (signs.GetExitNumberString(max_count, limit_by_consecutive_count) != expectedString) {
    throw std::runtime_error("Incorrect Exit Number String - expected: " + expectedString);
  }
}

void TryGetExitBranchString(const Signs& signs,
                            uint32_t max_count,
                            bool limit_by_consecutive_count,
                            const std::string& expectedString) {

  if (signs.GetExitBranchString(max_count, limit_by_consecutive_count) != expectedString) {
    throw std::runtime_error("Incorrect Exit Branch String - expected: " + expectedString);
  }
}

void TryGetExitTowardString(const Signs& signs,
                            uint32_t max_count,
                            bool limit_by_consecutive_count,
                            const std::string& expectedString) {

  if (signs.GetExitTowardString(max_count, limit_by_consecutive_count) != expectedString) {
    throw std::runtime_error("Incorrect Exit Toward String - expected: " + expectedString);
  }
}

void TryGetExitNameString(const Signs& signs,
                          uint32_t max_count,
                          bool limit_by_consecutive_count,
                          const std::string& expectedString) {

  if (signs.GetExitNameString(max_count, limit_by_consecutive_count) != expectedString) {
    throw std::runtime_error("Incorrect Exit Name String - expected: " + expectedString);
  }
}

void TryGetGuideBranchString(const Signs& signs,
                             uint32_t max_count,
                             bool limit_by_consecutive_count,
                             const std::string& expectedString) {

  if (signs.GetGuideBranchString(max_count, limit_by_consecutive_count) != expectedString) {
    throw std::runtime_error("Incorrect Guide Branch String - expected: " + expectedString);
  }
}

void TryGetGuideTowardString(const Signs& signs,
                             uint32_t max_count,
                             bool limit_by_consecutive_count,
                             const std::string& expectedString) {

  if (signs.GetGuideTowardString(max_count, limit_by_consecutive_count) != expectedString) {
    throw std::runtime_error("Incorrect Guide Toward String - expected: " + expectedString);
  }
}

void TestGetExitTowardString_PA283_onto_PA743() {
  // Create toward sign
  // Specify input in descending consecutive count order
  Signs signs =
      GetExitTowardSigns({std::make_tuple("Elizabethtown", 0, 1), std::make_tuple("Hershey", 0, 0)});

  TryGetExitTowardString(signs, 4, false, "Elizabethtown/Hershey");
  TryGetExitTowardString(signs, 2, false, "Elizabethtown/Hershey");
  TryGetExitTowardString(signs, 1, false, "Elizabethtown");

  TryGetExitTowardString(signs, 2, true, "Elizabethtown");
  TryGetExitTowardString(signs, 1, true, "Elizabethtown");
}

void TestGetExitNumberString_I81S_onto_US322W() {
  // Create number sign
  // Specify input in descending consecutive count order
  Signs signs = GetExitNumberSigns({std::make_tuple("67B", 0, 1), std::make_tuple("67A", 0, 0)});

  TryGetExitNumberString(signs, 4, false, "67B/67A");
  TryGetExitNumberString(signs, 2, false, "67B/67A");
  TryGetExitNumberString(signs, 1, false, "67B");

  TryGetExitNumberString(signs, 4, true, "67B");
  TryGetExitNumberString(signs, 2, true, "67B");
  TryGetExitNumberString(signs, 1, true, "67B");
}

void TestGetExitBranchString_I81S_onto_US322W() {
  // Create branch sign
  // Specify input in descending consecutive count order
  Signs signs =
      GetExitBranchSigns({std::make_tuple("US 322 West", 1, 2), std::make_tuple("US 22 West", 1, 1),
                          std::make_tuple("US 22 East", 1, 0), std::make_tuple("PA 230 East", 1, 0),
                          std::make_tuple("Cameron Street", 0, 0)});

  TryGetExitBranchString(signs, 0, false,
                         "US 322 West/US 22 West/US 22 East/PA 230 East/Cameron Street");
  TryGetExitBranchString(signs, 5, false,
                         "US 322 West/US 22 West/US 22 East/PA 230 East/Cameron Street");
  TryGetExitBranchString(signs, 4, false, "US 322 West/US 22 West/US 22 East/PA 230 East");
  TryGetExitBranchString(signs, 2, false, "US 322 West/US 22 West");
  TryGetExitBranchString(signs, 1, false, "US 322 West");

  TryGetExitBranchString(signs, 0, true, "US 322 West");
  TryGetExitBranchString(signs, 5, true, "US 322 West");
  TryGetExitBranchString(signs, 4, true, "US 322 West");
  TryGetExitBranchString(signs, 2, true, "US 322 West");
  TryGetExitBranchString(signs, 1, true, "US 322 West");
}

void TestGetExitTowardString_I81S_onto_US322W() {
  // Create toward sign
  // Specify input in descending consecutive count order
  Signs signs =
      GetExitTowardSigns({std::make_tuple("Lewistown", 0, 1), std::make_tuple("State College", 0, 1),
                          std::make_tuple("Harrisburg", 0, 0)});

  TryGetExitTowardString(signs, 4, false, "Lewistown/State College/Harrisburg");
  TryGetExitTowardString(signs, 2, false, "Lewistown/State College");
  TryGetExitTowardString(signs, 1, false, "Lewistown");

  TryGetExitTowardString(signs, 4, true, "Lewistown/State College");
  TryGetExitTowardString(signs, 2, true, "Lewistown/State College");
  TryGetExitTowardString(signs, 1, true, "Lewistown");
}

void TestGetExitNameString() {
  // Create name sign
  // Specify input in descending consecutive count order
  Signs signs = GetExitNameSigns(
      {std::make_tuple("Gettysburg Pike", 0, 1), std::make_tuple("Harrisburg Pike", 0, 0)});

  TryGetExitNameString(signs, 4, false, "Gettysburg Pike/Harrisburg Pike");
  TryGetExitNameString(signs, 2, false, "Gettysburg Pike/Harrisburg Pike");
  TryGetExitNameString(signs, 1, false, "Gettysburg Pike");

  TryGetExitNameString(signs, 4, true, "Gettysburg Pike");
  TryGetExitNameString(signs, 2, true, "Gettysburg Pike");
  TryGetExitNameString(signs, 1, true, "Gettysburg Pike");
}

void TestGetGuideBranchString_LinglestownRoad_onto_US322W() {
  // Create branch sign
  // Specify input in descending consecutive count order
  Signs signs = GetGuideBranchSigns(
      {std::make_tuple("US 322 West", 1, 1), std::make_tuple("US 22 West", 1, 0)});

  TryGetGuideBranchString(signs, 0, false, "US 322 West/US 22 West");
  TryGetGuideBranchString(signs, 4, false, "US 322 West/US 22 West");
  TryGetGuideBranchString(signs, 2, false, "US 322 West/US 22 West");
  TryGetGuideBranchString(signs, 1, false, "US 322 West");

  TryGetGuideBranchString(signs, 0, true, "US 322 West");
  TryGetGuideBranchString(signs, 4, true, "US 322 West");
  TryGetGuideBranchString(signs, 2, true, "US 322 West");
  TryGetGuideBranchString(signs, 1, true, "US 322 West");
}

void TestGetGuideTowardString_roundabout_toward_A1() {
  // Create toward sign
  // Specify input in descending consecutive count order
  Signs signs = GetGuideTowardSigns({std::make_tuple("A 1", 0, 1), std::make_tuple("Remscheid", 0, 1),
                                     std::make_tuple("Wermelskirchen", 0, 0)});

  TryGetGuideTowardString(signs, 4, false, "A 1/Remscheid/Wermelskirchen");
  TryGetGuideTowardString(signs, 2, false, "A 1/Remscheid");
  TryGetGuideTowardString(signs, 1, false, "A 1");

  TryGetGuideTowardString(signs, 4, true, "A 1/Remscheid");
  TryGetGuideTowardString(signs, 2, true, "A 1/Remscheid");
  TryGetGuideTowardString(signs, 1, true, "A 1");
}

} // namespace

int main() {
  test::suite suite("signs");

  // GetExitTowardString_PA283_onto_PA743
  suite.test(TEST_CASE(TestGetExitTowardString_PA283_onto_PA743));

  // GetExitNumberString_I81S_onto_US322W
  suite.test(TEST_CASE(TestGetExitNumberString_I81S_onto_US322W));

  // GetExitBranchString_I81S_onto_US322W
  suite.test(TEST_CASE(TestGetExitBranchString_I81S_onto_US322W));

  // GetExitTowardString_I81S_onto_US322W
  suite.test(TEST_CASE(TestGetExitTowardString_I81S_onto_US322W));

  // GetExitNameString
  suite.test(TEST_CASE(TestGetExitNameString));

  // GetGuideBranchString_LinglestownRoad_onto_US322W
  suite.test(TEST_CASE(TestGetGuideBranchString_LinglestownRoad_onto_US322W));

  // GetGuideTowardString_roundabout_toward_A1
  suite.test(TEST_CASE(TestGetGuideTowardString_roundabout_toward_A1));

  return suite.tear_down();
}
