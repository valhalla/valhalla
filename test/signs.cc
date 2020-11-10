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
                   std::vector<valhalla::odin::Sign>* sign_list) {
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

Signs GetGuideSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& branch_sign_items,
                    const std::vector<std::tuple<std::string, bool, uint32_t>>& toward_sign_items) {
  Signs signs;
  PopulateSigns(branch_sign_items, signs.mutable_guide_branch_list());
  PopulateSigns(toward_sign_items, signs.mutable_guide_toward_list());

  return signs;
}

Signs GetJunctionNameSigns(const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  Signs signs;
  PopulateSigns(sign_items, signs.mutable_junction_name_list());

  return signs;
}

void TryGetExitNumberString(const Signs& signs,
                            uint32_t max_count,
                            bool limit_by_consecutive_count,
                            const std::string& expectedString) {

  EXPECT_EQ(signs.GetExitNumberString(max_count, limit_by_consecutive_count), expectedString);
}

void TryGetExitBranchString(const Signs& signs,
                            uint32_t max_count,
                            bool limit_by_consecutive_count,
                            const std::string& expectedString) {

  EXPECT_EQ(signs.GetExitBranchString(max_count, limit_by_consecutive_count), expectedString);
}

void TryGetExitTowardString(const Signs& signs,
                            uint32_t max_count,
                            bool limit_by_consecutive_count,
                            const std::string& expectedString) {

  EXPECT_EQ(signs.GetExitTowardString(max_count, limit_by_consecutive_count), expectedString);
}

void TryGetExitNameString(const Signs& signs,
                          uint32_t max_count,
                          bool limit_by_consecutive_count,
                          const std::string& expectedString) {

  EXPECT_EQ(signs.GetExitNameString(max_count, limit_by_consecutive_count), expectedString);
}

void TryGetGuideBranchString(const Signs& signs,
                             uint32_t max_count,
                             bool limit_by_consecutive_count,
                             const std::string& expectedString) {

  EXPECT_EQ(signs.GetGuideBranchString(max_count, limit_by_consecutive_count), expectedString);
}

void TryGetGuideTowardString(const Signs& signs,
                             uint32_t max_count,
                             bool limit_by_consecutive_count,
                             const std::string& expectedString) {

  EXPECT_EQ(signs.GetGuideTowardString(max_count, limit_by_consecutive_count), expectedString);
}

void TryGetGuideString(const Signs& signs,
                       uint32_t max_count,
                       bool limit_by_consecutive_count,
                       const std::string& expectedString) {

  EXPECT_EQ(signs.GetGuideString(max_count, limit_by_consecutive_count), expectedString);
}

void TryGetGuideSigns(const Signs& signs,
                      uint32_t max_count,
                      bool limit_by_consecutive_count,
                      const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  std::vector<valhalla::odin::Sign> expected_signs;
  PopulateSigns(sign_items, &expected_signs);
  ASSERT_THAT(signs.GetGuideSigns(max_count, limit_by_consecutive_count), expected_signs);
}

void TryTrimSigns(const std::vector<valhalla::odin::Sign>& signs,
                  const std::vector<std::tuple<std::string, bool, uint32_t>>& sign_items) {
  std::vector<valhalla::odin::Sign> expected_signs;
  PopulateSigns(sign_items, &expected_signs);
  ASSERT_THAT(signs, expected_signs);
}

void TryGetJunctionNameString(const Signs& signs,
                              uint32_t max_count,
                              bool limit_by_consecutive_count,
                              const std::string& expectedString) {

  EXPECT_EQ(signs.GetJunctionNameString(max_count, limit_by_consecutive_count), expectedString);
}

TEST(Signs, TestGetExitTowardString_PA283_onto_PA743) {
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

TEST(Signs, TestGetExitNumberString_I81S_onto_US322W) {
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

TEST(Signs, TestGetExitBranchString_I81S_onto_US322W) {
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

TEST(Signs, TestGetExitTowardString_I81S_onto_US322W) {
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

TEST(Signs, TestGetExitNameString) {
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

TEST(Signs, TestGetGuideBranchString_LinglestownRoad_onto_US322W) {
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

TEST(Signs, TestGetGuideTowardString_roundabout_toward_A1) {
  // Create toward sign
  // Specify input in descending consecutive count order
  Signs signs = GetGuideTowardSigns({std::make_tuple("A 1", 1, 1), std::make_tuple("Remscheid", 0, 1),
                                     std::make_tuple("Wermelskirchen", 0, 0)});

  TryGetGuideTowardString(signs, 4, false, "A 1/Remscheid/Wermelskirchen");
  TryGetGuideTowardString(signs, 2, false, "A 1/Remscheid");
  TryGetGuideTowardString(signs, 1, false, "A 1");

  TryGetGuideTowardString(signs, 4, true, "A 1/Remscheid");
  TryGetGuideTowardString(signs, 2, true, "A 1/Remscheid");
  TryGetGuideTowardString(signs, 1, true, "A 1");
}

TEST(Signs, TestGetGuideString_BranchOnly) {
  // Create guide sign
  // Specify input in descending consecutive count order
  Signs signs =
      GetGuideSigns({std::make_tuple("US 322 West", 1, 1), std::make_tuple("US 22 West", 1, 0),
                     std::make_tuple("Freedom Highway", 0, 0),
                     std::make_tuple("Valhalla Highway", 0, 0)},
                    {});

  TryGetGuideString(signs, 0, false, "US 322 West/US 22 West/Freedom Highway/Valhalla Highway");
  TryGetGuideString(signs, 4, false, "US 322 West/US 22 West/Freedom Highway/Valhalla Highway");
  TryGetGuideString(signs, 3, false, "US 322 West/US 22 West/Freedom Highway");
  TryGetGuideString(signs, 2, false, "US 322 West/US 22 West");
  TryGetGuideString(signs, 1, false, "US 322 West");

  TryGetGuideString(signs, 0, true, "US 322 West");
  TryGetGuideString(signs, 4, true, "US 322 West");
  TryGetGuideString(signs, 2, true, "US 322 West");
  TryGetGuideString(signs, 1, true, "US 322 West");
}

TEST(Signs, TestGetGuideString_TowardOnly) {
  // Create guide sign
  // Specify input in descending consecutive count order
  Signs signs = GetGuideSigns({}, {std::make_tuple("A 1", 1, 1), std::make_tuple("Remscheid", 0, 1),
                                   std::make_tuple("Wermelskirchen", 0, 0),
                                   std::make_tuple("Hückeswagen", 0, 0)});

  TryGetGuideString(signs, 0, false, "A 1/Remscheid/Wermelskirchen/Hückeswagen");
  TryGetGuideString(signs, 4, false, "A 1/Remscheid/Wermelskirchen/Hückeswagen");
  TryGetGuideString(signs, 3, false, "A 1/Remscheid/Wermelskirchen");
  TryGetGuideString(signs, 2, false, "A 1/Remscheid");
  TryGetGuideString(signs, 1, false, "A 1");

  TryGetGuideString(signs, 0, true, "A 1/Remscheid");
  TryGetGuideString(signs, 4, true, "A 1/Remscheid");
  TryGetGuideString(signs, 2, true, "A 1/Remscheid");
  TryGetGuideString(signs, 1, true, "A 1");
}

TEST(Signs, TestGetGuideString_NoConsecutiveCount) {
  // Create guide sign
  // Specify input in descending consecutive count order
  Signs signs =
      GetGuideSigns({std::make_tuple("US 322 West", 1, 0), std::make_tuple("US 22 West", 1, 0),
                     std::make_tuple("Freedom Highway", 0, 0),
                     std::make_tuple("Valhalla Highway", 0, 0)},
                    {std::make_tuple("A 1", 1, 0), std::make_tuple("Remscheid", 0, 0),
                     std::make_tuple("Wermelskirchen", 0, 0), std::make_tuple("Hückeswagen", 0, 0)});

  TryGetGuideString(
      signs, 0, false,
      "US 322 West/US 22 West/Freedom Highway/Valhalla Highway/A 1/Remscheid/Wermelskirchen/Hückeswagen");
  TryGetGuideString(signs, 4, false, "US 322 West/US 22 West/A 1/Remscheid");
  TryGetGuideString(signs, 3, false, "US 322 West/US 22 West/A 1");
  TryGetGuideString(signs, 2, false, "US 322 West/A 1");
  TryGetGuideString(signs, 1, false, "US 322 West");

  TryGetGuideString(
      signs, 0, true,
      "US 322 West/US 22 West/Freedom Highway/Valhalla Highway/A 1/Remscheid/Wermelskirchen/Hückeswagen");
  TryGetGuideString(signs, 4, true, "US 322 West/US 22 West/A 1/Remscheid");
  TryGetGuideString(signs, 3, true, "US 322 West/US 22 West/A 1");
  TryGetGuideString(signs, 2, true, "US 322 West/A 1");
  TryGetGuideString(signs, 1, true, "US 322 West");
}

TEST(Signs, TestGetGuideString_SingleConsecutiveCount) {
  // Create guide sign
  // Specify input in descending consecutive count order
  Signs signs =
      GetGuideSigns({std::make_tuple("US 322 West", 1, 1), std::make_tuple("US 22 West", 1, 0),
                     std::make_tuple("Freedom Highway", 0, 0),
                     std::make_tuple("Valhalla Highway", 0, 0)},
                    {std::make_tuple("A 1", 1, 1), std::make_tuple("Remscheid", 0, 0),
                     std::make_tuple("Wermelskirchen", 0, 0), std::make_tuple("Hückeswagen", 0, 0)});

  TryGetGuideString(
      signs, 0, false,
      "US 322 West/US 22 West/Freedom Highway/Valhalla Highway/A 1/Remscheid/Wermelskirchen/Hückeswagen");
  TryGetGuideString(signs, 4, false, "US 322 West/US 22 West/A 1/Remscheid");
  TryGetGuideString(signs, 3, false, "US 322 West/US 22 West/A 1");
  TryGetGuideString(signs, 2, false, "US 322 West/A 1");
  TryGetGuideString(signs, 1, false, "US 322 West");

  TryGetGuideString(signs, 0, true, "US 322 West/A 1");
  TryGetGuideString(signs, 4, true, "US 322 West/A 1");
  TryGetGuideString(signs, 3, true, "US 322 West/A 1");
  TryGetGuideString(signs, 2, true, "US 322 West/A 1");
  TryGetGuideString(signs, 1, true, "US 322 West");
}

TEST(Signs, TestGetGuideString_MultipleConsecutiveCount) {
  // Create guide sign
  // Specify input in descending consecutive count order
  Signs signs =
      GetGuideSigns({std::make_tuple("US 322 West", 1, 2), std::make_tuple("US 22 West", 1, 2),
                     std::make_tuple("Freedom Highway", 0, 1),
                     std::make_tuple("Valhalla Highway", 0, 0)},
                    {std::make_tuple("A 1", 1, 2), std::make_tuple("Remscheid", 0, 2),
                     std::make_tuple("Wermelskirchen", 0, 1), std::make_tuple("Hückeswagen", 0, 0)});

  TryGetGuideString(
      signs, 0, false,
      "US 322 West/US 22 West/Freedom Highway/Valhalla Highway/A 1/Remscheid/Wermelskirchen/Hückeswagen");
  TryGetGuideString(signs, 4, false, "US 322 West/US 22 West/A 1/Remscheid");
  TryGetGuideString(signs, 3, false, "US 322 West/US 22 West/A 1");
  TryGetGuideString(signs, 2, false, "US 322 West/A 1");
  TryGetGuideString(signs, 1, false, "US 322 West");

  TryGetGuideString(signs, 0, true, "US 322 West/US 22 West/A 1/Remscheid");
  TryGetGuideString(signs, 4, true, "US 322 West/US 22 West/A 1/Remscheid");
  TryGetGuideString(signs, 3, true, "US 322 West/US 22 West/A 1");
  TryGetGuideString(signs, 2, true, "US 322 West/A 1");
  TryGetGuideString(signs, 1, true, "US 322 West");
}

TEST(Signs, TestGetJunctionNameString) {
  // Create named junction sign
  // Specify input in descending consecutive count order
  Signs signs = GetJunctionNameSigns(
      {std::make_tuple("万年橋東", 0, 1), std::make_tuple("Mannenbashi East", 0, 0)});

  TryGetJunctionNameString(signs, 4, false, "万年橋東/Mannenbashi East");
  TryGetJunctionNameString(signs, 2, false, "万年橋東/Mannenbashi East");
  TryGetJunctionNameString(signs, 1, false, "万年橋東");

  TryGetJunctionNameString(signs, 4, true, "万年橋東");
  TryGetJunctionNameString(signs, 2, true, "万年橋東");
  TryGetJunctionNameString(signs, 1, true, "万年橋東");
}

TEST(Signs, TestGetGuideSigns_BranchOnly) {
  // Create guide sign
  // Specify input in descending consecutive count order
  auto US_322_West = std::make_tuple("US 322 West", 1, 1);
  auto US_22_West = std::make_tuple("US 22 West", 1, 0);
  auto Freedom_Highway = std::make_tuple("Freedom Highway", 0, 0);
  auto Valhalla_Highway = std::make_tuple("Valhalla Highway", 0, 0);

  Signs signs = GetGuideSigns({US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway}, {});

  TryGetGuideSigns(signs, 0, false, {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway});
  // Try with max_count > length of branch list
  TryGetGuideSigns(signs, 10, false, {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway});
  TryGetGuideSigns(signs, 4, false, {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway});
  TryGetGuideSigns(signs, 3, false, {US_322_West, US_22_West, Freedom_Highway});
  TryGetGuideSigns(signs, 2, false, {US_322_West, US_22_West});
  TryGetGuideSigns(signs, 1, false, {US_322_West});

  TryGetGuideSigns(signs, 0, true, {US_322_West});
  TryGetGuideSigns(signs, 4, true, {US_322_West});
  TryGetGuideSigns(signs, 3, true, {US_322_West});
  TryGetGuideSigns(signs, 2, true, {US_322_West});
  TryGetGuideSigns(signs, 1, true, {US_322_West});
}

TEST(Signs, TestGetGuideSigns_TowardOnly) {
  // Create guide sign
  // Specify input in descending consecutive count order
  auto A_1 = std::make_tuple("A 1", 1, 1);
  auto Remscheid = std::make_tuple("Remscheid", 0, 1);
  auto Wermelskirchen = std::make_tuple("Wermelskirchen", 0, 0);
  auto Huckeswagen = std::make_tuple("Hückeswagen", 0, 0);

  Signs signs = GetGuideSigns({}, {A_1, Remscheid, Wermelskirchen, Huckeswagen});

  TryGetGuideSigns(signs, 0, false, {A_1, Remscheid, Wermelskirchen, Huckeswagen});
  // Try with max_count > length of toward list
  TryGetGuideSigns(signs, 10, false, {A_1, Remscheid, Wermelskirchen, Huckeswagen});
  TryGetGuideSigns(signs, 4, false, {A_1, Remscheid, Wermelskirchen, Huckeswagen});
  TryGetGuideSigns(signs, 3, false, {A_1, Remscheid, Wermelskirchen});
  TryGetGuideSigns(signs, 2, false, {A_1, Remscheid});
  TryGetGuideSigns(signs, 1, false, {A_1});

  TryGetGuideSigns(signs, 0, true, {A_1, Remscheid});
  TryGetGuideSigns(signs, 4, true, {A_1, Remscheid});
  TryGetGuideSigns(signs, 3, true, {A_1, Remscheid});
  TryGetGuideSigns(signs, 2, true, {A_1, Remscheid});
  TryGetGuideSigns(signs, 1, true, {A_1});
}

TEST(Signs, TestGetGuideSigns_NoConsecutiveCount) {
  // Create guide sign
  // Specify input in descending consecutive count order
  auto US_322_West = std::make_tuple("US 322 West", 1, 0);
  auto US_22_West = std::make_tuple("US 22 West", 1, 0);
  auto Freedom_Highway = std::make_tuple("Freedom Highway", 0, 0);
  auto Valhalla_Highway = std::make_tuple("Valhalla Highway", 0, 0);
  auto A_1 = std::make_tuple("A 1", 1, 0);
  auto Remscheid = std::make_tuple("Remscheid", 0, 0);
  auto Wermelskirchen = std::make_tuple("Wermelskirchen", 0, 0);
  auto Huckeswagen = std::make_tuple("Hückeswagen", 0, 0);

  Signs signs = GetGuideSigns({US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway},
                              {A_1, Remscheid, Wermelskirchen, Huckeswagen});

  TryGetGuideSigns(signs, 0, false,
                   {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway, A_1, Remscheid,
                    Wermelskirchen, Huckeswagen});
  TryGetGuideSigns(signs, 4, false, {US_322_West, US_22_West, A_1, Remscheid});
  TryGetGuideSigns(signs, 3, false, {US_322_West, US_22_West, A_1});
  TryGetGuideSigns(signs, 2, false, {US_322_West, A_1});
  TryGetGuideSigns(signs, 1, false, {US_322_West});

  TryGetGuideSigns(signs, 0, true,
                   {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway, A_1, Remscheid,
                    Wermelskirchen, Huckeswagen});
  TryGetGuideSigns(signs, 4, true, {US_322_West, US_22_West, A_1, Remscheid});
  TryGetGuideSigns(signs, 3, true, {US_322_West, US_22_West, A_1});
  TryGetGuideSigns(signs, 2, true, {US_322_West, A_1});
  TryGetGuideSigns(signs, 1, true, {US_322_West});
}

TEST(Signs, TestGetGuideSigns_SingleConsecutiveCount) {
  // Create guide sign
  // Specify input in descending consecutive count order
  auto US_322_West = std::make_tuple("US 322 West", 1, 1);
  auto US_22_West = std::make_tuple("US 22 West", 1, 0);
  auto Freedom_Highway = std::make_tuple("Freedom Highway", 0, 0);
  auto Valhalla_Highway = std::make_tuple("Valhalla Highway", 0, 0);
  auto A_1 = std::make_tuple("A 1", 1, 1);
  auto Remscheid = std::make_tuple("Remscheid", 0, 0);
  auto Wermelskirchen = std::make_tuple("Wermelskirchen", 0, 0);
  auto Huckeswagen = std::make_tuple("Hückeswagen", 0, 0);

  Signs signs = GetGuideSigns({US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway},
                              {A_1, Remscheid, Wermelskirchen, Huckeswagen});

  TryGetGuideSigns(signs, 0, false,
                   {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway, A_1, Remscheid,
                    Wermelskirchen, Huckeswagen});
  TryGetGuideSigns(signs, 4, false, {US_322_West, US_22_West, A_1, Remscheid});
  TryGetGuideSigns(signs, 3, false, {US_322_West, US_22_West, A_1});
  TryGetGuideSigns(signs, 2, false, {US_322_West, A_1});
  TryGetGuideSigns(signs, 1, false, {US_322_West});

  TryGetGuideSigns(signs, 0, true, {US_322_West, A_1});
  TryGetGuideSigns(signs, 4, true, {US_322_West, A_1});
  TryGetGuideSigns(signs, 3, true, {US_322_West, A_1});
  TryGetGuideSigns(signs, 2, true, {US_322_West, A_1});
  TryGetGuideSigns(signs, 1, true, {US_322_West});
}

TEST(Signs, TestGetGuideSigns_MultipleConsecutiveCount) {
  // Create guide sign
  // Specify input in descending consecutive count order
  auto US_322_West = std::make_tuple("US 322 West", 1, 2);
  auto US_22_West = std::make_tuple("US 22 West", 1, 2);
  auto Freedom_Highway = std::make_tuple("Freedom Highway", 0, 1);
  auto Valhalla_Highway = std::make_tuple("Valhalla Highway", 0, 0);
  auto A_1 = std::make_tuple("A 1", 1, 2);
  auto Remscheid = std::make_tuple("Remscheid", 0, 2);
  auto Wermelskirchen = std::make_tuple("Wermelskirchen", 0, 1);
  auto Huckeswagen = std::make_tuple("Hückeswagen", 0, 0);

  Signs signs = GetGuideSigns({US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway},
                              {A_1, Remscheid, Wermelskirchen, Huckeswagen});

  TryGetGuideSigns(signs, 0, false,
                   {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway, A_1, Remscheid,
                    Wermelskirchen, Huckeswagen});
  TryGetGuideSigns(signs, 4, false, {US_322_West, US_22_West, A_1, Remscheid});
  TryGetGuideSigns(signs, 3, false, {US_322_West, US_22_West, A_1});
  TryGetGuideSigns(signs, 2, false, {US_322_West, A_1});
  TryGetGuideSigns(signs, 1, false, {US_322_West});

  TryGetGuideSigns(signs, 0, true, {US_322_West, US_22_West, A_1, Remscheid});
  TryGetGuideSigns(signs, 4, true, {US_322_West, US_22_West, A_1, Remscheid});
  TryGetGuideSigns(signs, 3, true, {US_322_West, US_22_West, A_1});
  TryGetGuideSigns(signs, 2, true, {US_322_West, A_1});
  TryGetGuideSigns(signs, 1, true, {US_322_West});
}

TEST(Signs, TestTrimSigns) {
  auto US_322_West = std::make_tuple("US 322 West", 1, 2);
  auto US_22_West = std::make_tuple("US 22 West", 1, 2);
  auto Freedom_Highway = std::make_tuple("Freedom Highway", 0, 1);
  auto Valhalla_Highway = std::make_tuple("Valhalla Highway", 0, 0);

  std::vector<valhalla::odin::Sign> signs;
  PopulateSigns({US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway}, &signs);

  // Test defaults (max_count = 0, limit_by_consecutive_count = false)
  TryTrimSigns(Signs::TrimSigns(signs), {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway});
  // max_count > input list size
  TryTrimSigns(Signs::TrimSigns(signs, 10, false),
               {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway});
  TryTrimSigns(Signs::TrimSigns(signs, 0, false),
               {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway});
  TryTrimSigns(Signs::TrimSigns(signs, 4, false),
               {US_322_West, US_22_West, Freedom_Highway, Valhalla_Highway});
  TryTrimSigns(Signs::TrimSigns(signs, 3, false), {US_322_West, US_22_West, Freedom_Highway});
  TryTrimSigns(Signs::TrimSigns(signs, 2, false), {US_322_West, US_22_West});
  TryTrimSigns(Signs::TrimSigns(signs, 1, false), {US_322_West});

  TryTrimSigns(Signs::TrimSigns(signs, 10, true), {US_322_West, US_22_West});
  TryTrimSigns(Signs::TrimSigns(signs, 4, true), {US_322_West, US_22_West});
  TryTrimSigns(Signs::TrimSigns(signs, 3, true), {US_322_West, US_22_West});
  TryTrimSigns(Signs::TrimSigns(signs, 2, true), {US_322_West, US_22_West});
  TryTrimSigns(Signs::TrimSigns(signs, 1, false), {US_322_West});
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
