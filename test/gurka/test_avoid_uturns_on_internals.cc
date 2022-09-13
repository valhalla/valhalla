#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

const std::vector<std::string>& costing = {"auto",    "taxi",          "bus",        "truck",
                                           "bicycle", "motor_scooter", "motorcycle", "pedestrian"};

TEST(Standalone, AvoidUturnsOnInternals) {
  constexpr double gridsize_metres = 5;

  const std::string ascii_map = R"(
             C D
             | |
             | |
             | |
             E-F-------G-------H
            1| |       |
             | |       |
             | |2      |3
             | |       I
             J-K--L        
             | |  |
             | |  N
             | |     
             O-P---Q-------R
             | |   |       |
             S-T---U-------V
             | |
             | |
             | |
             W X

    )";

  const gurka::ways ways = {
      {"CEJOSW", {{"highway", "tertiary"}, {"name", "Rijkerstreek"}, {"oneway", "yes"}}},
      {"XTPKFD", {{"highway", "tertiary"}, {"name", "Rijkerstreek"}, {"oneway", "yes"}}},
      {"JK", {{"highway", "tertiary"}}},
      {"OP", {{"highway", "tertiary"}}},
      {"ST", {{"highway", "tertiary"}}},
      {"FGH", {{"highway", "unclassified"}, {"name", "Toekanweg"}}},
      {"GI", {{"highway", "unclassified"}, {"name", "Maraboeweg"}}},
      {"KLN", {{"highway", "unclassified"}}},
      {"PQ", {{"highway", "unclassified"}, {"name", "Valkweg"}}},
      {"TU", {{"highway", "unclassified"}, {"name", "Valkweg"}}},
      {"QRVU", {{"highway", "service"}}},
      {"QU", {{"highway", "unclassified"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_avoid_uturns_on_internals",
                               build_config);

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"1", "3"}, c);
    if (c == "bicycle" || c == "motor_scooter" || c == "motorcycle" || c == "pedestrian")
      gurka::assert::raw::expect_path(result, {"Rijkerstreek", "JK", "Rijkerstreek", "Toekanweg",
                                               "Maraboeweg"});
    else
      gurka::assert::raw::expect_path(result, {"Rijkerstreek", "Rijkerstreek", "OP", "Valkweg", "QU",
                                               "Valkweg", "Rijkerstreek", "Rijkerstreek",
                                               "Rijkerstreek", "Toekanweg", "Maraboeweg"});
  }

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"1", "2"}, c);
    if (c == "bicycle" || c == "motor_scooter" || c == "motorcycle" || c == "pedestrian")
      gurka::assert::raw::expect_path(result, {"Rijkerstreek", "JK", "Rijkerstreek"});
    else
      gurka::assert::raw::expect_path(result,
                                      {"Rijkerstreek", "Rijkerstreek", "OP", "Valkweg", "QU",
                                       "Valkweg", "Rijkerstreek", "Rijkerstreek", "Rijkerstreek"});
  }
}
