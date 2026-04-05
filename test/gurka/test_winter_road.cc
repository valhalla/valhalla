// Test that OSM winter_road=yes and ice_road=yes tags apply a strong cost
// penalty so the router prefers normal roads when an alternative exists,
// but still routes across seasonal roads when there is no other option.
// Covers issue #6025.

#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {

// Map layout:
//
//   A---B---C
//       |   |
//       D---E
//
// AB, BC, CE, DE: normal primary roads
// BD: winter_road=yes shortcut (same length as CE, but penalized)
//
// Route A→E: prefer A-B-C-E (all normal) over A-B-D-E (shortcut via winter road)

const std::string kAsciiMap = R"(
    A---B---C
        |   |
        D---E
)";

const gurka::ways kWays = {
    {"AB", {{"highway", "primary"}}},
    {"BC", {{"highway", "primary"}}},
    {"CE", {{"highway", "primary"}}},
    {"DE", {{"highway", "primary"}}},
    {"BD", {{"highway", "primary"}, {"winter_road", "yes"}}},
};

TEST(WinterRoad, AvoidsWinterRoadWhenAlternativeExists) {
  const auto layout = gurka::detail::map_to_coordinates(kAsciiMap, 100);
  auto map = gurka::buildmap(layout, kWays, {});

  // Router should choose A-B-C-E (normal roads) over A-B-D-E (BD is winter_road)
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE"});
}

// Same topology but BD tagged ice_road=yes instead
const gurka::ways kWaysIce = {
    {"AB", {{"highway", "primary"}}},
    {"BC", {{"highway", "primary"}}},
    {"CE", {{"highway", "primary"}}},
    {"DE", {{"highway", "primary"}}},
    {"BD", {{"highway", "primary"}, {"ice_road", "yes"}}},
};

TEST(WinterRoad, AvoidsIceRoadWhenAlternativeExists) {
  const auto layout = gurka::detail::map_to_coordinates(kAsciiMap, 100);
  auto map = gurka::buildmap(layout, kWaysIce, {});

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE"});
}

// When only a winter road connects origin to destination, routing must still succeed.
const std::string kAsciiMapDirect = R"(
    A---B
)";

const gurka::ways kWaysDirect = {
    {"AB", {{"highway", "primary"}, {"winter_road", "yes"}}},
};

TEST(WinterRoad, RoutesAcrossWinterRoadWhenNoAlternative) {
  const auto layout = gurka::detail::map_to_coordinates(kAsciiMapDirect, 100);
  auto map = gurka::buildmap(layout, kWaysDirect, {});

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB"});
}

} // namespace
