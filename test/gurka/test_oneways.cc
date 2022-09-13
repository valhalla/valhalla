#include "gurka.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>
#include <gtest/gtest.h>
#include <valhalla/proto/options.pb.h>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "loki/polygon_search.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"
#include "sif/costfactory.h"
#include "worker.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::vector<std::string>& costing = {"auto",    "taxi",          "bus",        "truck",
                                           "bicycle", "motor_scooter", "motorcycle", "pedestrian"};

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};
constexpr double gridsize_metres = 100;

// parameterized test class to test all costings
class OnewayTest
    : public ::testing::TestWithParam<std::tuple<std::string, std::map<std::string, std::string>>> {
protected:
  static gurka::ways ways;
  static std::string ascii_map;
  static std::map<std::string, midgard::PointLL> layout;
  static void SetUpTestSuite() {
    ascii_map = R"(
        A---B---C---D---E
                    |   |
        J---I---H---G---F
                                     )";

    ways = {
        {"AB", {{"highway", "unclassified"}}}, {"BC", {{"highway", "unclassified"}}},
        {"CD", {{"highway", "unclassified"}}}, {"DE", {{"highway", "unclassified"}}},
        {"EF", {{"highway", "unclassified"}}}, {"FG", {{"highway", "unclassified"}}},
        {"GH", {{"highway", "unclassified"}}}, {"HI", {{"highway", "unclassified"}}},
        {"IJ", {{"highway", "unclassified"}}},
    };
    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  }
};

gurka::ways OnewayTest::ways = {};
std::string OnewayTest::ascii_map = {};
std::map<std::string, midgard::PointLL> OnewayTest::layout = {};

TEST_P(OnewayTest, TestOneways) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  test_ways.emplace("DG", std::get<1>(GetParam()));

  auto map = gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == "pedestrian" && c != param_cost)
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestReverseOneways) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "highway" || w.first == "oneway")
      updated_attributes.emplace(w.first, w.second);
    else {
      updated_attributes.emplace(w.first, "-1");
      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_reverse_oneway", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || c == "pedestrian" || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else // still has to go around the block...oneway:<mode>=-1 should have no impact on this costing.
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOnewaysWithYes) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "oneway")
      continue; // toss oneway=yes
    else {
      updated_attributes.emplace(w.first, w.second);

      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_with_yes", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOnewaysWithOnewayNo) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "oneway")
      updated_attributes.emplace(w.first, "no");
    else {
      updated_attributes.emplace(w.first, w.second);

      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map = gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_no", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOnewaysWithNo) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "highway" || w.first == "oneway")
      updated_attributes.emplace(w.first, w.second);
    else {
      updated_attributes.emplace(w.first, "no");
      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }

  test_ways.emplace("DG", updated_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_with_no", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == param_cost || c == "pedestrian" || (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestSharedLane) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  auto way_attributes = std::get<1>(GetParam());
  way_attributes.emplace("cycleway", "shared_lane");
  test_ways.emplace("DG", way_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_shared", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == "pedestrian" && c != param_cost)
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOpposite) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  auto way_attributes = std::get<1>(GetParam());
  way_attributes.emplace("cycleway", "opposite");
  test_ways.emplace("DG", way_attributes);

  auto map =
      gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_opposite", build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);

  if (param_cost == "bicycle") // opposite flips the onewayness
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ"});
  else
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if ((c == "pedestrian" && c != param_cost) || c == "bicycle")
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST_P(OnewayTest, TestOppositeWithNo) {

  const auto& param_cost = std::get<0>(GetParam());
  auto test_ways = ways;
  bool is_psv = false;
  auto way_attributes = std::get<1>(GetParam());
  std::map<std::string, std::string> updated_attributes;

  for (auto const& w : way_attributes) {
    if (w.first == "highway" || w.first == "oneway")
      updated_attributes.emplace(w.first, w.second);
    else {
      updated_attributes.emplace(w.first, "no");
      if (w.first == "oneway:psv") // taxi and bus onewayness will be updated.
        is_psv = true;
    }
  }
  updated_attributes.emplace("cycleway", "opposite");
  test_ways.emplace("DG", updated_attributes);

  auto map = gurka::buildtiles(layout, test_ways, {}, {}, "test/data/gurka_oneway_opposite_no",
                               build_config);
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, param_cost);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});

  // loop over the other modes
  for (auto const& c : costing) {
    result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    if (c == "pedestrian" || c == "bicycle" || c == param_cost ||
        (is_psv && (c == "bus" || c == "taxi")))
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "DG", "CD", "BC", "AB"});
    else
      gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

INSTANTIATE_TEST_SUITE_P(
    OnewayProfilesTest,
    OnewayTest,
    ::testing::Values(std::make_tuple("taxi",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:psv", "yes"}}),
                      std::make_tuple("taxi",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:taxi", "yes"}}),
                      std::make_tuple("bus",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:psv", "yes"}}),
                      std::make_tuple("bus",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:bus", "yes"}}),
                      std::make_tuple("bicycle",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:bicycle", "yes"}}),
                      std::make_tuple("motor_scooter",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:mofa", "yes"}}),
                      std::make_tuple("motor_scooter",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:moped", "yes"}}),
                      std::make_tuple("motorcycle",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:motorcycle",
                                                                          "yes"}}),
                      std::make_tuple("pedestrian",
                                      std::map<std::string, std::string>{{"highway", "unclassified"},
                                                                         {"oneway", "yes"},
                                                                         {"oneway:foot", "yes"}})));
