#include "gurka.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, BasicMatch) {

  const std::string ascii_map = R"(
      1
    A---2B-3-4C
              |
              |5
              D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/basic_match");

  auto result = gurka::do_action(valhalla::Options::trace_route, map, {"1", "2", "3", "4", "5"},
                                 "auto", {}, {}, nullptr, "via");

  gurka::assert::osrm::expect_match(result, {"AB", "CD"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg::Maneuver::kStart,
                                                DirectionsLeg::Maneuver::kRight,
                                                DirectionsLeg::Maneuver::kDestination});

  gurka::assert::raw::expect_path_length(result, 0.100, 0.001);
}

#if 1
TEST(Standalone, messing_around) {

  gurka::nodelayout layout =
    {
//        {"0", {38.159604000000, 13.262611000000}},
//        {"1", {38.159587000000, 13.262670000000}},
//        {"2", {38.159551000000, 13.262678000000}},
//        {"3", {38.159492000000, 13.262654000000}},
//        {"4", {38.159471000000, 13.262608000000}},
//        {"5", {38.159414000000, 13.262630000000}},
//        {"6", {38.159391000000, 13.262686000000}},
//        {"7", {38.159325000000, 13.262726000000}},
//        {"8", {38.159230000000, 13.262753000000}},
//        {"9", {38.159102000000, 13.262694000000}},
//        {"10", {38.158925000000, 13.262587000000}},
//        {"11", {38.158813000000, 13.262576000000}},
//        {"12", {38.158667000000, 13.262568000000}},
//        {"13", {38.158551000000, 13.262544000000}},
//        {"14", {38.158471000000, 13.262587000000}},
//        {"15", {38.158404000000, 13.262683000000}},
//        {"16", {38.158402000000, 13.262903000000}},
//        {"17", {38.158402000000, 13.262903000000}},
//        {"18", {38.158387000000, 13.262989000000}},
//        {"19", {38.158395000000, 13.263257000000}},
//        {"20", {38.158480000000, 13.263298000000}},
//        {"21", {38.158619000000, 13.263333000000}},
//        {"22", {38.158731000000, 13.263365000000}},
//        {"23", {38.158939000000, 13.263483000000}},
//        {"24", {38.159068000000, 13.263539000000}},
//        {"25", {38.159296000000, 13.263552000000}},
//        {"26", {38.159500000000, 13.263544000000}},
//        {"27", {38.159741000000, 13.263493000000}},
//        {"28", {38.159887646896, 13.263439271611}},
//        {"29", {38.159973000000, 13.263408000000}},
//        {"30", {38.160215000000, 13.263333000000}},
//        {"31", {38.160313084784, 13.263274382017}},
//        {"32", {38.160371000000, 13.263188000000}},
//        {"33", {38.160426000000, 13.263174000000}},
//        {"34", {38.160454000000, 13.263249000000}},
//        {"35", {38.160373422496, 13.263399470031}},
//        {"36", {38.160210743575, 13.263565989551}},
//        {"37", {38.160198000000, 13.263577000000}},
//        {"38", {38.160110000000, 13.263662000000}},
//        {"39", {38.160101000000, 13.263716000000}},
//        {"40", {38.160114000000, 13.263802000000}},
//        {"41", {38.160289000000, 13.263692000000}},
//        {"42", {38.160289000000, 13.263692000000}},
//        {"43", {38.160485000000, 13.263563000000}},
//        {"44", {38.160605000000, 13.263499000000}},
//        {"45", {38.160702000000, 13.263392000000}},
//        {"46", {38.160833000000, 13.263129000000}},
//        {"47", {38.160833000000, 13.263129000000}},
//        {"48", {38.160911000000, 13.263161000000}},
//        {"49", {38.160889779480, 13.263327609695}},
//        {"50", {38.160865000000, 13.263394000000}},
//        {"51", {38.160799000000, 13.263628000000}},
//        {"52", {38.160638349111, 13.263906054699}},
//        {"53", {38.160629000000, 13.263917000000}},
//        {"54", {38.160470000000, 13.264132000000}},
        {"A", {38.160293000000, 13.264266000000}},
        {"B", {38.160293000000, 13.264266000000}},
        {"C", {38.160198000000, 13.264330000000}},
        {"D", {38.160125000000, 13.264486000000}},
        {"E", {38.159962000000, 13.264738000000}},
        {"F", {38.159762000000, 13.264974000000}},
        {"G", {38.159612000000, 13.265103000000}},
        {"H", {38.159441000000, 13.265261000000}},
        {"I", {38.159321000000, 13.265366000000}},
        {"J", {38.159146000000, 13.265556000000}},
        {"K", {38.159167000000, 13.265631000000}},
        {"L", {38.159247000000, 13.265655000000}},
        {"M", {38.159718000000, 13.265956000000}},
        {"N", {38.159859000000, 13.266039000000}},
        {"O", {38.160135880491, 13.266137423902}},
        {"P", {38.160371000000, 13.265985000000}},
        {"Q", {38.160464000000, 13.265875000000}},
        {"R", {38.160639000000, 13.265725000000}},
        {"S", {38.160745000000, 13.265666000000}},
        {"T", {38.160745000000, 13.265666000000}},
        {"U", {38.160884000000, 13.265620000000}},
        {"V", {38.161055000000, 13.265714000000}},
        {"W", {38.161238000000, 13.265779000000}},
        {"X", {38.161719000000, 13.265940000000}},
        {"Y", {38.161909000000, 13.266015000000}},
        {"Z", {38.162164000000, 13.266084000000}},
        {"1", {38.15965960, 13.26263720}},
        {"2", {38.15969960, 13.26260910}},
    };

  for (auto & iter : layout) {
    midgard::PointLL & point = iter.second;
    double temp = point.x();
    point.set_x(point.y());
    point.set_y(temp);
  }

  //const gurka::ways ways = {{"ABCDEFGHIJKLMNOPQRSTUVWXYZ", {{"highway", "primary"}}}};
  const gurka::ways ways = {{"OPQRSTUVWXYZ", {{"highway", "primary"}}}};
  //const gurka::ways ways = {{"ZYXWVUTSRQPONMLKJIHGFEDCBA", {{"highway", "primary"}}}};

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/messing_around");

  auto result = gurka::do_action(valhalla::Options::trace_route, map, {"1", "2"},
                       "auto", {}, {}, nullptr, "via");

}
#endif


TEST(Standalone, manual_attempt_at_segfault_didnt_work) {

  const std::string ascii_map = R"(
                            A
                            B
                            C
                            D
                            E
                            F
                            G
                            H
                            I
                            J
                            K
                            L
                            M
    1-----------------------N----------------------------2
                            O
                            P
                            Q
                            R
                            S
                            T
                            U
                            V
                            W
                            X
                            Y
                            Z
         )";

  const midgard::PointLL topleft = {13.2626372, 38.1596596};

  for (double i = 0.1; i < 10; i += 0.01) {

    auto layout = gurka::detail::map_to_coordinates(ascii_map, i, topleft);

    //  gurka::nodelayout layout =
    //      {
    //          { "A", {13.26261125, 38.15960393}},
    //          { "B", {13.26267000, 38.15958700}},
    //          { "1", {13.26263720, 38.15965960}},
    //          { "2", {13.26260910, 38.15969960}},
    //      };

    for ( double k = 1e-10; k < 1e-4; k /= 10.0) {
      int j = 0;
      for (char c = 'A'; c <= 'Z'; c++, j++) {
        std::string key = {c};
        midgard::PointLL& point = layout[key];
        point.set_x(point.lng() + (j % 5 - 2) * k);
        point.set_y(point.lat() + (j % 5 - 2) * k);
      }

      const gurka::ways ways = {{"1N", {{"highway", "primary"}}}, {"N2", {{"highway", "primary"}}}};

      auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/messing_around");

      auto result =
          gurka::do_action(valhalla::Options::trace_route, map,
                           {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
                            "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"},
                           "auto", {}, {}, nullptr, "via");
    }
  }
}


// not sure if this is a problem:
// unknown file: Failure
// C++ exception with description "No path could be found for input" thrown in the test body.
TEST(Standalone, weird_failure) {

  const std::string ascii_map = R"(
                            A
                            B
                            C
                            D
                            E
                            F
                            G
                            H
                            I
                            J
                            K
                            L
                            M
    1-----------------------N----------------------------2
                            O
                            P
                            Q
                            R
                            S
                            T
                            U
                            V
                            W
                            X
                            Y
                            Z
         )";

  const midgard::PointLL topleft = {13.2626372, 38.1596596};


   auto layout = gurka::detail::map_to_coordinates(ascii_map, 0.1, topleft);

  //  gurka::nodelayout layout =
  //      {
  //          { "A", {13.26261125, 38.15960393}},
  //          { "B", {13.26267000, 38.15958700}},
  //          { "1", {13.26263720, 38.15965960}},
  //          { "2", {13.26260910, 38.15969960}},
  //      };

  int j = 0;
  for (char c = 'A'; c <= 'Z'; c++, j++) {
    std::string key = {c};
    midgard::PointLL & point = layout[key];
    point.set_x( point.lng() + (j%3-1) * 1e-3 );
  }

  const gurka::ways ways = {{"1N", {{"highway", "primary"}}},
                            {"N2", {{"highway", "primary"}}}};

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/messing_around");

  auto result = gurka::do_action(valhalla::Options::trace_route, map,
                                 {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
                                  "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"},
                                 "auto", {}, {}, nullptr, "via");

}


TEST(Standalone, UturnMatch) {

  const std::string ascii_map = "A--1--2--B";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uturn_match");
  std::vector<std::string> expected_names;
  for (const auto& test_case : std::vector<std::pair<std::string, float>>{{"break", 90},
                                                                          {"via", 90},
                                                                          {"through", 210},
                                                                          {"break_through", 210}}) {

    auto result = gurka::do_action(valhalla::Options::trace_route, map, {"1", "2", "1", "2"}, "auto",
                                   {{"/trace_options/penalize_immediate_uturn", "0"}}, {}, nullptr,
                                   test_case.first);

    // throughs or vias will make a uturn without a destination notification (left hand driving)
    std::vector<DirectionsLeg::Maneuver::Type> expected_maneuvers{
        DirectionsLeg::Maneuver::kStart,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kUturnRight,
        DirectionsLeg::Maneuver::kDestination,
    };
    expected_names = {"AB", "AB", "AB"};

    // break will have destination at the trace point
    if (test_case.first == "break") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart, DirectionsLeg::Maneuver::kDestination,
      };
      expected_names = {"AB", "AB", "AB"};
    } // break through will have destinations at the trace points but will have to make uturns
    else if (test_case.first == "break_through") {
      expected_maneuvers = {
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kDestination,
          DirectionsLeg::Maneuver::kStart,       DirectionsLeg::Maneuver::kUturnRight,
          DirectionsLeg::Maneuver::kDestination, DirectionsLeg::Maneuver::kStart,
          DirectionsLeg::Maneuver::kUturnRight,  DirectionsLeg::Maneuver::kDestination,
      };
      expected_names = {"AB", "AB", "AB", "AB", "AB"};
    }

    gurka::assert::osrm::expect_match(result, {"AB"});
    gurka::assert::raw::expect_path(result, expected_names);
    gurka::assert::raw::expect_maneuvers(result, expected_maneuvers);
    gurka::assert::raw::expect_path_length(result, test_case.second / 1000, 0.001);
  }
}

TEST(Standalone, UturnTrimmingAsan) {
  const std::string ascii_map = R"(
A--1--2--B
         |
         |
D--3--4--C--5--6--E)";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"DCE", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/uturn_asan");

  auto result =
      gurka::do_action(valhalla::Options::trace_route, map, {"2", "6", "3"}, "auto",
                       {{"/trace_options/penalize_immediate_uturn", "0"}}, {}, nullptr, "via");

  auto shape =
      midgard::decode<std::vector<midgard::PointLL>>(result.trip().routes(0).legs(0).shape());
  // TODO: Remove the duplicate 6 when we fix odin to handle uturn maneuver generation with only one
  // turn around point
  auto expected_shape = decltype(shape){
      map.nodes["2"], map.nodes["B"], map.nodes["C"], map.nodes["6"],
      map.nodes["6"], map.nodes["C"], map.nodes["3"],
  };
  EXPECT_EQ(shape.size(), expected_shape.size());
  for (int i = 0; i < shape.size(); ++i) {
    EXPECT_TRUE(shape[i].ApproximatelyEqual(expected_shape[i]));
  }
}

/****************THIS BOILERPLATE WAS COPYPASTA'D FROM THE ROUTE TEST****************/
class TrafficBasedTest : public ::testing::Test {
protected:
  static gurka::map map;
  static uint32_t current, historical, constrained, freeflow;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      b----1----c----2----d
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      3         4         5
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      a----6----f----7----e
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      8         9         0
      |         |         |
      |         |         |
      |         |         |
      |         |         |
      g----A----h----B----i
    )";

    const gurka::ways ways = {
        {"abcdefaghie", {{"highway", "residential"}}},
        {"cfh", {{"highway", "tertiary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/match_timedep",
                            {
                                {"mjolnir.shortcuts", "false"},
                                {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            });
    map.config.put("mjolnir.traffic_extract", "test/data/match_timedep/traffic.tar");

    // add live traffic
    test::build_live_traffic_data(map.config);
    test::customize_live_traffic_data(map.config, [&](baldr::GraphReader&, baldr::TrafficTile&, int,
                                                      valhalla::baldr::TrafficSpeed* traffic_speed) {
      traffic_speed->overall_speed = 50 >> 1;
      traffic_speed->speed1 = 50 >> 1;
      traffic_speed->breakpoint1 = 255;
    });

    test::customize_historical_traffic(map.config, [](DirectedEdge& e) {
      e.set_constrained_flow_speed(25);
      e.set_free_flow_speed(75);
      std::array<float, kBucketsPerWeek> historical;
      historical.fill(7);
      for (size_t i = 0; i < historical.size(); ++i) {
        // TODO: if we are in morning or evening set a different speed and add another test
      }
      return historical;
    });

    // tests below use saturday at 9am and thursday 4am (27 if for leap seconds)
    size_t second_of_week_constrained = 5 * 24 * 60 * 60 + 9 * 60 * 60 + 27;
    size_t second_of_week_freeflow = 3 * 24 * 60 * 60 + 4 * 60 * 60 + 27;
    auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
    for (auto tile_id : reader->GetTileSet()) {
      auto tile = reader->GetGraphTile(tile_id);
      for (const auto& e : tile->GetDirectedEdges()) {
        current = tile->GetSpeed(&e, baldr::kCurrentFlowMask, 0);
        EXPECT_EQ(current, 50);
        historical = tile->GetSpeed(&e, baldr::kPredictedFlowMask, second_of_week_constrained);
        EXPECT_EQ(historical, 7);
        constrained = tile->GetSpeed(&e, baldr::kConstrainedFlowMask, second_of_week_constrained);
        EXPECT_EQ(constrained, 25);
        freeflow = tile->GetSpeed(&e, baldr::kFreeFlowMask, second_of_week_freeflow);
        EXPECT_EQ(freeflow, 75);
      }
    }
  }
};

gurka::map TrafficBasedTest::map = {};
uint32_t TrafficBasedTest::current = 0, TrafficBasedTest::historical = 0,
         TrafficBasedTest::constrained = 0, TrafficBasedTest::freeflow = 0;

uint32_t speed_from_edge(const valhalla::Api& api) {
  uint32_t kmh = -1;
  const auto& nodes = api.trip().routes(0).legs(0).node();
  for (int i = 0; i < nodes.size() - 1; ++i) {
    const auto& node = nodes.Get(i);
    if (!node.has_edge())
      break;
    auto km = node.edge().length_km();
    auto h = (nodes.Get(i + 1).cost().elapsed_cost().seconds() -
              node.cost().elapsed_cost().seconds() - node.cost().transition_cost().seconds()) /
             3600.0;
    auto new_kmh = static_cast<uint32_t>(km / h + .5);
    if (kmh != -1)
      EXPECT_EQ(kmh, new_kmh);
    kmh = new_kmh;
  }
  return kmh;
}

// map matching currently only allows forward time tracking and invariant so types 0, 1, 3.
// type 2 arrive_by is not yet supported
TEST_F(TrafficBasedTest, forward) {
  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"4", "0"}, "auto");
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), constrained);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "1"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "constrained"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), constrained);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "predicted"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), historical);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "0"},
                                 {"/date_time/value", "2020-10-30T09:00"},
                                 {"/costing_options/auto/speed_types/0", "current"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), current);
  }

  {
    auto api = gurka::do_action(valhalla::Options::trace_route, map, {"A", "2"}, "auto",
                                {{"/date_time/type", "3"},
                                 {"/date_time/value", "2020-10-28T04:00"},
                                 {"/costing_options/auto/speed_types/0", "freeflow"}});
    EXPECT_EQ(api.trip().routes(0).legs(0).algorithms(0), "map_snap");
    EXPECT_EQ(speed_from_edge(api), freeflow);
  }
}
