#include <vector>

#include "gurka.h"
#include <gtest/gtest.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "gurka.h"

using namespace valhalla;

// NOTE, lots of warnings are also tested in other tests where they make more sense

TEST(StandAlone, DeprecatedParams) {
  for (const std::string costing : {"auto_shorter", "hov", "auto_data_fix"}) {
    Api request;
    std::string request_str =
        R"({"best_paths": 2, "locations": [{"lat": 0.0, "lon": 0.0},{"lat": 1.0, "lon": 1.0}], "costing": ")" +
        costing + R"("})";
    ParseApi(request_str, Options::route, request);
    // 2 because of best_path
    EXPECT_EQ(request.info().warnings_size(), 2);
  }
}

TEST(StandAlone, ClampedParameters) {
  const auto value_to_clamp = std::to_string(std::numeric_limits<float>::max());

  // just test a few options, can't test them all..
  std::vector<std::string> options = {"maneuver_penalty", "alley_penalty",
                                      "destination_only_penalty"};
  for (const std::string& option : options) {
    Api request;
    std::string request_str =
        R"({"costing":"auto","locations":[{"lat":0,"lon":0},{"lat":1,"lon": 1}],"costing_options":{"auto":{")" +
        option + R"(":)" + value_to_clamp + R"(}}})";
    ParseApi(request_str, Options::route, request);
    EXPECT_EQ(request.info().warnings().size(), 1);
    EXPECT_EQ(request.info().warnings(0).code(), 500);
    EXPECT_THAT(request.info().warnings(0).description(), testing::HasSubstr(option));
  }
}

TEST(Standalone, BidirAstarFallback) {
  constexpr double gridsize = 1000;

  const std::string ascii_map = R"(
      A-----B-----C
    )";

  const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                            {"BC", {{"highway", "residential"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  const auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/warnings_bidir_fallback",
                        {{"service_limits.max_timedep_distance", "1000"},
                         {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"}});

  // depart_at: falling back to bidir a* with time set
  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto",
                                {{"/date_time/type", "1"}, {"/date_time/value", "2020-10-30T09:00"}});
    EXPECT_EQ(api.info().warnings().size(), 1);
    EXPECT_EQ(api.info().warnings(0).code(), 402);
    EXPECT_THAT(api.info().warnings(0).description(),
                testing::HasSubstr("max_timedep_distance for depart_at"));
  }

  // arrive_by: falling back to bidir a* with time ignored
  {
    auto api = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto",
                                {{"/date_time/type", "2"}, {"/date_time/value", "2020-10-30T09:00"}});
    EXPECT_EQ(api.info().warnings().size(), 1);
    EXPECT_EQ(api.info().warnings(0).code(), 212);
    EXPECT_THAT(api.info().warnings(0).description(),
                testing::HasSubstr("max_timedep_distance for arrive_by"));
  }
}
