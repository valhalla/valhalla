#include "gurka.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, SacScaleAttributes) {

  const std::string ascii_map = R"(
      1
    A---2B-3-4C
              |
              |5
              D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "track"}, {"sac_scale", "hiking"}}},
                            {"BC", {{"highway", "track"}, {"sac_scale", "alpine_hiking"}}},
                            {"CD", {{"highway", "track"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/sac_scale_attributes");

  std::string trace_json;
  auto api = gurka::do_action(valhalla::Options::trace_attributes, map, {"1", "2", "3", "4", "5"},
                              "pedestrian", {{"/costing_options/pedestrian/max_hiking_difficulty", "5"}},
                               {}, &trace_json, "via");

  rapidjson::Document result;
  result.Parse(trace_json.c_str());

  auto edges = result["edges"].GetArray();
  ASSERT_EQ(edges.Size(), 3);

  EXPECT_TRUE(edges[0].HasMember("sac_scale"));
  EXPECT_EQ(edges[0]["sac_scale"].GetInt(), 1);
  EXPECT_TRUE(edges[1].HasMember("sac_scale"));
  EXPECT_EQ(edges[1]["sac_scale"].GetInt(), 4);
  EXPECT_TRUE(edges[2].HasMember("sac_scale"));
  EXPECT_EQ(edges[2]["sac_scale"].GetInt(), 0);
}
