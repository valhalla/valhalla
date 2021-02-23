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

  const gurka::ways ways = {{"AB", {{"highway", "footway"}, {"sac_scale", "hiking"}}},
                            {"BC", {{"highway", "footway"}, {"sac_scale", "alpine_hiking"}}},
                            {"CD", {{"highway", "footway"}}}};

  const double gridsize = 10;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/sac_scale_attributes");

  auto result = gurka::do_action(valhalla::Options::trace_attributes, map, {"1", "2", "3", "4", "5"},
                                 "pedestrian", {}, {}, nullptr, "via");

  auto d = gurka::convert_to_json(result, valhalla::Options_Format_json);
  auto edges = d["edges"].GetArray();

  ASSERT_EQ(edges.Size(), 3);

  /*
    EXPECT_TRUE(edges[0].HasMember("sac_scale"));
    EXPECT_STREQ(edges[0]["sac_scale"].GetString(), "hiking");
    EXPECT_TRUE(edges[1].HasMember("sac_scale"));
    EXPECT_STREQ(edges[1]["sac_scale"].GetString(), "alpine_hiking");
    EXPECT_FALSE(edges[2].HasMember("sac_scale"));
  */
}
