#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

TEST(MatrixTest, MatrixSimple) {
  constexpr double gridsize = 1000;

  // the grid below is 50 x 100 meters at 36 km/h, i.e. 10 m/s
  const std::string ascii_map = R"(
      A----B
      |    |
      |    |
      C----D
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"AC", {{"highway", "residential"}}},
      {"BD", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/shortest");

  const std::vector<double> exp_dists = {3.f, 8.f, 8.f, 3.f};
  const std::vector<uint32_t> exp_times = {308, 823, 823, 308};
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, map, {"A", "B"}, {"C", "D"},
                                       "auto", {}, {}, &res);

  // get the MultiLineString feature
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  size_t i = 0;
  for (const auto& origin_row : res_doc["sources_to_targets"].GetArray()) {
    auto origin_td = origin_row.GetArray();
    for (const auto& v : origin_td) {
      std::string msg = "Problem at source " + std::to_string(i / origin_td.Size()) + " and target " +
                        std::to_string(i % origin_td.Size());
      EXPECT_EQ(v.GetObject()["distance"].GetDouble(), exp_dists[i]) << msg;
      EXPECT_EQ(v.GetObject()["time"].GetUint(), exp_times[i]) << msg;
      i++;
    }
  }
}
