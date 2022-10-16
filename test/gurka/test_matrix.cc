#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Matrix, MatrixSimple) {
  constexpr double gridsize = 100;

  // the grid below is 50 x 100 meters at 36 km/h, i.e. 10 m/s
  const std::string ascii_map = R"(
    A-----B
    |     | \
    |     |  \
    C-----D---E
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}}, {"AC", {{"highway", "residential"}}},
      {"BD", {{"highway", "residential"}}}, {"CD", {{"highway", "residential"}}},
      {"DE", {{"highway", "residential"}}}, {"BE", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_simple");

  const std::vector<float> exp_dists = {0.f,  1.2f, 0.3f, 0.9f, 1.1f, 0.6f, 0.f,  0.9f, 0.3f,
                                        0.5f, 0.3f, 0.9f, 0.f,  1.2f, 1.f,  0.9f, 0.3f, 0.6f,
                                        0.f,  0.4f, 1.1f, 0.5f, 1.4f, 0.8f, 0.f};
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, map, {"A", "B", "C", "D", "E"},
                                       {"A", "B", "C", "D", "E"}, "auto", {}, {}, &res);

  // get the MultiLineString feature
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  size_t i = 0;
  for (const auto& origin_row : res_doc["sources_to_targets"].GetArray()) {
    auto origin_td = origin_row.GetArray();
    for (const auto& v : origin_td) {
      std::string msg = "Problem at source " + std::to_string(i / origin_td.Size()) + " and target " +
                        std::to_string(i % origin_td.Size());
      EXPECT_EQ(v.GetObject()["distance"].GetFloat(), exp_dists[i]) << msg;
      // EXPECT_EQ(v.GetObject()["time"].GetUint(), exp_times[i]) << msg;
      i++;
    }
  }
}
