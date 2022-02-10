#include "test.h"

#include "valhalla/baldr/curl_tilegetter.h"
#include "valhalla/skadi/sample.h"

namespace {

struct TestSample : public valhalla::skadi::sample {
  static uint16_t get_tile_index(const valhalla::midgard::PointLL& coord) {
    return valhalla::skadi::sample::get_tile_index(coord);
  }
};

TEST(Sample, test_make_single_point_url) {
  struct test_case {
    std::string url;
    valhalla::midgard::PointLL point;
    std::string result;
  };
  // clang-format off
  std::vector<test_case> tests{{{"http://s3.amazonaws.com/mapzen.valhalla{tilePath}"},
                                {-73.512143, 40.646556},
                                {"http://s3.amazonaws.com/mapzen.valhalla/N40/N40W074.hgt"}},
                               {{"http://s3.amazonaws.com/mapzen.valhalla{tilePath}"},
                                {-78.504476, -0.212297},
                                {"http://s3.amazonaws.com/mapzen.valhalla/S01/S01W079.hgt"}},
                               {{"http://s3.amazonaws.com/mapzen.valhalla{tilePath}"},
                                {152.967888, -27.533658},
                                {"http://s3.amazonaws.com/mapzen.valhalla/S28/S28E152.hgt"}},
                               {{"http://s3.amazonaws.com/mapzen.valhalla{tilePath}"},
                                {139.737345, 35.628096},
                                {"http://s3.amazonaws.com/mapzen.valhalla/N35/N35E139.hgt"}},
                               {{"http://s3.amazonaws.com/mapzen.valhalla{tilePath}"},
                                {4.898484, 52.380697},
                                {"http://s3.amazonaws.com/mapzen.valhalla/N52/N52E004.hgt"}}};

  for (auto&& test : tests) {
    EXPECT_EQ(test.result, valhalla::baldr::make_single_point_url(test.url,
                    valhalla::skadi::get_hgt_file_name(TestSample::get_tile_index(test.point))));
  }
  // clang-format on
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}