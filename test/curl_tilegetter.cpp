#include "test.h"

#include "valhalla/baldr/curl_tilegetter.h"
#include "valhalla/skadi/sample.h"

namespace {

TEST(Sample, test_make_single_point_url) {
    struct test_case {
        std::string url;
        midgard::PointLL point;
        std::string result;
    };

    std::vector <test_case> tests{{{"http://s3.amazonaws.com/mapzen.valhalla{DataPath}"}, {-73.512143, 40.646556},  {"http://s3.amazonaws.com/mapzen.valhalla/N40/N40W074.hgt"}},
                                  {{"http://s3.amazonaws.com/mapzen.valhalla{DataPath}"}, {-78.504476, -0.212297},  {"http://s3.amazonaws.com/mapzen.valhalla/S01/S01W079.hgt"}},
                                  {{"http://s3.amazonaws.com/mapzen.valhalla{DataPath}"}, {152.967888, -27.533658}, {"http://s3.amazonaws.com/mapzen.valhalla/S28/S28E152.hgt"}},
                                  {{"http://s3.amazonaws.com/mapzen.valhalla{DataPath}"}, {139.737345, 35.628096},  {"http://s3.amazonaws.com/mapzen.valhalla/N35/N35E139.hgt"}},
                                  {{"http://s3.amazonaws.com/mapzen.valhalla{DataPath}"}, {4.898484,   52.380697},  {"http://s3.amazonaws.com/mapzen.valhalla/N52/N52E004.hgt"}}};

    // clang-form off
    valhalla::baldr::curl_tile_getter_t tile_getter{1, "", false};
    tile_getter.set_path_pattern("{DataPath}");
    for (auto &&test : tests) {
        EXPECT_EQ(test.result, tile_getter.test_make_single_point_url(test.url,
            sample::get_hgt_file_name(sample::get_tile_index(test.point))));
    }
    // clang-format on
}

} // namespace

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}