#include "gurka.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <vector>

using namespace valhalla;

TEST(StandAlone, DeprecatedParams) {
  for (const std::string& costing : {"auto_shorter", "hov", "auto_data_fix"}) {
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
