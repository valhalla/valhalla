#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Warning, DeprecatedParams) {
  for (std::string costing : {"auto_shorter", "hov", "auto_data_fix"}) {
    Api request;
    std::string request_str =
        R"({"best_paths": 2, "locations": [{"lat": 0.0, "lon": 0.0},{"lat": 1.0, "lon": 1.0}], "costing": ")" +
        costing + R"("})";
    ParseApi(request_str, Options::route, request);
    EXPECT_EQ(request.info().warnings_size(), 2);
  }
}
