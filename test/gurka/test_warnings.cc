#include "gurka.h"
#include <gtest/gtest.h>
#include <vector>

using namespace valhalla;

TEST(Warning, DeprecatedParams) {
  for (auto& costing : {"auto_shorter", "hov", "auto_data_fix"}) {
    Api request;
    std::string request_str = valhalla::gurka::detail::build_valhalla_request("locations", {{1.0, 1.0}, {2.0, 2.0}}, costing, {{"/best_paths", "2"}});
    ParseApi(request_str, Options::route, request);
    EXPECT_EQ(request.info().warnings_size(), 2);
  }
}
