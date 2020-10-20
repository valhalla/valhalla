#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include "valhalla/worker.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::loki;
using namespace valhalla::tyr;

namespace {

TEST(AutoShorterDeprecated, test_auto_shorter_replace) {
  // Test if auto_shorter will be converted to auto with shortest: true
  Api request;
  std::string request_str =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto_shorter"})";
  ParseApi(request_str, Options::route, request);

  EXPECT_EQ(Costing_Enum_Name(request.options().costing()), "auto");
  EXPECT_EQ(request.options().costing_options(static_cast<int>(auto_)).shortest(), true);
}

TEST(AutoShorterDeprecated, test_auto_shorter_shortest_false) {
  // test if the costing_options are copied from auto_shortest
  Api request;
  std::string request_str =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto_shorter",)"
      R"("costing_options":{"auto_shorter":{"shortest":false, "use_ferry":0.1}}})";
  ParseApi(request_str, Options::route, request);

  EXPECT_EQ(Costing_Enum_Name(request.options().costing()), "auto");
  EXPECT_EQ(request.options().costing_options(static_cast<int>(auto_)).shortest(), true);
  EXPECT_EQ(request.options().costing_options(static_cast<int>(auto_)).use_ferry(), 0.1f);
}

TEST(AutoShorterDeprecated, test_auto_shorter_shortest_true) {
  // if both auto & auto_shorter costing options were provided, auto costing should be overridden
  Api request;
  std::string request_str =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto_shorter",)"
      R"("costing_options":{"auto":{"use_ferry":0.8}, "auto_shorter":{"use_ferry":0.1, "use_tolls": 0.1}}})";
  ParseApi(request_str, Options::route, request);

  EXPECT_EQ(Costing_Enum_Name(request.options().costing()), "auto");
  EXPECT_EQ(request.options().costing_options(static_cast<int>(auto_)).shortest(), true);
  EXPECT_EQ(request.options().costing_options(static_cast<int>(auto_)).use_ferry(), 0.1f);
  EXPECT_EQ(request.options().costing_options(static_cast<int>(auto_)).use_tolls(), 0.1f);
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}