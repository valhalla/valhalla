#include "test.h"

#include <string>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::loki;
using namespace valhalla::tyr;

namespace {

struct route_tester {
  route_tester()
      : conf(test::make_config("test/data/utrecht_tiles")),
        reader(new GraphReader(conf.get_child("mjolnir"))), loki_worker(conf, reader),
        thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json) {
    Api request;
    ParseApi(request_json, Options::route, request);
    loki_worker.route(request);
    thor_worker.route(request);
    odin_worker.narrate(request);
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
    return request;
  }
  boost::property_tree::ptree conf;
  std::shared_ptr<GraphReader> reader;
  loki_worker_t loki_worker;
  thor_worker_t thor_worker;
  odin_worker_t odin_worker;
};

// http://localhost:8002/route?json={"locations":[{"lat":52.10160225589803,"lon":5.116925239562988},{"lat":52.09403591712907,"lon":5.113234519958496}],"costing":"auto","directions_options":{"units":"miles","format":"osrm"}}&access_token=
TEST(Urban2, test_urban) {
  bool is_urban;
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.10160225589803,"lon":5.116925239562988},{"lat":52.09403591712907,"lon":5.113234519958496}],"costing":"auto","units":"miles","format":"osrm","filters":{"action":"include","attributes":["edge.is_urban"]}})";
  auto response = tester.test(request);

  // get the osrm json
  auto json_str = serializeDirections(response);
  rapidjson::Document json;
  json.Parse(json_str);

  ASSERT_FALSE(json.HasParseError());

  // loop over all routes all legs
  for (const auto& route : json["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("is_urban")) {
            is_urban = intersection["is_urban"].GetBool();
            EXPECT_EQ(is_urban, true);
          }
        }
      }
    }
  }
}

TEST(Urban2, test_urban_excluded_by_default) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.10160225589803,"lon":5.116925239562988},{"lat":52.09403591712907,"lon":5.113234519958496}],"costing":"auto","units":"miles","format":"osrm"})";
  auto response = tester.test(request);

  // get the osrm json
  auto json_str = serializeDirections(response);
  rapidjson::Document json;
  json.Parse(json_str);

  ASSERT_FALSE(json.HasParseError());

  // loop over all routes all legs
  for (const auto& route : json["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          EXPECT_EQ(intersection.HasMember("is_urban"), false);
        }
      }
    }
  }
}

} // namespace

int main(int argc, char* argv[]) {
  valhalla::midgard::logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
