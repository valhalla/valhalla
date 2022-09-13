#include "test.h"

#include <iostream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

const auto conf = test::make_config("test/data/utrecht_tiles");

struct route_tester {
  route_tester()
      : reader(new GraphReader(conf.get_child("mjolnir"))), loki_worker(conf, reader),
        thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json, std::string& response_json) {
    Api request;
    ParseApi(request_json, Options::route, request);
    loki_worker.route(request);
    std::pair<std::list<TripLeg>, std::list<DirectionsLeg>> results;
    thor_worker.route(request);
    response_json = odin_worker.narrate(request);
    return request;
  }
  std::shared_ptr<GraphReader> reader;
  loki_worker_t loki_worker;
  thor_worker_t thor_worker;
  odin_worker_t odin_worker;
};

void test_alternates(int num_alternates) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.111893,"lon":5.125282},
      {"lat":52.113731,"lon":5.091155}],"costing":"auto",
      "alternates":)" +
      std::to_string(num_alternates) + "}";

  std::string json;
  auto response = tester.test(request, json);
  const auto& routes = response.trip().routes();
  ASSERT_EQ(routes.size(), num_alternates + 1);
  std::unordered_set<std::string> shapes;
  for (const auto& route : routes) {
    shapes.insert(route.legs(0).shape());
  }
  ASSERT_EQ(shapes.size(), num_alternates + 1);

  rapidjson::Document doc;
  doc.Parse(json);
  ASSERT_FALSE(doc.HasParseError());
  ASSERT_TRUE(doc.HasMember("trip"));
  ASSERT_TRUE(doc.HasMember("alternates") == (num_alternates > 0));
  if (num_alternates > 0) {
    for (const auto& alt : doc["alternates"].GetArray()) {
      ASSERT_TRUE(alt.HasMember("trip"));
    }
  }
}
} // namespace

TEST(Alternates, test_zero_alternates) {
  test_alternates(0);
}

TEST(Alternates, test_one_alternate) {
  test_alternates(1);
}

TEST(Alternates, test_two_alternates) {
  test_alternates(2);
}
