#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "tyr/actor.h"

#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

namespace {

// fake up config against pine grove traffic extract
const auto conf = test::make_config(VALHALLA_SOURCE_DIR "test/traffic_matcher_tiles");

TEST(Actor, Basic) {
  tyr::actor_t actor(conf);

  actor.route(R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
      {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route_json = actor.route(R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
          {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route = test::json_to_pt(route_json);
  route_json.find("Tulpehocken");

  actor.trace_attributes(R"({"shape":[{"lat":40.546115,"lon":-76.385076},
      {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})");
  actor.cleanup();
  auto attributes_json = actor.trace_attributes(R"({"shape":[{"lat":40.546115,"lon":-76.385076},
      {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})");
  actor.cleanup();
  auto attributes = test::json_to_pt(attributes_json);
  attributes_json.find("Tulpehocken");

  actor.transit_available(R"({"locations":[{"lat":35.647452, "lon":-79.597477, "radius":20},
      {"lat":34.766908, "lon":-80.325936,"radius":10}]})");
  actor.cleanup();
  auto transit_json =
      actor.transit_available(R"({"locations":[{"lat":35.647452, "lon":-79.597477, "radius":20},
      {"lat":34.766908, "lon":-80.325936,"radius":10}]})");
  actor.cleanup();
  auto transit = test::json_to_pt(transit_json);
  transit_json.find(std::to_string(false));

  // TODO: test the rest of them
}

struct test_exception_t {};

TEST(Actor, Route) {
  tyr::actor_t actor(conf);
  std::string request = R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
        {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})";
  std::function<void()> interrupt = [] { throw test_exception_t{}; };
  EXPECT_THROW(actor.route(request, &interrupt), test_exception_t);
}

TEST(Actor, TraceAttributes) {
  tyr::actor_t actor(conf);
  std::string request = R"({"shape":[{"lat":40.546115,"lon":-76.385076},
        {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})";
  std::function<void()> interrupt = [] { throw test_exception_t{}; };
  EXPECT_THROW(actor.trace_attributes(request, &interrupt), test_exception_t);
}

// TODO: test the rest of them

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
