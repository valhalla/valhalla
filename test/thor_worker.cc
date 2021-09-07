#include "test.h"

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "thor/attributes_controller.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include <algorithm>
#include <thread>
#include <unistd.h>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::thor;
using namespace std::string_literals;

namespace {

// fake config
const auto conf = test::make_config("test/data/utrecht_tiles");

TEST(ThorWorker, test_parse_filter_attributes_defaults) {
  tyr::actor_t actor(conf, true);

  auto result = test::json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}]})"));

  EXPECT_FALSE(result.get_child_optional("shape_attributes")) << "Expected excluded shape_attributes";

  EXPECT_TRUE(result.get_child_optional("edges")) << "Expected included edges";

  EXPECT_TRUE(result.get_child_optional("shape")) << "Expected included shape";
  EXPECT_FALSE(result.get_child_optional("edge.show_incidents"))
      << "Expected excluded edge.show_incidents";
}

TEST(ThorWorker, test_parse_filter_attributes_excludes) {
  tyr::actor_t actor(conf, true);

  std::vector<std::string> test_cases = {
      actor.trace_attributes(
          R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],
          "filters":{"attributes":["shape"], "action":"exclude"}})"),
      // TODO currently EnhancedTripPath sets any excluded fields to their default value
      // so this test can't check for excluded keys. Come back and add tests for
      // shape_attributes once we have them since those will be untouched by ETP.
      // actor.trace_route(
      //     R"({"costing":"auto","shape_match":"map_snap","shape":[
      //     {"lat":52.09110,"lon":5.09806},
      //     {"lat":52.09098,"lon":5.09679}],
      //     "filters":{"attributes":["shape"], "action":"exclude"}})"),
      // actor.route(
      //     R"({"costing":"auto","locations":[
      //     {"lat":52.09110,"lon":5.09806},
      //     {"lat":52.09098,"lon":5.09679}],
      //     "filters":{"attributes":["shape"], "action":"exclude"}})"),
  };
  std::vector<std::string> excluded_keys = {"shape", "trip.legs..shape", "trip.legs..shape"};

  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto result = test::json_to_pt(test_cases[i]);
    EXPECT_FALSE(result.get_child_optional(excluded_keys[i]))
        << "Expected excluded shape | found " + excluded_keys[i] + "=" +
               result.get<std::string>(excluded_keys[i]);
  }
}

TEST(ThorWorker, test_parse_filter_attributes_includes) {
  tyr::actor_t actor(conf, true);

  std::vector<std::string> test_cases = {
      actor.trace_attributes(
          R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],"filters":{"attributes":["shape"], "action":"include"}})"),
      actor.trace_route(
          R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],
          "filters":{"attributes":["shape"], "action":"include"}})"),
      actor.route(
          R"({"costing":"auto","locations":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],"filters":{"attributes":["shape"],
          "action":"include"}})"),
  };
  std::vector<std::string> included_keys = {"shape", "trip.legs..shape", "trip.legs..shape"};

  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto result = test::json_to_pt(test_cases[i]);
    EXPECT_TRUE(result.get_child_optional(included_keys[i]))
        << "Expected " + included_keys[i] + " to be present";
  }
}

TEST(ThorWorker, test_linear_references) {
  std::vector<std::string> requests = {
      R"({"costing":"auto","linear_references":true,"locations":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],
          "action":"include"})",
  };
  const std::vector<std::string>& expected = {
      "CwOgEyUK5SKXAP/H//wiBw==",
      "CwOf+CUK4iKXAP/k//8iBw==",
      "CwOf6yUK4SKXAP/Y//0iBw==",
  };
  tyr::actor_t actor(conf, true);
  for (const auto& request : requests) {
    auto result = test::json_to_pt(actor.route(request));
    std::vector<std::string> references;
    for (const auto& reference : result.get_child("trip.linear_references"))
      references.push_back(reference.second.get_value<std::string>());
    EXPECT_EQ(references.size(), 3);
    EXPECT_EQ(expected, references);
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
