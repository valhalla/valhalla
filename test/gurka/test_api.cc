#include "gurka.h"
#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

using namespace valhalla;

TEST(api, minimal) {
}

TEST(api, pbf_in) {
  const std::string ascii_map = R"(A-----B-----C)";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_api_minimal");

  std::unordered_set<Options::Action> pbf_actions{
      Options::route,    Options::trace_route,      Options::optimized_route,
      Options::centroid, Options::trace_attributes, Options::status,
  };

  for (int action = Options::Action_MIN; action <= Options::Action_MAX; ++action) {
    // don't have convenient support of these in gurka yet
    if (action == Options::sources_to_targets || action == Options::isochrone ||
        action == Options::expansion)
      continue;

    std::string expected_json;
    auto api = gurka::do_action(Options::Action(action), map, {"A", "C"}, "pedestrian", {}, {},
                                &expected_json);

    Api pbf_in_json_out;
    pbf_in_json_out.mutable_options()->CopyFrom(api.options());
    pbf_in_json_out.mutable_options()->clear_costing_options();
    auto actual_json = gurka::do_action(map, pbf_in_json_out);
    EXPECT_EQ(actual_json, expected_json);

    if (pbf_actions.count(Options::Action(action))) {
      Api pbf_in_pbf_out;
      pbf_in_pbf_out.mutable_options()->CopyFrom(api.options());
      pbf_in_pbf_out.mutable_options()->clear_costing_options();
      auto pbf_bytes = gurka::do_action(map, pbf_in_json_out);
      Api actual_pbf;
      EXPECT_TRUE(actual_pbf.ParseFromString(pbf_bytes));
      EXPECT_EQ(actual_pbf.trip().SerializeAsString(), api.trip().SerializeAsString());
    }
  }
}
