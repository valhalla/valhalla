#include "gurka.h"
#include "test.h"
#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

using namespace valhalla;

TEST(pbf_api, pbf_in_out) {
  const std::string ascii_map = R"(
    A-1--B--2-C
    |    |    |
    3    4    5
    |    |    |
    D-6--E--7-F
    |    |    |
    8    9    0
    |    |    |
    G-a--H--b-I)";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},    {"BC", {{"highway", "primary"}}},
      {"AD", {{"highway", "residential"}}}, {"BE", {{"highway", "motorway_link"}}},
      {"CF", {{"highway", "pedestrian"}}},  {"DE", {{"highway", "trunk"}}},
      {"EF", {{"highway", "secondary"}}},   {"DG", {{"highway", "trunk_link"}}},
      {"EH", {{"highway", "cycleway"}}},    {"FI", {{"highway", "service"}}},
      {"GH", {{"highway", "tertiary"}}},    {"HI", {{"highway", "unclassified"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_api_minimal");

  std::unordered_set<Options::Action> pbf_actions{Options::route,
                                                  Options::trace_route,
                                                  Options::optimized_route,
                                                  Options::centroid,
                                                  Options::trace_attributes,
                                                  Options::status,
                                                  Options::sources_to_targets,
                                                  Options::isochrone};

  PbfFieldSelector select_all;
  select_all.set_directions(true);
  select_all.set_trip(true);
  select_all.set_status(true);
  select_all.set_options(true);
  select_all.set_matrix(true);
  select_all.set_isochrone(true);

  for (int action = Options::no_action + 1; action <= Options::Action_MAX; ++action) {
    // don't have convenient support of these in gurka yet
    if (action == Options::expansion)
      continue;

    // do the regular request with json in and out
    std::string expected_json, request_json;
    Api expected_pbf;
    if (action == Options::sources_to_targets) {
      expected_pbf = gurka::do_action(Options::Action(action), map, {"A", "C"}, {"I", "G"},
                                      "pedestrian", {}, {}, &expected_json, &request_json);
    } else if (action == Options::isochrone) {
      expected_pbf =
          gurka::do_action(Options::Action(action), map, {"A"}, "pedestrian",
                           {{"/contours/0/time", "10"}}, {}, &expected_json, "break", &request_json);
    } else {
      expected_pbf = gurka::do_action(Options::Action(action), map, {"A", "C", "I", "G"},
                                      "pedestrian", {}, {}, &expected_json, "break", &request_json);
    }

    // get some clean input options for pbf requests
    Api clean_pbf;
    ParseApi(request_json, Options::Action(action), clean_pbf);

    // do the request still with json out but with pbf in
    Api json_out;
    json_out.mutable_options()->CopyFrom(clean_pbf.options());
    json_out.mutable_options()->clear_costings();
    auto actual_json = gurka::do_action(map, json_out);
    EXPECT_EQ(actual_json, expected_json);

    // if this action supports pbf out so try that
    if (pbf_actions.count(Options::Action(action))) {
      Api pbf_out;
      pbf_out.mutable_options()->CopyFrom(clean_pbf.options());
      pbf_out.mutable_options()->clear_costings();
      pbf_out.mutable_options()->set_format(Options::pbf);
      pbf_out.mutable_options()->mutable_pbf_field_selector()->CopyFrom(select_all);

      auto pbf_bytes = gurka::do_action(map, pbf_out);
      Api actual_pbf;
      EXPECT_TRUE(actual_pbf.ParseFromString(pbf_bytes));
      EXPECT_EQ(actual_pbf.trip().SerializeAsString(), expected_pbf.trip().SerializeAsString());
      EXPECT_TRUE(actual_pbf.has_options());
      EXPECT_TRUE(actual_pbf.has_trip() || action == Options::status ||
                  action == Options::sources_to_targets || action == Options::isochrone);
      EXPECT_TRUE(actual_pbf.has_directions() || action != Options::trace_route ||
                  action != Options::route);
      EXPECT_TRUE(actual_pbf.has_status() || action != Options::status);
      EXPECT_TRUE(actual_pbf.has_info() || action == Options::status);
      EXPECT_TRUE(actual_pbf.has_matrix() || action != Options::sources_to_targets);
      EXPECT_TRUE(actual_pbf.has_isochrone() || action != Options::isochrone);

      // lets try it again but this time we'll disable all the fields but one
      Api slimmed;
      slimmed.mutable_options()->CopyFrom(clean_pbf.options());
      slimmed.mutable_options()->clear_costings();
      slimmed.mutable_options()->set_format(Options::pbf);
      slimmed.mutable_options()->mutable_pbf_field_selector()->set_trip(true);
      pbf_bytes = gurka::do_action(map, slimmed);
      Api actual_slimmed;
      EXPECT_TRUE(actual_slimmed.ParseFromString(pbf_bytes));
      EXPECT_FALSE(actual_slimmed.has_options());
      EXPECT_TRUE(actual_slimmed.has_trip() || action == Options::status ||
                  action == Options::sources_to_targets || action == Options::isochrone);
      EXPECT_FALSE(actual_slimmed.has_directions());
      EXPECT_FALSE(actual_slimmed.has_status());
      EXPECT_TRUE(actual_slimmed.has_info() || action == Options::status);

      // lets try it one more time but this time we'll let it default to the right output
      slimmed.Clear();
      slimmed.mutable_options()->CopyFrom(clean_pbf.options());
      slimmed.mutable_options()->clear_costings();
      slimmed.mutable_options()->set_format(Options::pbf);
      pbf_bytes = gurka::do_action(map, slimmed);
      actual_slimmed.Clear();
      EXPECT_TRUE(actual_slimmed.ParseFromString(pbf_bytes));
      EXPECT_FALSE(actual_slimmed.has_options());
      EXPECT_TRUE(actual_slimmed.has_trip() || action != Options::trace_attributes ||
                  action != Options::isochrone);
      EXPECT_TRUE(actual_slimmed.has_directions() || action != Options::trace_route ||
                  action != Options::route);
      EXPECT_TRUE(actual_slimmed.has_status() || action != Options::status);
      EXPECT_TRUE(actual_slimmed.has_info() || action == Options::status);
      EXPECT_TRUE(actual_slimmed.has_matrix() || action != Options::sources_to_targets);
    }
  }
}

TEST(pbf_api, pbf_error) {
  // we set nothing so we should get an error about having not selecting an action
  Api api;
  gurka::map map{test::make_config("./foobar"), {}};

  for (int i = 0; i < 2; ++i) {
    try {
      api.mutable_options()->set_format(Options::pbf);
      gurka::do_action(map, api);
      throw std::runtime_error("We should not get to here");
    } catch (valhalla_exception_t& e) {
      auto pbf_bytes = serialize_error(e, api);
      Api actual;
      EXPECT_TRUE(actual.ParseFromString(pbf_bytes));
      EXPECT_FALSE(actual.has_options());
      EXPECT_FALSE(actual.has_trip());
      EXPECT_FALSE(actual.has_directions());
      EXPECT_FALSE(actual.has_status());
      EXPECT_FALSE(actual.has_matrix());
      EXPECT_EQ(actual.info().errors().size(), 1);
    }
    // try again with an action but no locations
    api.Clear();
    api.mutable_options()->set_action(Options::route);
  }
}
