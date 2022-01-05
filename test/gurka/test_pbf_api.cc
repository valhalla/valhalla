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

  std::unordered_set<Options::Action> pbf_actions{
      Options::route,    Options::trace_route,      Options::optimized_route,
      Options::centroid, Options::trace_attributes, Options::status,
  };

  for (int action = Options::Action_MIN; action <= Options::Action_MAX; ++action) {
    // don't have convenient support of these in gurka yet
    if (action == Options::sources_to_targets || action == Options::isochrone ||
        action == Options::expansion || action == Options::chinese_postman)
      continue;

    // do the regular request with json in and out
    std::string expected_json, request_json;
    auto expected_pbf =
        gurka::do_action(Options::Action(action), map, {"A", "C", "I", "G"}, "pedestrian", {}, {},
                         &expected_json, "break", &request_json);

    // get some clean input options for pbf requests
    Api clean_pbf;
    ParseApi(request_json, Options::Action(action), clean_pbf);

    // do the request still with json out but with pbf in
    Api json_out;
    json_out.mutable_options()->CopyFrom(clean_pbf.options());
    json_out.mutable_options()->clear_costing_options();
    auto actual_json = gurka::do_action(map, json_out);
    EXPECT_EQ(actual_json, expected_json);

    // auto write_to_file = [](const std::string& path, const std::string& buf) {
    //  std::ofstream f(path);
    //  f.write(buf.data(), buf.size());
    // };

    // if this action support pbf out try that
    if (pbf_actions.count(Options::Action(action))) {
      Api pbf_out;
      pbf_out.mutable_options()->CopyFrom(clean_pbf.options());
      pbf_out.mutable_options()->clear_costing_options();
      pbf_out.mutable_options()->set_format(Options::pbf);
      auto pbf_bytes = gurka::do_action(map, pbf_out);
      Api actual_pbf;
      EXPECT_TRUE(actual_pbf.ParseFromString(pbf_bytes));
      // write_to_file("actual.pbf", actual_pbf.SerializeAsString());
      // write_to_file("expected.pbf", expected_pbf.SerializeAsString());
      EXPECT_EQ(actual_pbf.trip().SerializeAsString(), expected_pbf.trip().SerializeAsString());
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
      EXPECT_EQ(actual.info().errors().size(), 1);
    }
    // try again with an action but no locations
    api.Clear();
    api.mutable_options()->set_action(Options::route);
  }
}
