#include "baldr/rapidjson_utils.h"
#include "gurka/gurka.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/dynamiccost.h"
#include "test.h"
#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

#include <boost/format.hpp>

#include <string>
#include <vector>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

boost::property_tree::ptree
make_test_config(const std::unordered_map<std::string, std::string>& overrides = {}) {
  return test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles", overrides);
}

HierarchyLimits makeHierarchyLimits(const uint32_t max_up, const double exp) {
  HierarchyLimits hl;
  hl.set_max_up_transitions(max_up);
  hl.set_expand_within_dist(exp);
  return hl;
}

struct HierarchyLimitsTestParams {
  HierarchyLimitsTestParams(std::string&& req,
                            std::vector<HierarchyLimits>& hl,
                            bool use_pbf,
                            const std::unordered_map<std::string, std::string>& overrides = {})
      : request(req), cfg(make_test_config(std::move(overrides))), hierarchy_limits_config_path(""),
        expected_hierarchy_limits(hl), pbf(use_pbf){};
  HierarchyLimitsTestParams(const std::unordered_map<std::string, std::string>& overrides,
                            std::string& config_path)
      : cfg(make_test_config(std::move(overrides))), hierarchy_limits_config_path(config_path){};

  std::string request;
  boost::property_tree::ptree cfg;
  std::string hierarchy_limits_config_path;
  std::vector<HierarchyLimits> expected_hierarchy_limits;
  bool pbf;
};

std::string makeJsonRequest(std::vector<std::pair<uint32_t, std::optional<float>>>& hl_values) {

  std::unordered_map<std::string, std::string> opts;
  for (size_t i = 0; i < hl_values.size(); ++i) {
    auto& hl = hl_values[i];
    opts["/costing_options/auto/hierarchy_limits/" + std::to_string(i) + "/max_up_transitions"] =
        std::to_string(hl.first);

    if (!hl.second)
      continue;
    opts["/costing_options/auto/hierarchy_limits/" + std::to_string(i) + "/expand_within_distance"] =
        std::to_string(*hl.second);
  }
  std::vector<std::vector<PointLL>> locs = {{{5.101728, 52.106337}}, {{5.089717, 52.111276}}};
  return gurka::detail::build_valhalla_request({"sources", "targets"}, locs, "auto", opts, "");
}
std::string makePbfRequest(std::vector<std::pair<uint32_t, std::optional<float>>>& hl_values) {
  Api request;
  auto* opts = request.mutable_options();
  opts->set_action(Options_Action_sources_to_targets);
  auto src = opts->mutable_sources()->Add();
  src->mutable_ll()->set_lng(5.101728);
  src->mutable_ll()->set_lat(52.106337);

  auto tgt = opts->mutable_targets()->Add();
  tgt->mutable_ll()->set_lng(5.089717);
  tgt->mutable_ll()->set_lat(52.111276);
  opts->set_costing_type(Costing_Type_auto_);

  Costing costing;
  for (size_t i = 0; i < hl_values.size(); ++i) {
    HierarchyLimits hierarchylims;
    auto& hl = hl_values[i];
    hierarchylims.set_max_up_transitions(hl.first);
    if (hl.second)
      hierarchylims.set_expand_within_dist(*hl.second);
    costing.mutable_options()->mutable_hierarchy_limits()->insert(
        {static_cast<unsigned int>(i), hierarchylims});
  }
  opts->mutable_costings()->insert({valhalla::Costing::auto_, costing});
  return request.SerializeAsString();
}

HierarchyLimitsTestParams
makeParamsWithRequest(std::vector<std::pair<uint32_t, std::optional<float>>>&& hl_values, bool pbf) {
  std::vector<HierarchyLimits> hlimits;
  for (size_t i = 0; i < hl_values.size(); ++i) {
    HierarchyLimits hierarchylims;
    auto& hl = hl_values[i];
    hierarchylims.set_max_up_transitions(hl.first);
    if (hl.second)
      hierarchylims.set_expand_within_dist(*hl.second);
    hlimits.push_back(hierarchylims);
  }
  std::unordered_map<std::string, std::string> allow_modification_override = {
      {"service_limits.hierarchy_limits.allow_modification", "1"}};
  if (pbf)
    return {makePbfRequest(hl_values), hlimits, pbf, allow_modification_override};

  return {makeJsonRequest(hl_values), hlimits, pbf, allow_modification_override};
}
void hierarchy_limits_equal(const std::vector<HierarchyLimits>& expected,
                            const std::vector<HierarchyLimits>& actual) {
  EXPECT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    const auto& actual_hl = actual[i];
    const auto& expected_hl = expected[i];
    EXPECT_EQ(actual_hl.max_up_transitions(), expected_hl.max_up_transitions());
    EXPECT_EQ(actual_hl.expand_within_dist(), expected_hl.expand_within_dist());
  }
}

class TestHierarchyLimits : public ::testing::TestWithParam<HierarchyLimitsTestParams> {};

TEST_P(TestHierarchyLimits, from_request) {

  const auto& test_params = GetParam();
  if (!test_params.hierarchy_limits_config_path.empty())
    return;

  Api request;
  if (test_params.pbf) {
    request.ParseFromString(test_params.request);
  } else {
    ParseApi(test_params.request, Options::sources_to_targets, request);
  }
  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  mode_costing[0] = CreateAutoCost(costings);

  // Now make sure the costmatrix hierarchy limits match up with what we expect
  hierarchy_limits_equal(test_params.expected_hierarchy_limits,
                         mode_costing[0]->GetHierarchyLimits());
}

INSTANTIATE_TEST_SUITE_P(pbf_request,
                         TestHierarchyLimits,
                         ::testing::Values(makeParamsWithRequest({{std::make_pair(12, 12.)},
                                                                  {std::make_pair(23, 23.)},
                                                                  {std::make_pair(24, 24.f)}},
                                                                 true),
                                           makeParamsWithRequest({{std::make_pair(12, 12.)},
                                                                  {std::make_pair(23, 23.)},
                                                                  {std::make_pair(24, 24.f)}},
                                                                 false),
                                           makeParamsWithRequest({{std::make_pair(7, 35.)},
                                                                  {std::make_pair(11, 39.)},
                                                                  {std::make_pair(20, 45.f)}},
                                                                 true),
                                           makeParamsWithRequest({{std::make_pair(7, 35.)},
                                                                  {std::make_pair(11, 39.)},
                                                                  {std::make_pair(20, 45.f)}},
                                                                 false),
                                           makeParamsWithRequest({{std::make_pair(1, 1.)},
                                                                  {std::make_pair(2, 2.)},
                                                                  {std::make_pair(3, 3.)}},
                                                                 true)));

TEST(StandAlone, ClampHierarchyLimitsMatrix) {
  Api request;
  auto test_params = makeParamsWithRequest({{std::make_pair(1200, 1200000.)},
                                            {std::make_pair(23000, 23000.)},
                                            {std::make_pair(240000, 240000.f)}},
                                           false);
  if (test_params.pbf) {
    request.ParseFromString(test_params.request);
  } else {
    ParseApi(test_params.request, Options::sources_to_targets, request);
  }

  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  mode_costing[0] = CreateAutoCost(costings);

  Costing_Options opts;
  hierarchy_limits_config_t config_matrix =
      parse_hierarchy_limits_from_config(test_params.cfg, "costmatrix", false);

  EXPECT_TRUE(check_hierarchy_limits(mode_costing[0]->GetHierarchyLimits(), mode_costing[0], opts,
                                     config_matrix, true, true));

  EXPECT_FALSE(mode_costing[0]->DefaultHierarchyLimits());

  std::vector<HierarchyLimits> expected_hl = {makeHierarchyLimits(0, 0), makeHierarchyLimits(400, 0),
                                              makeHierarchyLimits(100, 0)};
  hierarchy_limits_equal(expected_hl, mode_costing[0]->GetHierarchyLimits());
}

TEST(StandAlone, ClampHierarchyLimitsBidirAStar) {
  Api request;
  auto test_params = makeParamsWithRequest({{std::make_pair(0, kMaxDistance)},
                                            {std::make_pair(23000, 23000.)},
                                            {std::make_pair(240000, 240000.f)}},
                                           false);
  if (test_params.pbf) {
    request.ParseFromString(test_params.request);
  } else {
    ParseApi(test_params.request, Options::sources_to_targets, request);
  }

  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  mode_costing[0] = CreateAutoCost(costings);

  Costing_Options opts;
  hierarchy_limits_config_t config_bidir =
      parse_hierarchy_limits_from_config(test_params.cfg, "bidirectional_astar", true);

  EXPECT_TRUE(check_hierarchy_limits(mode_costing[0]->GetHierarchyLimits(), mode_costing[0], opts,
                                     config_bidir, true, true));

  EXPECT_FALSE(mode_costing[0]->DefaultHierarchyLimits());

  std::vector<HierarchyLimits> expected_hl = {makeHierarchyLimits(0, 1e8),
                                              makeHierarchyLimits(400, 20000),
                                              makeHierarchyLimits(100, 5000)};
  hierarchy_limits_equal(expected_hl, mode_costing[0]->GetHierarchyLimits());
}

TEST(StandAlone, ClampHierarchyLimitsUnidirAStar) {
  Api request;
  auto test_params = makeParamsWithRequest({{std::make_pair(0, kMaxDistance)},
                                            {std::make_pair(23000, 230000.)},
                                            {std::make_pair(240000, 240000.f)}},
                                           false);
  if (test_params.pbf) {
    request.ParseFromString(test_params.request);
  } else {
    ParseApi(test_params.request, Options::sources_to_targets, request);
  }

  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  mode_costing[0] = CreateAutoCost(costings);

  Costing_Options opts;
  hierarchy_limits_config_t config_bidir =
      parse_hierarchy_limits_from_config(test_params.cfg, "unidirectional_astar", true);

  EXPECT_TRUE(check_hierarchy_limits(mode_costing[0]->GetHierarchyLimits(), mode_costing[0], opts,
                                     config_bidir, true, true));

  EXPECT_FALSE(mode_costing[0]->DefaultHierarchyLimits());

  std::vector<HierarchyLimits> expected_hl = {makeHierarchyLimits(0, 1e8),
                                              makeHierarchyLimits(400, 100000),
                                              makeHierarchyLimits(100, 5000)};
  hierarchy_limits_equal(expected_hl, mode_costing[0]->GetHierarchyLimits());
}

TEST(StandAlone, Warnings) {
  // test whether warnings are only added when
  //  a) modification is not allowed by the server and user tried to pass custom limits
  //  b) modification is allowed and user passed invalid limits

  const std::string ascii_map = R"(
      A---B---C---D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
  };

  constexpr double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});
  auto map_no_mod =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/hierarchylimits_no_mod");

  // single leg route, no customization
  auto result = gurka::do_action(valhalla::Options::route, map_no_mod, {"A", "D"}, "auto");
  EXPECT_EQ(result.info().warnings().size(), 0);

  // single leg route, disallowed customization
  std::string req = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"hierarchy_limits":{"1":{"max_up_transitions": 1000}}}}
    })";

  std::string from = "A";
  std::string via = "B";
  std::string to = "C";
  req =
      (boost::format(req) % std::to_string(map_no_mod.nodes.at(from).lat()) %
       std::to_string(map_no_mod.nodes.at(from).lng()) %
       std::to_string(map_no_mod.nodes.at(to).lat()) % std::to_string(map_no_mod.nodes.at(to).lng()))
          .str();
  result = gurka::do_action(valhalla::Options::route, map_no_mod, req);
  EXPECT_EQ(result.info().warnings().size(), 1);

  // double leg route, disallowed customization
  req = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"hierarchy_limits":{"1":{"max_up_transitions": 1000}}}}
    })";

  req =
      (boost::format(req) % std::to_string(map_no_mod.nodes.at(from).lat()) %
       std::to_string(map_no_mod.nodes.at(from).lng()) %
       std::to_string(map_no_mod.nodes.at(via).lat()) %
       std::to_string(map_no_mod.nodes.at(via).lng()) %
       std::to_string(map_no_mod.nodes.at(to).lat()) % std::to_string(map_no_mod.nodes.at(to).lng()))
          .str();
  result = gurka::do_action(valhalla::Options::route, map_no_mod, req);
  EXPECT_EQ(result.info().warnings().size(), 1);
  EXPECT_EQ(result.info().warnings(0).code(), 209);

  // double leg route, no customization
  result = gurka::do_action(valhalla::Options::route, map_no_mod, {"A", "B", "D"}, "auto");
  EXPECT_EQ(result.info().warnings().size(), 0);

  // single leg route, customization allowed but nothing passed
  auto map_mod =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/hierarchylimits_mod",
                        {{"service_limits.hierarchy_limits.allow_modification", "1"}});
  result = gurka::do_action(valhalla::Options::route, map_mod, {"A", "B"}, "auto");
  EXPECT_EQ(result.info().warnings().size(), 0);

  // double leg route, customization allowed but nothing passed
  result = gurka::do_action(valhalla::Options::route, map_mod, {"A", "B", "D"}, "auto");
  EXPECT_EQ(result.info().warnings().size(), 0);

  // single leg route, clamped customization
  req = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"hierarchy_limits":{"1":{"max_up_transitions": 100000}}}}
    })";

  req = (boost::format(req) % std::to_string(map_mod.nodes.at(from).lat()) %
         std::to_string(map_mod.nodes.at(from).lng()) % std::to_string(map_mod.nodes.at(to).lat()) %
         std::to_string(map_mod.nodes.at(to).lng()))
            .str();
  result = gurka::do_action(valhalla::Options::route, map_mod, req);
  EXPECT_EQ(result.info().warnings().size(), 1);
  EXPECT_EQ(result.info().warnings(0).code(), 210);

  // double leg route, clamped customization
  req = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"hierarchy_limits":{"1":{"max_up_transitions": 1000}}}}
    })";

  req = (boost::format(req) % std::to_string(map_mod.nodes.at(from).lat()) %
         std::to_string(map_mod.nodes.at(from).lng()) % std::to_string(map_mod.nodes.at(via).lat()) %
         std::to_string(map_mod.nodes.at(via).lng()) % std::to_string(map_mod.nodes.at(to).lat()) %
         std::to_string(map_mod.nodes.at(to).lng()))
            .str();
  result = gurka::do_action(valhalla::Options::route, map_mod, req);
  EXPECT_EQ(result.info().warnings().size(), 1);
  EXPECT_EQ(result.info().warnings(0).code(), 210);

  // single leg route, allowed customization
  req = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"hierarchy_limits":{"1":{"max_up_transitions": 10, "expand_within_distance": 10}}}}
    })";

  req = (boost::format(req) % std::to_string(map_mod.nodes.at(from).lat()) %
         std::to_string(map_mod.nodes.at(from).lng()) % std::to_string(map_mod.nodes.at(to).lat()) %
         std::to_string(map_mod.nodes.at(to).lng()))
            .str();
  result = gurka::do_action(valhalla::Options::route, map_mod, req);
  EXPECT_EQ(result.info().warnings().size(), 0);

  // double leg route, allowed customization
  req = R"({
      "locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],
      "costing": "auto",
      "costing_options":{"auto":{"hierarchy_limits":{"1":{"max_up_transitions": 10, "expand_within_distance": 10}}}}
    })";

  req = (boost::format(req) % std::to_string(map_mod.nodes.at(from).lat()) %
         std::to_string(map_mod.nodes.at(from).lng()) % std::to_string(map_mod.nodes.at(via).lat()) %
         std::to_string(map_mod.nodes.at(via).lng()) % std::to_string(map_mod.nodes.at(to).lat()) %
         std::to_string(map_mod.nodes.at(to).lng()))
            .str();
  result = gurka::do_action(valhalla::Options::route, map_mod, req);
  EXPECT_EQ(result.info().warnings().size(), 0);
}
int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}