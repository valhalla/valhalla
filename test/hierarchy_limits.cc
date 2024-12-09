#include "gurka/gurka.h"
#include "test.h"

#include <string>
#include <vector>

#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/dynamiccost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

boost::property_tree::ptree
make_test_config(std::unordered_map<std::string, std::string> overrides = {}) {
  return test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles", overrides);
}

struct HierarchyLimitsTestParams {
  HierarchyLimitsTestParams(std::string&& req,
                            std::unordered_map<uint32_t, HierarchyLimits>& hl,
                            bool use_pbf,
                            std::unordered_map<std::string, std::string> overrides = {})
      : request(req), expected_hierarchy_limits(hl), hierarchy_limits_config_path(""), pbf(use_pbf) {
    cfg = make_test_config(overrides);
  };
  HierarchyLimitsTestParams(std::unordered_map<std::string, std::string> overrides,
                            std::string& config_path)
      : cfg(make_test_config(overrides)), hierarchy_limits_config_path(config_path) {};

  std::string request;
  boost::property_tree::ptree cfg;
  std::string hierarchy_limits_config_path;
  std::unordered_map<uint32_t, HierarchyLimits> expected_hierarchy_limits;
  bool pbf;
};

std::string
makeJsonRequest(std::unordered_map<uint32_t, std::pair<uint32_t, std::optional<float>>>& hl_values) {

  std::unordered_map<std::string, std::string> opts;
  for (auto [level, hl] : hl_values) {
    opts["/costing_options/auto/hierarchy_limits/" + std::to_string(level) + "/max_up_transitions"] =
        std::to_string(hl.first);

    if (!hl.second)
      continue;
    opts["/costing_options/auto/hierarchy_limits/" + std::to_string(level) +
         "/expand_within_distance"] = std::to_string(*hl.second);
  }
  std::vector<std::vector<PointLL>> locs = {{{5.101728, 52.106337}, {5.089717, 52.111276}}};
  return gurka::detail::build_valhalla_request({""}, locs, "auto", opts, "");
}
std::string
makePbfRequest(std::unordered_map<uint32_t, std::pair<uint32_t, std::optional<float>>>& hl_values) {
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
  for (auto [level, hl] : hl_values) {
    HierarchyLimits hierarchylims;
    hierarchylims.set_max_up_transitions(hl.first);
    if (hl.second)
      hierarchylims.set_expansion_within_dist(*hl.second);
    costing.mutable_options()->mutable_hierarchy_limits()->insert({level, hierarchylims});
  }
  opts->mutable_costings()->insert({valhalla::Costing::auto_, costing});
  return request.SerializeAsString();
}

HierarchyLimitsTestParams makeParamsWithRequest(
    std::unordered_map<uint32_t, std::pair<uint32_t, std::optional<float>>>&& hl_values,
    bool pbf) {
  std::unordered_map<uint32_t, HierarchyLimits> hlimits;
  for (auto [level, hl] : hl_values) {
    HierarchyLimits hierarchylims;
    hierarchylims.set_max_up_transitions(hl.first);
    if (hl.second)
      hierarchylims.set_expansion_within_dist(*hl.second);
    hlimits.insert({level, hierarchylims});
  }
  if (pbf)
    return {makePbfRequest(hl_values),
            hlimits,
            pbf,
            {{"service_limits.hierarchy_limits.allow_modification", "1"}}};

  // todo
  return {"", hlimits, pbf};
}

class TestHierarchyLimits : public ::testing::TestWithParam<HierarchyLimitsTestParams> {
protected:
  void hierarchy_limits_equal(std::unordered_map<uint32_t, HierarchyLimits>& expected,
                              std::unordered_map<uint32_t, HierarchyLimits> actual) {
    EXPECT_EQ(expected.size(), actual.size());
    for (auto [level, hl] : actual) {
      auto res = expected.find(level);
      EXPECT_TRUE(res != expected.end());
      auto expected_hl = res->second;
      EXPECT_EQ(hl.max_up_transitions(), expected_hl.max_up_transitions());
      EXPECT_EQ(hl.expansion_within_dist(), expected_hl.expansion_within_dist());
    }
  }
};

TEST_P(TestHierarchyLimits, from_request) {

  auto test_params = GetParam();
  if (!test_params.hierarchy_limits_config_path.empty())
    return;

  Api request;
  if (test_params.pbf) {
    request.ParseFromString(test_params.request);
  } else {
    ParseApi(test_params.request, Options::sources_to_targets, request);
  }
  loki_worker_t loki_worker(test_params.cfg);

  // here we parse it and make sure it's in accordance with the service limits
  loki_worker.matrix(request);

  GraphReader reader(test_params.cfg.get_child("mjolnir"));

  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  mode_costing[0] = CreateAutoCost(costings);

  EXPECT_FALSE(mode_costing[0]->DefaultHierarchyLimits());

  // Now make sure the costmatrix hierarchy limits match up with what we expect
  hierarchy_limits_equal(test_params.expected_hierarchy_limits,
                         mode_costing[0]->GetHierarchyLimits());
}

INSTANTIATE_TEST_SUITE_P(
    pbf_request,
    TestHierarchyLimits,
    ::testing::Values(
        makeParamsWithRequest({{1, std::make_pair(23, 23.)}, {2, std::make_pair(24, 24.f)}}, true),
        makeParamsWithRequest({{1, std::make_pair(0, 0.)}, {2, std::make_pair(0, 0.f)}}, true),
        makeParamsWithRequest({{1, std::make_pair(1, 2.)}, {2, std::make_pair(1, 2.)}}, true)));

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}