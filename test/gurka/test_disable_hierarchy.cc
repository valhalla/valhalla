#include "gurka.h"
#include "proto/options.pb.h"
#include "test.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

// utils
// Create costing options (reference: /test/astar.cc)
void create_costing_options(Options& options, Costing::Type type) {
  const rapidjson::Document doc;
  sif::ParseCosting(doc, "/costing_options", options);
  options.set_costing_type(type);
}

// Set disable_hierarchy_pruning to true in costing options
void set_disable_hierarchy_pruning(Options& options, Costing::Type type) {
  Costing* costing = &(*options.mutable_costings())[type];
  auto* co = costing->mutable_options();
  co->set_disable_hierarchy_pruning(true);
}

class HierarchyTest : public ::testing::TestWithParam<Costing::Type> {
protected:
  // Test the hierarchy limits are actually disabled when disable_hierarchy_pruning = true
  void doTest(Costing::Type costing_type) {
    // Set costing options
    Options options;
    set_disable_hierarchy_pruning(options, costing_type);
    create_costing_options(options, costing_type);
    valhalla::sif::TravelMode travel_mode;
    const auto mode_costing = valhalla::sif::CostFactory().CreateModeCosting(options, travel_mode);

    // Check hierarchy limits
    auto& hierarchy_limits = mode_costing[int(travel_mode)]->GetHierarchyLimits();
    for (auto& hierarchy : hierarchy_limits) {
      EXPECT_EQ(hierarchy.max_up_transitions, kUnlimitedTransitions);
    }
  }
};

TEST_P(HierarchyTest, TestDisableHierarchy) {
  doTest(GetParam());
}

INSTANTIATE_TEST_SUITE_P(disable_hierarchy_pruning,
                         HierarchyTest,
                         ::testing::Values(Costing::auto_,
                                           Costing::bus,
                                           Costing::motor_scooter,
                                           Costing::truck,
                                           Costing::motorcycle,
                                           Costing::taxi));