#include "gurka.h"
#include "proto/options.pb.h"
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

// Test the hierarchy limits are actually disabled when disable_hierarchy_pruning = true
void doTest(Costing::Type costing_type) {
  // Set costing options
  Options options;
  set_disable_hierarchy_pruning(options, costing_type);
  create_costing_options(options, costing_type);
  auto travel_mode = valhalla::sif::TravelMode::kDrive;
  const auto mode_costing = valhalla::sif::CostFactory().CreateModeCosting(options, travel_mode);

  // Check hierarchy limits
  auto& hierarchy_limits =
      mode_costing[int(travel_mode)]->GetHierarchyLimits(); // access mutable limits
  for (auto& hierarchy : hierarchy_limits) {
    EXPECT_EQ(hierarchy.max_up_transitions, kUnlimitedTransitions);
  }
}

TEST(DisableHierarchyPruningTest, Auto) {
  Costing::Type type = Costing::auto_;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, Bike) {
  Costing::Type type = Costing::bicycle;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, Bus) {
  Costing::Type type = Costing::bus;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, MotorScooter) {
  Costing::Type type = Costing::motor_scooter;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, Pedestrian) {
  Costing::Type type = Costing::pedestrian;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, Truck) {
  Costing::Type type = Costing::truck;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, Motorcycle) {
  Costing::Type type = Costing::motorcycle;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, Taxi) {
  Costing::Type type = Costing::taxi;
  doTest(type);
}

TEST(DisableHierarchyPruningTest, None) {
  Costing::Type type = Costing::none_;
  doTest(type);
}
