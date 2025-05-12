#include "baldr/nodeinfo.h"
#include "midgard/util.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

// Expected size is 32 bytes.
constexpr size_t kNodeInfoExpectedSize = 32;

namespace {

TEST(NodeInfo, Sizeof) {
  EXPECT_EQ(sizeof(NodeInfo), kNodeInfoExpectedSize);
}

TEST(NodeInfo, LL) {
  float kEpsilon = .00001;

  PointLL base_ll(-70.0f, 40.0f);
  NodeInfo n;
  PointLL node_ll = n.latlng(base_ll);
  EXPECT_NEAR(node_ll.lng(), -70.0f, kEpsilon);
  EXPECT_NEAR(node_ll.lat(), 40.0f, kEpsilon);

  NodeInfo t;
  PointLL nodell0(-69.5f, 40.25f);
  t.set_latlng(base_ll, nodell0);
  node_ll = t.latlng(base_ll);
  EXPECT_NEAR(node_ll.lng(), nodell0.lng(), kEpsilon);
  EXPECT_NEAR(node_ll.lat(), nodell0.lat(), kEpsilon);

  // Test lon just outside tile bounds
  PointLL nodell1(-70.000005f, 40.25f);
  t.set_latlng(base_ll, nodell1);
  node_ll = t.latlng(base_ll);
  // NodeInfo ll should be -70.0, 40.25
  EXPECT_NEAR(node_ll.lng(), base_ll.lng(), kEpsilon);
  EXPECT_NEAR(node_ll.lat(), nodell1.lat(), kEpsilon);

  // Test lat just outside tile bounds
  PointLL nodell2(-69.5f, 39.999995f);
  t.set_latlng(base_ll, nodell2);
  node_ll = t.latlng(base_ll);
  // NodeInfo ll should be -69.5, 40.0
  EXPECT_NEAR(node_ll.lng(), nodell2.lng(), kEpsilon);
  EXPECT_NEAR(node_ll.lat(), base_ll.lat(), kEpsilon);
}

// Write to file and read into NodeInfo
TEST(NodeInfo, WriteRead) {
  // Test building NodeInfo and reading back values
  NodeInfo nodeinfo;

  // Headings are reduced to 8 bits
  nodeinfo.set_heading(0, 266);
  nodeinfo.set_heading(1, 90);
  nodeinfo.set_heading(2, 32);
  nodeinfo.set_heading(3, 180);
  nodeinfo.set_heading(4, 185);
  nodeinfo.set_heading(5, 270);
  nodeinfo.set_heading(6, 145);
  nodeinfo.set_heading(7, 0);

  EXPECT_EQ(nodeinfo.heading(0), 266);
  EXPECT_EQ(nodeinfo.heading(1), 90);
  EXPECT_EQ(nodeinfo.heading(2), 32);
  EXPECT_EQ(nodeinfo.heading(3), 180);
  EXPECT_EQ(nodeinfo.heading(4), 184);
  EXPECT_EQ(nodeinfo.heading(5), 270);
  EXPECT_EQ(nodeinfo.heading(6), 145);
  EXPECT_EQ(nodeinfo.heading(7), 0);

  nodeinfo.set_local_driveability(3, Traversability::kBoth);
  nodeinfo.set_local_driveability(5, Traversability::kNone);
  nodeinfo.set_local_driveability(7, Traversability::kForward);
  nodeinfo.set_local_driveability(1, Traversability::kBackward);

  EXPECT_EQ(nodeinfo.local_driveability(3), Traversability::kBoth);
  EXPECT_EQ(nodeinfo.local_driveability(5), Traversability::kNone);
  EXPECT_EQ(nodeinfo.local_driveability(7), Traversability::kForward);
  EXPECT_EQ(nodeinfo.local_driveability(1), Traversability::kBackward);
}

// Test elevation
TEST(NodeInfo, Elevation) {
  // Test elevation at 0
  NodeInfo node;
  node.set_elevation(0);
  EXPECT_LE(std::abs(node.elevation()), 0.25f);

  // Elevation < -500 is set to -500
  node.set_elevation(-700.0f);
  EXPECT_LE(std::abs((node.elevation() - -500.0f)), 0.25f);

  node.set_elevation(700.0f);
  EXPECT_LE(std::abs((node.elevation() - 700.0f)), 0.25f);

  node.set_elevation(1426.511963f);
  EXPECT_LE(std::abs((node.elevation() - 1426.511963f)), 0.25f);

  // Highest road elevation is ~5600m, test at 6000m to be safe
  node.set_elevation(6000.0f);
  EXPECT_LE(std::abs((node.elevation() - 6000.0f)), 0.25f);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
