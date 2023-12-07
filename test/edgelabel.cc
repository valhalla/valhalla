
#include "sif/edgelabel.h"

#include "test.h"

using namespace std;
using namespace valhalla::sif;

// Expected size is 40 bytes.
constexpr size_t kEdgeLabelExpectedSize = 40;
constexpr size_t kPathEdgeLabelExpectedSize = 48;
constexpr size_t kBDEdgeLabelExpectedSize = 64;
constexpr size_t kMMEdgeLabelExpectedSize = 72;

namespace {

TEST(EdgeLabel, test_sizeof) {
  EXPECT_EQ(sizeof(EdgeLabel), kEdgeLabelExpectedSize);
  EXPECT_EQ(sizeof(PathEdgeLabel), kPathEdgeLabelExpectedSize);
  EXPECT_EQ(sizeof(BDEdgeLabel), kBDEdgeLabelExpectedSize);
  EXPECT_EQ(sizeof(MMEdgeLabel), kMMEdgeLabelExpectedSize);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
