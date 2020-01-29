#include "baldr/nodetransition.h"
#include "baldr/graphid.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 32 bytes.
constexpr size_t kNodeTransitionExpectedSize = 8;

namespace {

TEST(NodeTransition, Sizeof) {
  EXPECT_EQ(sizeof(NodeTransition), kNodeTransitionExpectedSize);
}

TEST(NodeTransition, WriteRead) {
  // Test building NodeTransition and reading back values
  GraphId id(1111, 2, 5555);
  NodeTransition nodetrans(id, true);

  EXPECT_EQ(nodetrans.endnode(), id);
  EXPECT_TRUE(nodetrans.up());
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
