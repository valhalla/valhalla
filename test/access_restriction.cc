#include "baldr/accessrestriction.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 16 bytes. We want to alert if somehow any change grows
// this structure size as that indicates incompatible tiles.
constexpr size_t kAccessRestrictionExpectedSize = 16;

namespace {

TEST(AccessRestrictions, SizeofCheck) {
  EXPECT_EQ(sizeof(AccessRestriction), kAccessRestrictionExpectedSize);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
