#include "baldr/admin.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 16 bytes. We want to alert if somehow any change grows
// this structure size as that indicates incompatible tiles.
constexpr size_t kAdminExpectedSize = 16;

namespace {

TEST(Admin, size) {
  EXPECT_EQ(sizeof(Admin), kAdminExpectedSize);
}

TEST(Admin, Create) {
  Admin ai(5, 6, "US", "PA");
  EXPECT_EQ(ai.country_offset(), 5);
  EXPECT_EQ(ai.state_offset(), 6);
  EXPECT_EQ(ai.country_iso(), "US");
  EXPECT_EQ(ai.state_iso(), "PA");
}

TEST(Admin, Create3CharStateIso) {
  Admin aiStateISO(5, 6, "GB", "WLS");
  EXPECT_EQ(aiStateISO.state_iso(), "WLS");
}

TEST(Admin, EmptyStrings) {
  Admin aiEmptyStrings(5, 6, "", "");
  EXPECT_EQ(aiEmptyStrings.country_iso(), "");
  EXPECT_EQ(aiEmptyStrings.state_iso(), "");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
