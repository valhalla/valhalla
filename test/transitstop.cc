#include "baldr/transitstop.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 8 bytes
constexpr size_t kTransitStopExpectedSize = 8;

namespace {

TEST(TransitStop, Sizeof) {
  EXPECT_EQ(sizeof(TransitStop), kTransitStopExpectedSize);
}

TEST(TransitStop, TestWriteRead) {
  // Test building a transit stop and reading back values
  uint32_t trav = static_cast<uint32_t>(Traversability::kForward);
  TransitStop transit_stop(555, 777, false, trav);

  EXPECT_EQ(transit_stop.one_stop_offset(), 555);
  EXPECT_EQ(transit_stop.name_offset(), 777);
  EXPECT_EQ(transit_stop.generated(), false);
  EXPECT_EQ(transit_stop.traversability(), Traversability::kForward);

  // Test bounds of one stop offset
  EXPECT_THROW(TransitStop transit_stop(kMaxNameOffset + 1, 777, false, trav);, std::runtime_error)
      << "TransitStop one_stop_offset limit check failed";

  // Test bounds of name offset
  EXPECT_THROW(TransitStop transit_stop(123, kMaxNameOffset + 1, false, trav);, std::runtime_error)
      << "TransitStop name offset limit check failed";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
