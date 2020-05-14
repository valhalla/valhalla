#include "mjolnir/idtable.h"
#include <cstdint>
#include <cstdlib>
#include <unordered_set>

#include "test.h"

namespace {

using namespace std;
using namespace valhalla::mjolnir;

constexpr uint64_t kTableSize = 40000;

TEST(IdTable, SetGet) {
  IdTable t(kTableSize);

  // set them all and check them all
  for (uint64_t i = 0; i < kTableSize; ++i) {
    EXPECT_FALSE(t.get(i));
    t.set(i);
    EXPECT_TRUE(t.get(i));
  }

  for (uint64_t i = 0; i < kTableSize; ++i) {
    EXPECT_TRUE(t.get(i));
  }
}

TEST(IdTable, Random) {
  // randomly set and then go get some
  IdTable t(kTableSize);
  std::unordered_set<uint64_t> ids;
  for (uint64_t i = 0; i < kTableSize; ++i) {
    uint64_t r = rand() % kTableSize;
    if (rand() % 2) {
      ids.emplace(r);
      t.set(r);
    }
  }

  for (uint64_t i = 0; i < kTableSize; ++i) {
    bool exists = ids.find(i) != ids.end();
    EXPECT_EQ(exists, t.get(i));
  }
}

TEST(IdTable, Bounds) {
  IdTable t(10);

  for (int i = 60; i < 70; ++i) {
    EXPECT_FALSE(t.get(i)) << "No bits can be set when they are higher than max";
  }

  EXPECT_EQ(t.max(), 63) << "Max id should be 10";

  for (int i = 60; i < 70; ++i) {
    if (i % 2)
      t.set(i);
  }

  for (int i = 60; i < 70; ++i) {
    EXPECT_EQ(t.get(i), i % 2) << "The odd ids should be set";
  }

  EXPECT_EQ(t.max(), 191) << "The max id should have been increased to 191";
}

TEST(IdTable, X86) {
  IdTable t(10000000000);
  uint64_t old_max = t.max();

  t.set(5528037441);
  EXPECT_EQ(t.max(), old_max) << "The max id shouldn't be changed";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
