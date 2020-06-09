#include "mjolnir/idtable.h"
#include <cstdint>
#include <cstdlib>
#include <unordered_set>

#include "test.h"

namespace {

using namespace std;
using namespace valhalla::mjolnir;

constexpr uint64_t kTableSize = 40000;

TEST(UnorderedIdTable, SetGet) {
  UnorderedIdTable t(kTableSize);

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

TEST(UnorderedIdTable, Random) {
  // randomly set and then go get some
  UnorderedIdTable t(kTableSize);
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

TEST(UnorderedIdTable, SerializeDeserialize) {
  // TODO
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
