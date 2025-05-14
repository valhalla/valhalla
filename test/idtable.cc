#include "../src/mjolnir/idtable.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <unordered_set>

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
  uint64_t node_count = 1300000000; // this is about how many there are in real life
  node_count = 1000;

  UnorderedIdTable a(node_count);
  for (uint64_t i = 0; i < node_count; ++i) {
    a.set(i * 2);
    EXPECT_FALSE(a.get(i * 2 - 1));
    EXPECT_TRUE(a.get(i * 2));
  }
  a.serialize("foo.bar");
  UnorderedIdTable b(node_count);
  b.deserialize("foo.bar");
  EXPECT_EQ(a, b);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
