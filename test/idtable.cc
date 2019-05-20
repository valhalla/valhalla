#include "test.h"

#include "mjolnir/idtable.h"
#include <cstdint>
#include <cstdlib>
#include <unordered_set>

using namespace std;
using namespace valhalla::mjolnir;

constexpr uint64_t kTableSize = 40000;

void TestSetGet() {

  IdTable t(kTableSize);

  // set them all and check them all
  for (uint64_t i = 0; i < kTableSize; ++i) {
    if (t.get(i))
      throw std::logic_error("Bit should not be set");
    t.set(i);
    if (!t.get(i))
      throw std::logic_error("Bit should be set");
  }
  for (uint64_t i = 0; i < kTableSize; ++i) {
    if (!t.get(i))
      throw std::logic_error("Bit should be set");
  }
}

void TestRandom() {

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
    if (exists != t.get(i))
      throw std::logic_error("Bit has wrong value");
  }
}

void TestBounds() {
  IdTable t(10);

  for (int i = 60; i < 70; ++i)
    if (t.get(i))
      throw std::logic_error("No bits can be set when they are higher than max");

  if (t.max() != 63)
    throw std::logic_error("Max id should be 10");

  for (int i = 60; i < 70; ++i)
    if (i % 2)
      t.set(i);

  for (int i = 60; i < 70; ++i)
    if (i % 2 != t.get(i))
      throw std::logic_error("The odd ids should be set");

  if (t.max() != 191)
    throw std::logic_error("The max id should have been increased to 181");
}

int main() {
  test::suite suite("nodetable");

  // Test setting and getting on random sizes of bit tables
  suite.test(TEST_CASE(TestSetGet));
  suite.test(TEST_CASE(TestRandom));
  suite.test(TEST_CASE(TestBounds));

  return suite.tear_down();
}
