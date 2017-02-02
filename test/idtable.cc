#include "test.h"

#include <cstdint>
#include <unordered_set>
#include <cstdlib>
#include "mjolnir/idtable.h"

using namespace std;
using namespace valhalla::mjolnir;

constexpr uint64_t kTableSize = 40000;

void TestSetGet() {

  IdTable t(kTableSize);

  //set them all and check them all
  for(uint64_t i = 0; i < kTableSize; ++i) {
    if(t.IsUsed(i))
      throw std::runtime_error("Bit should not be set");
    t.set(i);
    if(!t.IsUsed(i))
      throw std::runtime_error("Bit should be set");
  }
  for(uint64_t i = 0; i < kTableSize; ++i) {
    if(!t.IsUsed(i))
      throw std::runtime_error("Bit should be set");
  }
}

void TestRandom() {

  //randomly set and then go get some
  IdTable t(kTableSize);
  std::unordered_set<uint64_t> ids;
  for(uint64_t i = 0; i < kTableSize; ++i) {
    uint64_t r = rand() % kTableSize;
    if(rand() % 2)
    {
      ids.emplace(r);
      t.set(r);
    }
  }
  for(uint64_t i = 0; i < kTableSize; ++i) {
    bool exists = ids.find(i) != ids.end();
    if(exists != t.IsUsed(i))
      throw std::runtime_error("Bit has wrong value");
  }


}

int main() {
  test::suite suite("nodetable");

  // Test setting and getting on random sizes of bit tables
  suite.test(TEST_CASE(TestSetGet));
  suite.test(TEST_CASE(TestRandom));

  return suite.tear_down();
}
