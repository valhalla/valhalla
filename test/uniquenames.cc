#include "test.h"
#include <cstdint>

#include "mjolnir/uniquenames.h"

using namespace std;
using namespace valhalla::mjolnir;

void TestSize() {
  UniqueNames names;
  names.index("Main Street");
  names.index("I-95");
  names.index("MD-32");
  names.index("I-95 S");
  if (names.Size() != 4)
    throw runtime_error("UniqueNames Size test failed");

  // Add a duplicate name and a unique name. Make sure Size
  // reflects only one name added
  names.index("MD-32");
  names.index("First Avenue");
  if (names.Size() != 5)
    throw runtime_error("UniqueNames Size test failed");
}

void TestAddAndIndex() {
  UniqueNames names;
  uint32_t index1 = names.index("I-95");
  uint32_t index2 = names.index("I-95 S");
  uint32_t index3 = names.index("I-95 N");
  uint32_t index4 = names.index("Interstate 95");
  uint32_t index5 = names.index("I-95");
  uint32_t index6 = names.index("I-95 N");

  if (index3 != index6)
    throw runtime_error("UniqueNames: indexes for common name are not equal");
  if (index1 != index5)
    throw runtime_error("UniqueNames: indexes for common name are not equal");

  // Test getting the name given the index
  if (names.name(index1) != "I-95")
    throw runtime_error("UniqueNames: name given an index failed");
  if (names.name(index2) != "I-95 S")
    throw runtime_error("UniqueNames: name given an index failed");
  if (names.name(index3) != "I-95 N")
    throw runtime_error("UniqueNames: name given an index failed");
  if (names.name(index4) != "Interstate 95")
    throw runtime_error("UniqueNames: name given an index failed");
  if (names.name(index5) != "I-95")
    throw runtime_error("UniqueNames: name given an index failed");
  if (names.name(index6) != "I-95 N")
    throw runtime_error("UniqueNames: name given an index failed");
}

int main() {
  test::suite suite("uniquenames");

  // Test adding names and getting them via index
  suite.test(TEST_CASE(TestAddAndIndex));

  // Test Size
  suite.test(TEST_CASE(TestSize));

  return suite.tear_down();
}
