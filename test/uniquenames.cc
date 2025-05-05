#include "mjolnir/uniquenames.h"
#include "test.h"

#include <cstdint>

using namespace std;
using namespace valhalla::mjolnir;

namespace {

TEST(UniqueNames, Size) {
  UniqueNames names;
  names.index("Main Street");
  names.index("I-95");
  names.index("MD-32");
  names.index("I-95 S");
  EXPECT_EQ(names.Size(), 4);

  // Add a duplicate name and a unique name. Make sure Size
  // reflects only one name added
  names.index("MD-32");
  names.index("First Avenue");
  EXPECT_EQ(names.Size(), 5);
}

TEST(UniqueNames, TestAddAndIndex) {
  UniqueNames names;
  uint32_t index1 = names.index("I-95");
  uint32_t index2 = names.index("I-95 S");
  uint32_t index3 = names.index("I-95 N");
  uint32_t index4 = names.index("Interstate 95");
  uint32_t index5 = names.index("I-95");
  uint32_t index6 = names.index("I-95 N");

  EXPECT_EQ(index3, index6) << "UniqueNames: indexes for common name are not equal";
  EXPECT_EQ(index1, index5) << "UniqueNames: indexes for common name are not equal";

  // Test getting the name given the index
  EXPECT_EQ(names.name(index1), "I-95");
  EXPECT_EQ(names.name(index2), "I-95 S");
  EXPECT_EQ(names.name(index3), "I-95 N");
  EXPECT_EQ(names.name(index4), "Interstate 95");
  EXPECT_EQ(names.name(index5), "I-95");
  EXPECT_EQ(names.name(index6), "I-95 N");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
