#include <algorithm>
#include <cstdint>
#include <vector>

#include "baldr/accessrestriction.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 16 bytes. We want to alert if somehow any change grows
// this structure size as that indicates incompatible tiles.
constexpr size_t kAccessRestrictionExpectedSize = 16;

namespace {

void test_sizeof() {
  if (sizeof(AccessRestriction) != kAccessRestrictionExpectedSize)
    throw std::runtime_error("AccessRestriction size should be " +
                             std::to_string(kAccessRestrictionExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(AccessRestriction)));
}

} // namespace

int main() {
  test::suite suite("access_restriction");

  // Test sizeof the structure
  suite.test(TEST_CASE(test_sizeof));

  return suite.tear_down();
}
