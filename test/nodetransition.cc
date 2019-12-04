#include "test.h"

#include "baldr/graphid.h"
#include "baldr/nodetransition.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 32 bytes.
constexpr size_t kNodeTransitionExpectedSize = 8;

namespace {

void test_sizeof() {
  if (sizeof(NodeTransition) != kNodeTransitionExpectedSize)
    throw std::runtime_error("NodeTransition size should be " +
                             std::to_string(kNodeTransitionExpectedSize) + " bytes but is " +
                             std::to_string(sizeof(NodeTransition)));
}

void TestWriteRead() {
  // Test building NodeTransition and reading back values
  GraphId id(1111, 2, 5555);
  NodeTransition nodetrans(id, true);

  if (nodetrans.endnode() != id) {
    throw runtime_error("NodeTransition endnode test failed");
  }
  if (!nodetrans.up()) {
    throw runtime_error("NodeTransition up test failed");
  }
}
} // namespace

int main(void) {
  test::suite suite("nodetransition");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into NodeTransition
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
