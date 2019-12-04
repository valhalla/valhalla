#include <algorithm>
#include <cstdint>
#include <vector>

#include "baldr/sign.h"
#include "baldr/signinfo.h"

#include "odin/sign.h"

#include "test.h"

using namespace std;
using namespace valhalla::odin;
using namespace valhalla::baldr;

// Expected size is 8 bytes. We want to alert if somehow any change grows
// this structure size as that indicates incompatible tiles.
constexpr size_t kSignExpectedSize = 8;

namespace {

void test_sizeof() {
  if (sizeof(valhalla::baldr::Sign) != kSignExpectedSize)
    throw std::runtime_error("Sign size should be " + std::to_string(kSignExpectedSize) + " bytes" +
                             " but is " + std::to_string(sizeof(valhalla::baldr::Sign)));
}

void TryCtor(const std::string& text, const bool is_route_number) {
  valhalla::odin::Sign sign(text, is_route_number);
  uint32_t consecutive_count = 0;

  if (text != sign.text())
    throw std::runtime_error("Incorrect sign text");

  if (is_route_number != sign.is_route_number())
    throw std::runtime_error("Incorrect sign is_route_number boolean");

  if (consecutive_count != sign.consecutive_count())
    throw std::runtime_error("Incorrect sign consecutive count");
}

void TestCtor() {
  // Exit number
  TryCtor("51A", false);

  // Exit branch
  TryCtor("I 81 South", true);

  // Exit toward
  TryCtor("Carlisle", false);

  // Exit name
  TryCtor("Harrisburg East", false);
}

void TryDescendingSortByConsecutiveCount(std::vector<valhalla::odin::Sign>& signs,
                                         const std::vector<valhalla::odin::Sign>& expectedSigns) {

  if (signs.size() != expectedSigns.size())
    throw std::runtime_error("DescendingSortByConsecutiveCount size mismatch");

  std::sort(signs.begin(), signs.end(),
            [](const valhalla::odin::Sign& lhs, const valhalla::odin::Sign& rhs) {
              return lhs.consecutive_count() > rhs.consecutive_count();
            });

  for (size_t x = 0, n = signs.size(); x < n; ++x) {
    if (signs.at(x).consecutive_count() != expectedSigns.at(x).consecutive_count())
      throw std::runtime_error("Incorrect DescendingSortByConsecutiveCount");
  }
}

void TestDescendingSortByConsecutiveCount_0_1() {
  valhalla::odin::Sign signConsecutiveCount0("Elizabethtown", false);

  valhalla::odin::Sign signConsecutiveCount1("Hershey", false);
  signConsecutiveCount1.set_consecutive_count(1);

  std::vector<valhalla::odin::Sign> signs = {signConsecutiveCount0, signConsecutiveCount1};

  TryDescendingSortByConsecutiveCount(signs, {signConsecutiveCount1, signConsecutiveCount0});
}

void TestDescendingSortByConsecutiveCount_1_2() {
  valhalla::odin::Sign signConsecutiveCount1("I 81 South", true);
  signConsecutiveCount1.set_consecutive_count(1);

  valhalla::odin::Sign signConsecutiveCount2("I 81 North", true);
  signConsecutiveCount2.set_consecutive_count(2);

  std::vector<valhalla::odin::Sign> signs = {signConsecutiveCount1, signConsecutiveCount2};

  TryDescendingSortByConsecutiveCount(signs, {signConsecutiveCount2, signConsecutiveCount1});
}

void TestDescendingSortByConsecutiveCount_2_4() {
  valhalla::odin::Sign signConsecutiveCount2("51A", false);
  signConsecutiveCount2.set_consecutive_count(2);

  valhalla::odin::Sign signConsecutiveCount4("51B", false);
  signConsecutiveCount4.set_consecutive_count(4);

  std::vector<valhalla::odin::Sign> signs = {signConsecutiveCount2, signConsecutiveCount4};

  TryDescendingSortByConsecutiveCount(signs, {signConsecutiveCount4, signConsecutiveCount2});
}

void TestDescendingSortByConsecutiveCount_0_1_2() {
  valhalla::odin::Sign signConsecutiveCount0("Towson", false);

  valhalla::odin::Sign signConsecutiveCount1("Baltimore", false);
  signConsecutiveCount1.set_consecutive_count(1);

  valhalla::odin::Sign signConsecutiveCount2("New York", false);
  signConsecutiveCount2.set_consecutive_count(2);

  std::vector<valhalla::odin::Sign> signs = {signConsecutiveCount0, signConsecutiveCount1,
                                             signConsecutiveCount2};

  // Reverse order
  TryDescendingSortByConsecutiveCount(signs, {signConsecutiveCount2, signConsecutiveCount1,
                                              signConsecutiveCount0});

  signs = {signConsecutiveCount2, signConsecutiveCount1, signConsecutiveCount0};

  // In order
  TryDescendingSortByConsecutiveCount(signs, {signConsecutiveCount2, signConsecutiveCount1,
                                              signConsecutiveCount0});

  signs = {signConsecutiveCount0, signConsecutiveCount2, signConsecutiveCount1};

  // Mixed order
  TryDescendingSortByConsecutiveCount(signs, {signConsecutiveCount2, signConsecutiveCount1,
                                              signConsecutiveCount0});
}

} // namespace

int main() {
  test::suite suite("sign");

  // Constructor
  suite.test(TEST_CASE(TestCtor));

  // Test sizeof the structure
  suite.test(TEST_CASE(test_sizeof));

  // DescendingSortByConsecutiveCount_0_1
  suite.test(TEST_CASE(TestDescendingSortByConsecutiveCount_0_1));

  // DescendingSortByConsecutiveCount_1_2
  suite.test(TEST_CASE(TestDescendingSortByConsecutiveCount_1_2));

  // DescendingSortByConsecutiveCount_2_4
  suite.test(TEST_CASE(TestDescendingSortByConsecutiveCount_2_4));

  // DescendingSortByConsecutiveCount_0_1_2
  suite.test(TEST_CASE(TestDescendingSortByConsecutiveCount_0_1_2));

  return suite.tear_down();
}
