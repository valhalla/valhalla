#include "test.h"
#include "valhalla/odin/sign.h"

#include <valhalla/baldr/sign.h>
#include <valhalla/baldr/signinfo.h>

#include <vector>
#include <algorithm>

using namespace std;
using namespace valhalla::odin;
using namespace valhalla::baldr;

namespace {

void TryCtor(const valhalla::baldr::Sign::Type& type, const std::string& text) {
  valhalla::odin::Sign sign(type, text);
  uint32_t consecutive_count = 0;
  if (type != sign.type())
    throw std::runtime_error("Incorrect sign type");

  if (text != sign.text())
    throw std::runtime_error("Incorrect sign text");

  if (consecutive_count != sign.consecutive_count())
    throw std::runtime_error("Incorrect sign consecutive count");
}

void TestCtor() {
  // Exit number
  TryCtor(valhalla::baldr::Sign::Type::kExitNumber, "51A");

  // Exit branch
  TryCtor(valhalla::baldr::Sign::Type::kExitBranch, "I 81 South");

  // Exit number
  TryCtor(valhalla::baldr::Sign::Type::kExitToward, "Carlisle");

  // Exit number
  TryCtor(valhalla::baldr::Sign::Type::kExitName, "Harrisburg East");
}

void TryDescendingSortByConsecutiveCount(
    std::vector<valhalla::odin::Sign>& signs,
    const std::vector<valhalla::odin::Sign>& expectedSigns) {

  if (signs.size() != expectedSigns.size())
    throw std::runtime_error("DescendingSortByConsecutiveCount size mismatch");

  std::sort(signs.begin(), signs.end(), DescendingSortByConsecutiveCount);

  for (size_t x = 0, n = signs.size(); x < n; ++x) {
    if (signs.at(x).consecutive_count()
        != expectedSigns.at(x).consecutive_count())
      throw std::runtime_error("Incorrect DescendingSortByConsecutiveCount");
  }
}

void TestDescendingSortByConsecutiveCount_0_1() {
  valhalla::odin::Sign signConsecutiveCount0(
      valhalla::baldr::Sign::Type::kExitToward, "Elizabethtown");

  valhalla::odin::Sign signConsecutiveCount1(
      valhalla::baldr::Sign::Type::kExitToward, "Hershey");
  signConsecutiveCount1.set_consecutive_count(1);

  std::vector<valhalla::odin::Sign> signs = { signConsecutiveCount0,
      signConsecutiveCount1 };

  TryDescendingSortByConsecutiveCount(signs, { signConsecutiveCount1,
                                          signConsecutiveCount0 });

}

void TestDescendingSortByConsecutiveCount_1_2() {
  valhalla::odin::Sign signConsecutiveCount1(
      valhalla::baldr::Sign::Type::kExitBranch, "I 81 South");
  signConsecutiveCount1.set_consecutive_count(1);

  valhalla::odin::Sign signConsecutiveCount2(
      valhalla::baldr::Sign::Type::kExitBranch, "I 81 North");
  signConsecutiveCount2.set_consecutive_count(2);

  std::vector<valhalla::odin::Sign> signs = { signConsecutiveCount1,
      signConsecutiveCount2 };

  TryDescendingSortByConsecutiveCount(signs, { signConsecutiveCount2,
                                          signConsecutiveCount1 });

}

void TestDescendingSortByConsecutiveCount_2_4() {
  valhalla::odin::Sign signConsecutiveCount2(
      valhalla::baldr::Sign::Type::kExitNumber, "51A");
  signConsecutiveCount2.set_consecutive_count(2);

  valhalla::odin::Sign signConsecutiveCount4(
      valhalla::baldr::Sign::Type::kExitBranch, "51B");
  signConsecutiveCount4.set_consecutive_count(4);

  std::vector<valhalla::odin::Sign> signs = { signConsecutiveCount2,
      signConsecutiveCount4 };

  TryDescendingSortByConsecutiveCount(signs, { signConsecutiveCount4,
                                          signConsecutiveCount2 });

}

void TestDescendingSortByConsecutiveCount_0_1_2() {
  valhalla::odin::Sign signConsecutiveCount0(
      valhalla::baldr::Sign::Type::kExitToward, "Towson");

  valhalla::odin::Sign signConsecutiveCount1(
      valhalla::baldr::Sign::Type::kExitToward, "Baltimore");
  signConsecutiveCount1.set_consecutive_count(1);

  valhalla::odin::Sign signConsecutiveCount2(
      valhalla::baldr::Sign::Type::kExitToward, "New York");
  signConsecutiveCount2.set_consecutive_count(2);

  std::vector<valhalla::odin::Sign> signs = { signConsecutiveCount0,
      signConsecutiveCount1, signConsecutiveCount2 };

  // Reverse order
  TryDescendingSortByConsecutiveCount(signs, { signConsecutiveCount2,
                                          signConsecutiveCount1,
                                          signConsecutiveCount0 });

  signs = {signConsecutiveCount2,
    signConsecutiveCount1, signConsecutiveCount0};

  // In order
  TryDescendingSortByConsecutiveCount(signs, { signConsecutiveCount2,
                                          signConsecutiveCount1,
                                          signConsecutiveCount0 });

  signs = {signConsecutiveCount0,
    signConsecutiveCount2, signConsecutiveCount1};

  // Mixed order
  TryDescendingSortByConsecutiveCount(signs, { signConsecutiveCount2,
                                          signConsecutiveCount1,
                                          signConsecutiveCount0 });

}

}

int main() {
  test::suite suite("sign");

  // Constructor
  suite.test(TEST_CASE(TestCtor));

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
