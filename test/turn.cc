#include "test.h"
#include <cstdint>

#include "baldr/turn.h"

#include <string>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryGetType(uint32_t turn_degree, Turn::Type expected) {
  if (Turn::GetType(turn_degree) != expected) {
    throw std::runtime_error(std::string("Incorrect turn type for turn degree=") +
                             std::to_string(turn_degree));
  }
}

void TestGetType() {
  // Straight lower bound
  TryGetType(350, Turn::Type::kStraight);
  // Straight middle
  TryGetType(0, Turn::Type::kStraight);
  TryGetType(360, Turn::Type::kStraight);
  // Straight upper bound
  TryGetType(10, Turn::Type::kStraight);

  // Slight right lower bound
  TryGetType(11, Turn::Type::kSlightRight);
  // Slight right middle
  TryGetType(28, Turn::Type::kSlightRight);
  // Slight right upper bound
  TryGetType(44, Turn::Type::kSlightRight);

  // Right lower bound
  TryGetType(45, Turn::Type::kRight);
  // Right middle
  TryGetType(90, Turn::Type::kRight);
  TryGetType(450, Turn::Type::kRight);
  TryGetType(810, Turn::Type::kRight);
  // Right upper bound
  TryGetType(135, Turn::Type::kRight);

  // Sharp right lower bound
  TryGetType(136, Turn::Type::kSharpRight);
  // Sharp right middle
  TryGetType(158, Turn::Type::kSharpRight);
  // Sharp right upper bound
  TryGetType(169, Turn::Type::kSharpRight);

  // Reverse lower bound
  TryGetType(170, Turn::Type::kReverse);
  // Reverse middle
  TryGetType(180, Turn::Type::kReverse);
  // Reverse upper bound
  TryGetType(190, Turn::Type::kReverse);

  // Sharp left lower bound
  TryGetType(191, Turn::Type::kSharpLeft);
  // Sharp left middle
  TryGetType(203, Turn::Type::kSharpLeft);
  // Sharp left upper bound
  TryGetType(224, Turn::Type::kSharpLeft);

  // Left lower bound
  TryGetType(225, Turn::Type::kLeft);
  // Left middle
  TryGetType(270, Turn::Type::kLeft);
  // Left upper bound
  TryGetType(315, Turn::Type::kLeft);

  // Slight left lower bound
  TryGetType(316, Turn::Type::kSlightLeft);
  // Slight left middle
  TryGetType(333, Turn::Type::kSlightLeft);
  // Slight left upper bound
  TryGetType(349, Turn::Type::kSlightLeft);
}

} // namespace

int main(void) {
  test::suite suite("turn");

  suite.test(TEST_CASE(TestGetType));

  return suite.tear_down();
}
