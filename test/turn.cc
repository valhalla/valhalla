#include <cstdint>

#include "baldr/turn.h"

#include <string>

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

TEST(Turn, TestGetType) {
  // Straight lower bound
  EXPECT_EQ(Turn::GetType(350), Turn::Type::kStraight);
  // Straight middle
  EXPECT_EQ(Turn::GetType(0), Turn::Type::kStraight);
  EXPECT_EQ(Turn::GetType(360), Turn::Type::kStraight);
  // Straight upper bound
  EXPECT_EQ(Turn::GetType(10), Turn::Type::kStraight);

  // Slight right lower bound
  EXPECT_EQ(Turn::GetType(11), Turn::Type::kSlightRight);
  // Slight right middle
  EXPECT_EQ(Turn::GetType(28), Turn::Type::kSlightRight);
  // Slight right upper bound
  EXPECT_EQ(Turn::GetType(44), Turn::Type::kSlightRight);

  // Right lower bound
  EXPECT_EQ(Turn::GetType(45), Turn::Type::kRight);
  // Right middle
  EXPECT_EQ(Turn::GetType(90), Turn::Type::kRight);
  EXPECT_EQ(Turn::GetType(450), Turn::Type::kRight);
  EXPECT_EQ(Turn::GetType(810), Turn::Type::kRight);
  // Right upper bound
  EXPECT_EQ(Turn::GetType(135), Turn::Type::kRight);

  // Sharp right lower bound
  EXPECT_EQ(Turn::GetType(136), Turn::Type::kSharpRight);
  // Sharp right middle
  EXPECT_EQ(Turn::GetType(148), Turn::Type::kSharpRight);
  // Sharp right upper bound
  EXPECT_EQ(Turn::GetType(159), Turn::Type::kSharpRight);

  // Reverse lower bound
  EXPECT_EQ(Turn::GetType(160), Turn::Type::kReverse);
  // Reverse middle
  EXPECT_EQ(Turn::GetType(180), Turn::Type::kReverse);
  // Reverse upper bound
  EXPECT_EQ(Turn::GetType(200), Turn::Type::kReverse);

  // Sharp left lower bound
  EXPECT_EQ(Turn::GetType(201), Turn::Type::kSharpLeft);
  // Sharp left middle
  EXPECT_EQ(Turn::GetType(213), Turn::Type::kSharpLeft);
  // Sharp left upper bound
  EXPECT_EQ(Turn::GetType(224), Turn::Type::kSharpLeft);

  // Left lower bound
  EXPECT_EQ(Turn::GetType(225), Turn::Type::kLeft);
  // Left middle
  EXPECT_EQ(Turn::GetType(270), Turn::Type::kLeft);
  // Left upper bound
  EXPECT_EQ(Turn::GetType(315), Turn::Type::kLeft);

  // Slight left lower bound
  EXPECT_EQ(Turn::GetType(316), Turn::Type::kSlightLeft);
  // Slight left middle
  EXPECT_EQ(Turn::GetType(333), Turn::Type::kSlightLeft);
  // Slight left upper bound
  EXPECT_EQ(Turn::GetType(349), Turn::Type::kSlightLeft);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
