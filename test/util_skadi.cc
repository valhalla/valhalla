#include "skadi/util.h"

#include "midgard/util.h"

#include "test.h"

using namespace valhalla;

namespace {

TEST(UtilSkadi, Grade) {
  // we'll do a series of hundred percent grades up and down
  auto down_grade = -100.0;
  auto down_weight = skadi::energy_weighting(down_grade);
  auto up_grade = 100.0;
  auto up_weight = skadi::energy_weighting(up_grade);
  auto answer = (down_grade * down_weight + up_grade * up_weight) / (down_weight + up_weight);

  // check it
  auto grade = std::get<0>(skadi::weighted_grade({0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0}, 1.0));
  EXPECT_NEAR(grade, answer, .00001) << "Weighted grade was not right";

  // check another
  answer = (down_grade * down_weight * 3 + up_grade * up_weight) / (down_weight * 3 + up_weight);
  grade = std::get<0>(skadi::weighted_grade({0, 1, 0, -1, -2}, 1.0));
  EXPECT_NEAR(grade, answer, .00001) << "Weighted grade was not right";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}