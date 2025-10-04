#include "helper.h"

#include <utility>

int main() {
  using P = std::pair<float, float>;
  helper::static_assert_all(CT_NAMED(FloatPairFirstOffsetIsZero, offsetof(P, first) == 0),
                            CT_NAMED(FloatPairNoInterPadding, offsetof(P, second) == sizeof(float)));
}