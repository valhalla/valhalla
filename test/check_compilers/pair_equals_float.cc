#include "helper.h"

#include <array>
#include <utility>

int main() {
  using P = std::pair<float, float>;
  using A = std::array<float, 2>;
  helper::static_assert_all(CT_NAMED(FloatPairSizeEqualsFloatArraySize, sizeof(P) == sizeof(A)));
}