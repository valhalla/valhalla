#include "helper.h"

#include <utility>

int main() {
  using P = std::pair<float, float>;
  helper::static_assert_all(CT_NAMED(FloatPairIsStandardLayout, std::is_standard_layout_v<P>));
}