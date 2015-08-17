#include "test.h"
#include "skadi/util.h"
using namespace valhalla;

namespace {

  void deltas() {

    auto d = skadi::discretized_deltas({0, 10, 0, 10, 0, 10, 0, 10, 0, 10, 0}, 1.0, 6);
    if(d.first != (50.0/10.0)/6.0)
      throw std::runtime_error("Descent wasn't right");
    if(d.second != (50.0/10.0)/6.0)
      throw std::runtime_error("Ascent wasn't right");
    d = skadi::discretized_deltas({0, 10, 0, 10, 0, 10, 0, 10, 0, 10, 0}, 1.0, 4.9);
    if(d.first != 1.0)
      throw std::runtime_error("Descent should have been clipped");
    if(d.second != 1.0)
      throw std::runtime_error("Ascent should have been clipped");
  }

}

int main() {
  test::suite suite("sample");

  suite.test(TEST_CASE(deltas));

  return suite.tear_down();
}
