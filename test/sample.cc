#include "test.h"

namespace {

void sample() {

}

}

int main() {
  test::suite suite("sample");

  suite.test(TEST_CASE(sample));

  return suite.tear_down();
}
