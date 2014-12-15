#include "test.h"

#include "include/config.h"

using namespace std;

namespace {
  void test_version() {
    if(strlen(VERSION) < 1)
      throw runtime_error("Version string had no length");
  }

  void test_failure() {
    throw logic_error("This is a failing test");
  }
}

int main(void)
{
  test::suite suite("valhalla");

  suite.test(TEST_CASE(test_version));
  //suite.test(TEST_CASE(test_failure));

  return suite.tear_down();
}
