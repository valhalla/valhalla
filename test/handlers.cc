#include "test.h"


int main() {
  test::suite suite("handlers");

  //suite.test(TEST_CASE(TestRouteHanlder));

  //suite.test(TEST_CASE(TestNearestHanlder));

  //suite.test(TEST_CASE(TestLocateHanlder));

  return suite.tear_down();
}
