#include "test.h"

#include "sif/costfactory.h"

using namespace std;
using namespace valhalla::sif;

namespace {
  void test_register() {
    //TODO: register all the costing models
    //TODO: then ask for some
  }
}

int main(void)
{
  test::suite suite("factory");

  suite.test(TEST_CASE(test_register));
  //TODO: many more

  return suite.tear_down();
}
