#include "test.h"

#include "sif/costfactory.h"
#include "sif/autocost.h"
#include "sif/pedestriancost.h"
#include "sif/bicyclecost.h"

using namespace std;
using namespace valhalla::sif;

namespace {
  void test_register() {
    CostFactory<DynamicCost> factory;
    factory.Register("auto", CreateAutoCost);
    factory.Register("auto_shorter", CreateAutoShorterCost);
    factory.Register("bicycle", CreateBicycleCost);
    factory.Register("pedestrian", CreatePedestrianCost);
    //TODO: then ask for some
    auto car = factory.Create("auto", boost::property_tree::ptree{});
  }
}

int main(void)
{
  test::suite suite("factory");

  suite.test(TEST_CASE(test_register));
  //TODO: many more

  return suite.tear_down();
}
