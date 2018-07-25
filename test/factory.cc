#include "test.h"

#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/costfactory.h"
#include "sif/pedestriancost.h"
#include <valhalla/proto/directions_options.pb.h>

using namespace std;
using namespace valhalla::sif;

namespace {
void test_register() {
  CostFactory<DynamicCost> factory;
  factory.Register(valhalla::odin::Costing::auto_, CreateAutoCost);
  factory.Register(valhalla::odin::Costing::auto_shorter, CreateAutoShorterCost);
  factory.Register(valhalla::odin::Costing::bicycle, CreateBicycleCost);
  factory.Register(valhalla::odin::Costing::pedestrian, CreatePedestrianCost);
  // TODO: then ask for some odin::DirectionsOptions& options
  auto car = factory.Create(valhalla::odin::Costing::auto_, valhalla::odin::DirectionsOptions());
}
} // namespace

int main(void) {
  test::suite suite("factory");

  suite.test(TEST_CASE(test_register));
  // TODO: many more

  return suite.tear_down();
}
