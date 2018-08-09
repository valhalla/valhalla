#include "test.h"

#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/costfactory.h"
#include "sif/pedestriancost.h"
#include <valhalla/proto/directions_options.pb.h>

using namespace std;
using namespace valhalla::sif;

namespace {

void create_costing_options(valhalla::odin::DirectionsOptions& directions_options) {
  // Add options in the order specified
  //  for (const auto costing : {auto_, auto_shorter, bicycle, bus, hov,
  //                              motor_scooter, multimodal, pedestrian, transit,
  //                              truck, motorcycle, auto_data_fix}) {
  // TODO - accept RapidJSON as argument.
  const rapidjson::Document doc;
  ParseAutoCostOptions(doc, "/costing_options/auto", directions_options.add_costing_options());
  ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter",
                              directions_options.add_costing_options());
  ParseBicycleCostOptions(doc, "/costing_options/bicycle", directions_options.add_costing_options());
  ParseBusCostOptions(doc, "/costing_options/bus", directions_options.add_costing_options());
  ParseHOVCostOptions(doc, "/costing_options/hov", directions_options.add_costing_options());
  ParseMotorScooterCostOptions(doc, "/costing_options/motor_scooter",
                               directions_options.add_costing_options());
  directions_options.add_costing_options();
  ParsePedestrianCostOptions(doc, "/costing_options/pedestrian",
                             directions_options.add_costing_options());
  ParseTransitCostOptions(doc, "/costing_options/transit", directions_options.add_costing_options());
  ParseTruckCostOptions(doc, "/costing_options/truck", directions_options.add_costing_options());
  ParseMotorcycleCostOptions(doc, "/costing_options/motorcycle",
                             directions_options.add_costing_options());
  ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter",
                              directions_options.add_costing_options());
  ParseAutoDataFixCostOptions(doc, "/costing_options/auto_data_fix",
                              directions_options.add_costing_options());
}

void test_register() {
  valhalla::odin::DirectionsOptions directions_options;
  create_costing_options(directions_options);

  CostFactory<DynamicCost> factory;
  factory.Register(valhalla::odin::Costing::auto_, CreateAutoCost);
  factory.Register(valhalla::odin::Costing::auto_shorter, CreateAutoShorterCost);
  factory.Register(valhalla::odin::Costing::bicycle, CreateBicycleCost);
  factory.Register(valhalla::odin::Costing::pedestrian, CreatePedestrianCost);
  // TODO: then ask for some odin::DirectionsOptions& options
  auto car = factory.Create(valhalla::odin::Costing::auto_, directions_options);
}
} // namespace

int main(void) {
  test::suite suite("factory");

  suite.test(TEST_CASE(test_register));
  // TODO: many more

  return suite.tear_down();
}
