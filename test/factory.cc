#include "test.h"

#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/costfactory.h"
#include "sif/pedestriancost.h"
#include <valhalla/proto/options.pb.h>

using namespace std;
using namespace valhalla;
using namespace valhalla::sif;

namespace {

void create_costing_options(Options& options) {
  // Add options in the order specified
  //  for (const auto costing : {auto_, auto_shorter, bicycle, bus, hov,
  //                              motor_scooter, multimodal, pedestrian, transit,
  //                              truck, motorcycle, auto_data_fix}) {
  // TODO - accept RapidJSON as argument.
  const rapidjson::Document doc;
  ParseAutoCostOptions(doc, "/costing_options/auto", options.add_costing_options());
  ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter", options.add_costing_options());
  ParseBicycleCostOptions(doc, "/costing_options/bicycle", options.add_costing_options());
  ParseBusCostOptions(doc, "/costing_options/bus", options.add_costing_options());
  ParseHOVCostOptions(doc, "/costing_options/hov", options.add_costing_options());
  ParseTaxiCostOptions(doc, "/costing_options/taxi", options.add_costing_options());
  ParseMotorScooterCostOptions(doc, "/costing_options/motor_scooter", options.add_costing_options());
  options.add_costing_options();
  ParsePedestrianCostOptions(doc, "/costing_options/pedestrian", options.add_costing_options());
  ParseTransitCostOptions(doc, "/costing_options/transit", options.add_costing_options());
  ParseTruckCostOptions(doc, "/costing_options/truck", options.add_costing_options());
  ParseMotorcycleCostOptions(doc, "/costing_options/motorcycle", options.add_costing_options());
  ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter", options.add_costing_options());
  ParseAutoDataFixCostOptions(doc, "/costing_options/auto_data_fix", options.add_costing_options());
}

void test_register() {
  Options options;
  create_costing_options(options);

  CostFactory<DynamicCost> factory;
  factory.Register(Costing::auto_, CreateAutoCost);
  factory.Register(Costing::auto_shorter, CreateAutoShorterCost);
  factory.Register(Costing::bicycle, CreateBicycleCost);
  factory.Register(Costing::pedestrian, CreatePedestrianCost);
  // TODO: then ask for some odin::Options& options
  auto car = factory.Create(Costing::auto_, options);
}
} // namespace

int main(void) {
  test::suite suite("factory");

  suite.test(TEST_CASE(test_register));
  // TODO: many more

  return suite.tear_down();
}
