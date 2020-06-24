#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/costfactory.h"
#include "sif/pedestriancost.h"
#include <valhalla/proto/options.pb.h>

#include "test.h"

using namespace std;
using namespace valhalla;
using namespace valhalla::sif;

namespace {

void create_costing_options(Options& options) {
  // TODO - accept RapidJSON as argument.
  const rapidjson::Document doc;
  for (int i = 0; i < Costing_ARRAYSIZE; ++i) {
    Costing costing = static_cast<Costing>(i);
    // Create the costing string
    const auto& costing_str = valhalla::Costing_Enum_Name(costing);
    // Create the costing options key
    const auto costing_options_key = "/costing_options/" + costing_str;
    // Parse the options
    sif::ParseCostOptions(costing, doc, costing_options_key, options.add_costing_options());
  }
}

TEST(Factory, Register) {
  Options options;
  create_costing_options(options);

  CostFactory<DynamicCost> factory;
  factory.Register(Costing::auto_, CreateAutoCost);
  factory.Register(Costing::auto_shorter, CreateAutoShorterCost);
  factory.Register(Costing::bicycle, CreateBicycleCost);
  factory.Register(Costing::pedestrian, CreatePedestrianCost);
  // TODO: then ask for some odin::Options& options
  auto car = factory.Create(Costing::auto_, options);
  options.set_costing(Costing::bicycle);
  auto bike = factory.Create(options);
}

// TODO: add many more tests!

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
