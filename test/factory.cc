#include "proto/options.pb.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/costfactory.h"
#include "sif/pedestriancost.h"

#include "test.h"

using namespace std;
using namespace valhalla;
using namespace valhalla::sif;

namespace {

TEST(Factory, Register) {
  Options options;
  const rapidjson::Document doc;
  CostFactory factory;
  options.set_costing_type(Costing::auto_);
  sif::ParseCosting(doc, "/costing_options", options);
  auto car = factory.Create(options);
  options.set_costing_type(Costing::bicycle);
  sif::ParseCosting(doc, "/costing_options", options);
  auto bike = factory.Create(options);
  auto truck = factory.Create(Costing::truck);
}

// TODO: add many more tests!

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
