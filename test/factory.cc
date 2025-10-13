#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "proto/options.pb.h"
#include "sif/costfactory.h"
#include "test/test.h"

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

using namespace std;
using namespace valhalla;
using namespace valhalla::sif;

namespace {

TEST(Factory, Register) {
  Options options;
  const rapidjson::Document doc;
  CostFactory factory;
  baldr::GraphReader reader(test::make_config("test/data/utrecht_tiles"));
  options.set_costing_type(Costing::auto_);
  sif::ParseCosting(doc, "/costing_options", options);
  auto car = factory.Create(options, reader);
  options.set_costing_type(Costing::bicycle);
  sif::ParseCosting(doc, "/costing_options", options);
  auto bike = factory.Create(options, reader);
  auto truck = factory.Create(Costing::truck, reader);
}

// TODO: add many more tests!

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
