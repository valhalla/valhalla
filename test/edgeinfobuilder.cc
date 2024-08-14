#include "test.h"
#include <cstdint>

#include <fstream>
#include <vector>

#include "baldr/edgeinfo.h"
#include "baldr/graphid.h"
#include "baldr/sign.h"
#include "mjolnir/edgeinfobuilder.h"
#include <boost/shared_array.hpp>
#include <memory>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

boost::shared_array<char> ToFileAndBack(const EdgeInfoBuilder& eibuilder) {
  // write EdgeInfoBuilder to binary file
  std::ofstream out_file("EdgeInfoBuilder_TestWriteRead.gph",
                         std::ios::out | std::ios::binary | std::ios::ate);
  if (out_file.is_open()) {
    out_file << eibuilder;
    out_file.close();
  } else {
    throw runtime_error("Failed to open file for writing");
  }

  // read EdgeInfo from binary file
  streampos size;
  boost::shared_array<char> memblock;

  std::ifstream in_file("EdgeInfoBuilder_TestWriteRead.gph",
                        std::ios::in | std::ios::binary | std::ios::ate);
  if (in_file.is_open()) {
    size = in_file.tellg();
    memblock.reset(new char[size]);
    in_file.seekg(0, ios::beg);
    in_file.read(memblock.get(), size);
    in_file.close();
  }

  return memblock;
}

TEST(EdgeInfoBuilder, TestWriteRead) {
  // Make a builder to write the info to disk
  EdgeInfoBuilder eibuilder;

  eibuilder.set_mean_elevation(kMinElevation - 100.0f);
  EXPECT_EQ(eibuilder.mean_elevation(), kMinElevation);

  eibuilder.set_mean_elevation(kMaxElevation + 100.0f);
  EXPECT_EQ(eibuilder.mean_elevation(), kMaxElevation);

  eibuilder.set_mean_elevation(0.0f);
  EXPECT_NEAR(eibuilder.mean_elevation(), 0, kElevationBinSize);

  eibuilder.set_mean_elevation(100.0f);
  EXPECT_NEAR(eibuilder.mean_elevation(), 100.0f, kElevationBinSize);

  eibuilder.set_wayid(6472927700900931484);

  // Name
  std::vector<NameInfo> name_info_list;
  name_info_list.push_back({963, 0, 0, 0, 0});
  name_info_list.push_back({957, 0, 0, 0, 0});
  name_info_list.push_back({862, 0, 0, 0, 0});
  eibuilder.set_name_info_list(name_info_list);

  // Shape
  std::vector<PointLL> shape;
  shape.push_back(PointLL(-76.3002, 40.0433));
  shape.push_back(PointLL(-76.3036, 40.043));
  eibuilder.set_shape(shape);

  // Make an edge info object from the memory
  boost::shared_array<char> memblock = ToFileAndBack(eibuilder);
  std::unique_ptr<EdgeInfo> ei(new EdgeInfo(memblock.get(), nullptr, 0));

  // TODO: errors thrown should say what was found and what was expected

  EXPECT_EQ(ei->wayid(), 6472927700900931484);

  // Validate the read in fields to the original EdgeInfoBuilder
  EXPECT_EQ(name_info_list.size(), ei->name_count());
  EXPECT_EQ(shape.size(), ei->shape().size());

  // Check the name indices
  for (uint8_t i = 0; i < ei->name_count(); ++i) {
    EXPECT_EQ(name_info_list[i].name_offset_, ei->GetNameInfo(i).name_offset_);
  }

  // Check the shape points
  for (size_t i = 0; i < ei->shape().size(); ++i) {
    ASSERT_TRUE(shape[i].ApproximatelyEqual(ei->shape()[i])) << "index " << i;
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
