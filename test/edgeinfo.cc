#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "mjolnir/edgeinfobuilder.h"

#include <boost/shared_array.hpp>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
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
    throw std::runtime_error("Failed to open file for writing");
  }

  // read EdgeInfo from binary file
  std::streampos size;
  boost::shared_array<char> memblock;

  std::ifstream in_file("EdgeInfoBuilder_TestWriteRead.gph",
                        std::ios::in | std::ios::binary | std::ios::ate);
  if (in_file.is_open()) {
    size = in_file.tellg();
    memblock.reset(new char[size]);
    in_file.seekg(0, std::ios::beg);
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

TEST(EdgeInfo, TaggedValueSize_Layer) {
  // Layer: tag byte + layer value + null terminator
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kLayer);
  tagged_value += static_cast<char>(-3); // layer value
  tagged_value += '\0';                  // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  // Should return 3 (tag + value + null)
  EXPECT_EQ(size, 3) << "Layer tagged value should be 3 bytes (tag + value + null)";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_Tunnel) {
  // Tunnel: tag byte + tunnel name + null terminator
  std::string tunnel_name = "Fort McHenry Tunnel";
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kTunnel);
  tagged_value += tunnel_name;
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  // Should return tag + name length + null
  size_t expected_size = 1 + tunnel_name.size() + 1;
  EXPECT_EQ(size, expected_size) << "Tunnel tagged value size mismatch";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_Bridge) {
  // Bridge: tag byte + bridge name + null terminator
  std::string bridge_name = "Golden Gate Bridge";
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kBridge);
  tagged_value += bridge_name;
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  // Should return tag + name length + null
  size_t expected_size = 1 + bridge_name.size() + 1;
  EXPECT_EQ(size, expected_size) << "Bridge tagged value size mismatch";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_Level) {
  // Level: tag byte + level string + null terminator
  std::string level_str = "2";
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kLevel);
  tagged_value += level_str;
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  // Should return tag + level string length + null
  size_t expected_size = 1 + level_str.size() + 1;
  EXPECT_EQ(size, expected_size) << "Level tagged value size mismatch";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_LevelRef) {
  // LevelRef: tag byte + level ref string + null terminator
  std::string level_ref_str = "Ground Floor";
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kLevelRef);
  tagged_value += level_ref_str;
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  // Should return tag + level ref string length + null
  size_t expected_size = 1 + level_ref_str.size() + 1;
  EXPECT_EQ(size, expected_size) << "LevelRef tagged value size mismatch";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_BssInfo) {
  // BssInfo: tag byte + info string + null terminator
  std::string bss_info = "station_123";
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kBssInfo);
  tagged_value += bss_info;
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  // Should return tag + info string length + null
  size_t expected_size = 1 + bss_info.size() + 1;
  EXPECT_EQ(size, expected_size) << "BssInfo tagged value size mismatch";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_OSMNodeIds) {
  // OSMNodeIds: tag byte + varint size + encoded data + null terminator
  std::vector<uint64_t> node_ids = {987653, 987654, 987655, 987656};
  std::string encoded_ids = encode7int(node_ids);

  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kOSMNodeIds);
  // Add varint size using encode7int with a single-element vector
  tagged_value += encode7int(std::vector<int32_t>{static_cast<int32_t>(encoded_ids.size())});
  // Add encoded data
  tagged_value += encoded_ids;
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  EXPECT_EQ(size, tagged_value.size()) << "OSMNodeIds TaggedValueSize should match actual size";
  EXPECT_GT(size, 1) << "OSMNodeIds should have size > 1";
}

TEST(EdgeInfo, TaggedValueSize_Levels) {
  // Levels: tag byte + varint size + varint precision + encoded levels + null terminator
  // Build the levels data using encode7int with single-element vectors (same as encode_level in
  // osmway.cc)
  std::string levels_data = encode7int(std::vector<int32_t>{5}) +
                            encode7int(std::vector<int32_t>{10}) +
                            encode7int(std::vector<int32_t>{100});

  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kLevels);
  // Add varint size using encode7int
  tagged_value += encode7int(std::vector<int32_t>{static_cast<int32_t>(levels_data.size())});
  // Add levels data
  tagged_value += levels_data;
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  EXPECT_EQ(size, tagged_value.size()) << "Levels TaggedValueSize should match actual size";
  EXPECT_GT(size, 1) << "Levels should have size > 1";
}

TEST(EdgeInfo, TaggedValueSize_ConditionalSpeedLimits) {
  // ConditionalSpeedLimits: tag byte + ConditionalSpeedLimit struct + null terminator
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kConditionalSpeedLimits);
  // Add the struct data (8 bytes filled with zeros for testing)
  char limit_data[sizeof(ConditionalSpeedLimit)] = {0};
  tagged_value.append(limit_data, sizeof(ConditionalSpeedLimit));
  tagged_value += '\0'; // null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  size_t expected_size = 1 + sizeof(ConditionalSpeedLimit) + 1;
  EXPECT_EQ(size, expected_size) << "ConditionalSpeedLimits tagged value size mismatch";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_Linguistic) {
  // Linguistic: tag byte + multiple linguistic entries + null terminator
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kLinguistic);

  // Add a linguistic entry
  linguistic_text_header_t header;
  std::memset(&header, 0, sizeof(header));
  header.language_ = 1;
  header.length_ = 5;
  header.phonetic_alphabet_ = 1;
  header.name_index_ = 0;

  // Write header (only first 3 bytes, as DO_NOT_USE_ is not stored)
  tagged_value.append(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize);
  // Write pronunciation
  tagged_value += "hello";

  // Add null terminator
  tagged_value += '\0';

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  EXPECT_EQ(size, tagged_value.size()) << "Linguistic TaggedValueSize should match actual size";
  EXPECT_GT(size, 1) << "Linguistic should have size > 1";
}

TEST(EdgeInfo, TaggedValueSize_EmptyString) {
  // Test with an empty string after the tag (edge case)
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kTunnel);
  tagged_value += '\0'; // Just tag + null terminator

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  // Should return 2 (tag + null)
  EXPECT_EQ(size, 2) << "Empty tunnel name should be 2 bytes (tag + null)";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

TEST(EdgeInfo, TaggedValueSize_MultipleLayerValues) {
  // Test multiple different layer values
  for (int8_t layer = -5; layer <= 5; ++layer) {
    if (layer == 0) {
      // Layer 0 is never serialized (it's the OSM default)
      continue;
    }

    std::string tagged_value;
    tagged_value += static_cast<char>(TaggedValue::kLayer);
    tagged_value += static_cast<char>(layer);
    tagged_value += '\0';

    size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

    EXPECT_EQ(size, 3) << "Layer " << static_cast<int>(layer) << " should be 3 bytes";
    EXPECT_EQ(size, tagged_value.size())
        << "TaggedValueSize should match actual size for layer " << static_cast<int>(layer);
  }
}

TEST(EdgeInfo, TaggedValueSize_LongStrings) {
  std::string long_tunnel_name(100, 'x');
  std::string tagged_value;
  tagged_value += static_cast<char>(TaggedValue::kTunnel);
  tagged_value += long_tunnel_name;
  tagged_value += '\0';

  size_t size = EdgeInfo::TaggedValueSize(tagged_value.data());

  size_t expected_size = 1 + long_tunnel_name.size() + 1;
  EXPECT_EQ(size, expected_size) << "Long tunnel name size mismatch";
  EXPECT_EQ(size, tagged_value.size()) << "TaggedValueSize should match actual size";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
