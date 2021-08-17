#include "baldr/graphconstants.h"
#include "mjolnir/osmway.h"
#include "mjolnir/uniquenames.h"
#include <iostream>

#include "test.h"

using namespace std;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

TEST(Names, NamesTest) {

  OSMWay w1{1234};
  OSMWay w2{1234};
  OSMWay w3{1234};

  UniqueNames name_offset_map;
  std::string ref = "I 79 North";

  w1.set_name_index(name_offset_map.index("William Flynn Highway"));
  w2.set_name_index(name_offset_map.index("Mon/Fayette Expressway"));
  w3.set_name_index(name_offset_map.index("Lancaster Pike"));

  w2.set_ref_index(name_offset_map.index("PA 43"));
  w3.set_ref_index(name_offset_map.index("PA 272"));

  w1.set_road_class(RoadClass::kMotorway);
  w2.set_road_class(RoadClass::kTrunk);
  w3.set_road_class(RoadClass::kPrimary);

  uint16_t types;
  std::vector<std::string> w1_names = w1.GetNames(ref, name_offset_map, types);

  // if road class = kTrunk or kMotorway, then ref comes first.  ref from relation overrides
  // ref from name_offset_map
  EXPECT_EQ(w1_names.at(0), "I 79 North");
  EXPECT_EQ(w1_names.at(1), "William Flynn Highway");

  EXPECT_EQ(types, 1) << "relation ref failed.  ref not in correct position.";

  std::vector<std::string> w2_names = w2.GetNames("", name_offset_map, types);

  // if road class = kTrunk or kMotorway, then ref comes first.  use ref from name_offset_map
  EXPECT_EQ(w2_names.at(0), "PA 43");
  EXPECT_EQ(w2_names.at(1), "Mon/Fayette Expressway");

  EXPECT_EQ(types, 1) << "ref_map failed.  ref not in correct position.";

  std::vector<std::string> w3_names = w3.GetNames("", name_offset_map, types);

  // if Road class < kTrunk, then name first then ref using ref from name_offset_map
  EXPECT_EQ(w3_names.at(0), "Lancaster Pike") << "Road class < kTrunk test failed.";
  EXPECT_EQ(w3_names.at(1), "PA 272") << "Road class < kTrunk test failed.";

  EXPECT_EQ(types, 2) << "Road class < kTrunk test failed.  ref not in correct position.";

  w3_names.clear();
  w3_names = w3.GetNames("PA 555", name_offset_map, types);

  // if Road class < kTrunk, then name first then ref using ref from relations
  EXPECT_EQ(w3_names.at(0), "Lancaster Pike") << "ref from relations";
  EXPECT_EQ(w3_names.at(1), "PA 555") << "ref from relations";

  EXPECT_EQ(types, 2)
      << "Road class < kTrunk test failed(ref from relations).  ref not in correct position.";

  w3.set_alt_name_index(name_offset_map.index("Lanc Pike"));
  w3.set_official_name_index(name_offset_map.index("LP"));
  w3.set_name_en_index(name_offset_map.index("LancP"));

  w3_names.clear();
  w3_names = w3.GetNames("", name_offset_map, types);

  EXPECT_EQ(types, 2) << "all other names test failed.  ref not in correct position.";

  // all other names should be last.

  EXPECT_EQ(w3_names.at(2), "Lanc Pike") << "Alt name failed.";
  EXPECT_EQ(w3_names.at(3), "LP") << "official name failed.";
  EXPECT_EQ(w3_names.at(4), "LancP") << "name en failed.";
}

TEST(Names, TaggedNamesTest) {

  OSMWay w1{1234};
  OSMWay w2{1234};

  UniqueNames name_offset_map;

  w1.set_tunnel_name_index(name_offset_map.index("Ted Williams Tunnel"));
  w2.set_tunnel_name_index(name_offset_map.index("Fort McHenry Tunnel"));
  w1.set_road_class(RoadClass::kMotorway);
  w2.set_road_class(RoadClass::kMotorway);

  std::vector<std::string> w1_tagged_values = w1.GetTaggedValues(name_offset_map);
  EXPECT_EQ(w1_tagged_values.at(0), "1Ted Williams Tunnel");

  std::vector<std::string> w2_tagged_values = w2.GetTaggedValues(name_offset_map);
  EXPECT_EQ(w2_tagged_values.at(0), "1Fort McHenry Tunnel");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
