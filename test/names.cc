#include "baldr/graphconstants.h"
#include "mjolnir/uniquenames.h"
#include "mjolnir/osmway.h"

#include "test.h"

using namespace std;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

void NamesTest() {

  OSMWay w1{1234};
  OSMWay w2{1234};
  OSMWay w3{1234};

  UniqueNames ref_offset_map;
  UniqueNames name_offset_map;
  std::string ref = "I 79 North";

  w1.set_name_index(name_offset_map.index("William Flynn Highway"));
  w2.set_name_index(name_offset_map.index("Mon/Fayette Expressway"));
  w3.set_name_index(name_offset_map.index("Lancaster Pike"));

  w2.set_ref_index(ref_offset_map.index("PA 43"));
  w3.set_ref_index(ref_offset_map.index("PA 272"));

  w1.set_road_class(RoadClass::kMotorway);
  w2.set_road_class(RoadClass::kTrunk);
  w3.set_road_class(RoadClass::kPrimary);

  std::vector<std::string> w1_names = w1.GetNames(ref,ref_offset_map,name_offset_map);

  //if road class = kTrunk or kMotorway, then ref comes first.  ref from relation overrides
  //ref from ref_offset_map
  if (w1_names.at(0) != "I 79 North" || w1_names.at(1) != "William Flynn Highway")
    throw std::runtime_error("relation ref failed.");

  std::vector<std::string> w2_names = w2.GetNames("",ref_offset_map,name_offset_map);

  //if road class = kTrunk or kMotorway, then ref comes first.  use ref from ref_offset_map
  if (w2_names.at(0) != "PA 43" || w2_names.at(1) != "Mon/Fayette Expressway")
    throw std::runtime_error("ref_map failed.");

  std::vector<std::string> w3_names = w3.GetNames("",ref_offset_map,name_offset_map);

  //if Road class < kTrunk, then name first then ref using ref from ref_offset_map
  if (w3_names.at(0) != "Lancaster Pike" || w3_names.at(1) != "PA 272")
    throw std::runtime_error("Road class < kTrunk test failed.");

  w3_names.clear();
  w3_names = w3.GetNames("PA 555",ref_offset_map,name_offset_map);

  //if Road class < kTrunk, then name first then ref using ref from relations
  if (w3_names.at(0) != "Lancaster Pike" || w3_names.at(1) != "PA 555")
    throw std::runtime_error("Road class < kTrunk test failed(ref from relations).");

  w3.set_alt_name_index(name_offset_map.index("Lanc Pike"));
  w3.set_official_name_index(name_offset_map.index("LP"));
  w3.set_name_en_index(name_offset_map.index("LancP"));

  w3_names.clear();
  w3_names = w3.GetNames("",ref_offset_map,name_offset_map);

  //all other names should be last.

  if (w3_names.at(2) != "Lanc Pike")
    throw std::runtime_error("Alt name failed.");

  if (w3_names.at(3) != "LP")
    throw std::runtime_error("official name failed.");

  if (w3_names.at(4) != "LancP")
    throw std::runtime_error("name en failed.");

}

}

int main() {
  test::suite suite("Names");

  // Test setting and getting on random sizes of bit tables
  suite.test(TEST_CASE(NamesTest));

  return suite.tear_down();
}
