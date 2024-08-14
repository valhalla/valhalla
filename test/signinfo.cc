#include "baldr/signinfo.h"
#include "baldr/graphid.h"
#include "mjolnir/uniquenames.h"

#include "test.h"

#include "mjolnir/graphbuilder.h"

using namespace valhalla::mjolnir;
using namespace valhalla::baldr;
using valhalla::mjolnir::GraphBuilder;

namespace {

TEST(Signinfo, ExitToTest) {
  OSMNode node{1234};
  OSMNode exit_node{1234};

  OSMWay way{};
  OSMData osmdata{};
  std::map<std::pair<uint8_t, uint8_t>, uint32_t> pronunciationMap;
  const std::map<std::pair<uint8_t, uint8_t>, uint32_t> langMap;

  bool fork = false;
  bool forward = true;

  exit_node.set_ref_index(osmdata.node_names.index("5"));
  exit_node.set_name_index(osmdata.node_names.index("PATP West Exit"));

  std::vector<SignInfo> signs;
  std::vector<std::string> linguistics;
  std::vector<std::pair<std::string, bool>> default_languages;
  const std::string linguistic_node_file = "test_sign_linguistic_node.bin";
  sequence<OSMNodeLinguistic> linguistic_node(linguistic_node_file, true);

  bool has_guide = GraphBuilder::CreateSignInfoList(exit_node, way, pronunciationMap, langMap,
                                                    osmdata, default_languages, linguistic_node,
                                                    signs, linguistics, fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 2) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kExitNumber) << "Exit 5 is not an exit Number";
    EXPECT_EQ(signs[0].text(), "5") << "Exitsign text is bad for Exit 5.";
    EXPECT_EQ(signs[1].type(), Sign::Type::kExitName) << "PATP West Exit is not an exit Name";
    EXPECT_EQ(signs[1].text(), "PATP West Exit") << "Exitsign text is bad for PATP West Exit.";
  } else {
    FAIL() << "Exit 5 failed to parse.";
  }

  node.set_exit_to_index(osmdata.node_names.index("US 11;To I 81;Carlisle;Harrisburg"));
  signs.clear();
  has_guide = GraphBuilder::CreateSignInfoList(node, way, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 4) {
    for (auto& exitsign : signs) {
      EXPECT_EQ(exitsign.type(), Sign::Type::kExitToward)
          << "US 11;To I 81;Carlisle;Harrisburg types are not all Toward";
    }
    EXPECT_EQ(signs[0].text(), "US 11");
    EXPECT_EQ(signs[1].text(), "I 81");
    EXPECT_EQ(signs[2].text(), "Carlisle");
    EXPECT_EQ(signs[3].text(), "Harrisburg");
  } else {
    FAIL() << "US 11/To I 81/Carlisle/Harrisburg failed to be parsed.";
  }

  signs.clear();
  node.set_exit_to_index(osmdata.node_names.index("US 11;Toward I 81;Carlisle;Harrisburg"));

  has_guide = GraphBuilder::CreateSignInfoList(node, way, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 4) {
    for (auto& exitsign : signs) {
      EXPECT_EQ(exitsign.type(), Sign::Type::kExitToward)
          << "US 11;Toward I 81;Carlisle;Harrisburg types are not all Toward";
    }
    EXPECT_EQ(signs[0].text(), "US 11");
    EXPECT_EQ(signs[1].text(), "I 81");
    EXPECT_EQ(signs[2].text(), "Carlisle");
    EXPECT_EQ(signs[3].text(), "Harrisburg");
  } else {
    FAIL() << "US 11;Toward I 81;Carlisle;Harrisburg failed to be parsed.";
  }

  signs.clear();
  node.set_exit_to_index(osmdata.node_names.index("I 95 To I 695"));

  has_guide = GraphBuilder::CreateSignInfoList(node, way, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 2) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kExitBranch) << "I 95 should be a branch.";
    EXPECT_EQ(signs[1].type(), Sign::Type::kExitToward) << "I 695 should be a toward.";
    EXPECT_EQ(signs[0].text(), "I 95");
    EXPECT_EQ(signs[1].text(), "I 695");
  } else {
    FAIL() << "I 95 To I 695 failed to be parsed.";
  }

  signs.clear();
  node.set_exit_to_index(osmdata.node_names.index("I 495 Toward I 270"));

  has_guide = GraphBuilder::CreateSignInfoList(node, way, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 2) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kExitBranch) << "I 495 should be a branch.";
    EXPECT_EQ(signs[1].type(), Sign::Type::kExitToward) << "I 270 should be a toward.";
    EXPECT_EQ(signs[0].text(), "I 495");
    EXPECT_EQ(signs[1].text(), "I 270");
  } else {
    FAIL() << "I 495 Toward I 270 failed to be parsed.";
  }

  signs.clear();
  node.set_exit_to_index(
      osmdata.node_names.index("I 495 Toward I 270 To I 95")); // default to toward.  Punt on parsing.

  has_guide = GraphBuilder::CreateSignInfoList(node, way, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 1) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kExitToward)
        << "I 495 Toward I 270 To I 95 should be a toward.";
    EXPECT_EQ(signs[0].text(), "I 495 Toward I 270 To I 95")
        << "Exitsign text is bad for I 495 Toward I 270 To I 95";
  } else {
    FAIL() << "I 495 Toward I 270 To I 95 failed to be parsed.";
  }

  // Add a ref branch sign
  signs.clear();
  auto index = osmdata.name_offset_map.index("I 495 North");
  way.set_destination_ref_index(index);
  has_guide = GraphBuilder::CreateSignInfoList(node, way, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 1) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kExitBranch) << "I 495 North should be a branch.";
    EXPECT_TRUE(signs[0].is_route_num()) << "I 495 North should be flagged as a route num";
    EXPECT_EQ(signs[0].text(), "I 495 North")
        << "Exitsign text is bad for I 495 North destination ref";
  } else {
    FAIL() << "destination ref I 495 North failed to create exist sign.";
  }

  signs.clear();
  index = osmdata.name_offset_map.index("I 495 North");
  way.set_destination_ref_index(index);
  has_guide = GraphBuilder::CreateSignInfoList(node, way, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, false, true);

  EXPECT_TRUE(has_guide) << "Guides should not be Exits";

  if (signs.size() == 1) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kGuideBranch) << "I 495 North should be a branch.";
    EXPECT_TRUE(signs[0].is_route_num()) << "I 495 North should be flagged as a route num";
    EXPECT_EQ(signs[0].text(), "I 495 North")
        << "Exitsign text is bad for I 495 North destination ref";
  } else {
    FAIL() << "destination ref I 495 North failed to create exist sign.";
  }

  // Add a ref toward sign
  OSMWay way2{};
  signs.clear();
  auto index2 = osmdata.name_offset_map.index("I 695 North");
  way2.set_destination_ref_to_index(index2);
  has_guide = GraphBuilder::CreateSignInfoList(node, way2, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, true, false);

  EXPECT_FALSE(has_guide) << "Exits should not be Guides";

  if (signs.size() == 1) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kExitToward) << "I 695 North should be a toward.";
    EXPECT_TRUE(signs[0].is_route_num()) << "I 695 North should be flagged as a route num";
    EXPECT_EQ(signs[0].text(), "I 695 North")
        << "Exitsign text is bad for I 695 North destination to ref";
  } else {
    FAIL() << "destination ref I 695 North failed to create exist sign.";
  }

  // Add a ref toward guide sign
  OSMWay way3{};
  signs.clear();
  auto index3 = osmdata.name_offset_map.index("I 695 North");
  way3.set_destination_ref_to_index(index3);
  has_guide = GraphBuilder::CreateSignInfoList(node, way2, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, false, true);

  EXPECT_TRUE(has_guide) << "Guides should not be Exits";

  if (signs.size() == 1) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kGuideToward) << "I 695 North should be a toward.";
    EXPECT_TRUE(signs[0].is_route_num()) << "I 695 North should be flagged as a route num";
    EXPECT_EQ(signs[0].text(), "I 695 North")
        << "Exitsign text is bad for I 695 North destination to ref";
  } else {
    FAIL() << "destination ref I 695 North failed to create exist sign.";
  }

  // Add a ref toward guide sign and we should not add a exit number or exit name.  note: using
  // exit_node
  signs.clear();
  has_guide = GraphBuilder::CreateSignInfoList(exit_node, way2, pronunciationMap, langMap, osmdata,
                                               default_languages, linguistic_node, signs, linguistics,
                                               fork, forward, false, true);

  EXPECT_TRUE(has_guide) << "Guides should not be Exits";

  if (signs.size() == 1) {
    EXPECT_EQ(signs[0].type(), Sign::Type::kGuideToward)
        << "I 695 North should be a toward. No exit 5 should exist.";
    EXPECT_TRUE(signs[0].is_route_num()) << "I 695 North should be flagged as a route num";
    EXPECT_EQ(signs[0].text(), "I 695 North")
        << "Exitsign text is bad for I 695 North destination to ref";
  } else {
    FAIL() << "destination ref I 695 North failed to create exist sign.  No exit 5 should exist.";
  }

  filesystem::remove(linguistic_node_file);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
