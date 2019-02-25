#include "baldr/signinfo.h"
#include "baldr/graphid.h"
#include "mjolnir/uniquenames.h"

#include "test.h"

#include "mjolnir/graphbuilder.h"

using namespace std;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;
using valhalla::mjolnir::GraphBuilder;

namespace {

void ExitToTest() {
  OSMNode node{1234};
  OSMWay way{};
  OSMData osmdata{};
  bool fork = false;
  bool forward = true;

  node.set_exit_to_index(osmdata.node_names.index("US 11;To I 81;Carlisle;Harrisburg"));

  std::vector<SignInfo> exitsigns;
  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 4) {
    for (auto& exitsign : exitsigns) {
      if (exitsign.type() != Sign::Type::kExitToward)
        throw std::runtime_error("US 11;To I 81;Carlisle;Harrisburg types are not all Toward");
    }

    if (exitsigns[0].text() != "US 11" && exitsigns[1].text() != "I 81" &&
        exitsigns[2].text() != "Carlisle" && exitsigns[3].text() != "Harrisburg")
      throw std::runtime_error("Exitsign text is bad for US 11;To I 81;Carlisle;Harrisburg.");
  } else
    throw std::runtime_error("US 11/To I 81/Carlisle/Harrisburg failed to be parsed.  " +
                             std::to_string(exitsigns.size()));

  exitsigns.clear();
  node.set_exit_to_index(osmdata.node_names.index("US 11;Toward I 81;Carlisle;Harrisburg"));

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 4) {
    for (auto& exitsign : exitsigns) {
      if (exitsign.type() != Sign::Type::kExitToward)
        throw std::runtime_error("US 11;Toward I 81;Carlisle;Harrisburg types are not all Toward");
    }
    if (exitsigns[0].text() != "US 11" && exitsigns[1].text() != "I 81" &&
        exitsigns[2].text() != "Carlisle" && exitsigns[3].text() != "Harrisburg")
      throw std::runtime_error("Exitsign text is bad for US 11;To I 81;Carlisle;Harrisburg.");
  } else
    throw std::runtime_error("US 11;Toward I 81;Carlisle;Harrisburg failed to be parsed.");

  exitsigns.clear();
  node.set_exit_to_index(osmdata.node_names.index("I 95 To I 695"));

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 2) {
    if (exitsigns[0].type() != Sign::Type::kExitBranch)
      throw std::runtime_error("I 95 should be a branch.");

    if (exitsigns[1].type() != Sign::Type::kExitToward)
      throw std::runtime_error("I 695 should be a toward.");

    if (exitsigns[0].text() != "I 95" && exitsigns[1].text() != "I 695")
      throw std::runtime_error("Exitsign text is bad for I 95 To I 695");
  } else
    throw std::runtime_error("I 95 To I 695 failed to be parsed.");

  exitsigns.clear();
  node.set_exit_to_index(osmdata.node_names.index("I 495 Toward I 270"));

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 2) {
    if (exitsigns[0].type() != Sign::Type::kExitBranch)
      throw std::runtime_error("I 495 should be a branch.");

    if (exitsigns[1].type() != Sign::Type::kExitToward)
      throw std::runtime_error("I 270 should be a toward.");

    if (exitsigns[0].text() != "I 495" && exitsigns[1].text() != "I 270")
      throw std::runtime_error("Exitsign text is bad for I 495 Toward I 270");
  } else
    throw std::runtime_error("I 495 Toward I 270 failed to be parsed.");

  exitsigns.clear();
  node.set_exit_to_index(
      osmdata.node_names.index("I 495 Toward I 270 To I 95")); // default to toward.  Punt on parsing.

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 1) {
    if (exitsigns[0].type() != Sign::Type::kExitToward)
      throw std::runtime_error("I 495 Toward I 270 To I 95 should be a toward.");

    if (exitsigns[0].text() != "I 495 Toward I 270 To I 95")
      throw std::runtime_error("Exitsign text is bad for I 495 Toward I 270 To I 95");
  } else
    throw std::runtime_error("I 495 Toward I 270 To I 95 failed to be parsed.");

  // Add a ref branch sign
  exitsigns.clear();
  auto index = osmdata.name_offset_map.index("I 495 North");
  way.set_destination_ref_index(index);
  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);
  if (exitsigns.size() == 1) {
    if (exitsigns[0].type() != Sign::Type::kExitBranch)
      throw std::runtime_error("I 495 North should be a branch.");

    if (!exitsigns[0].is_route_num())
      throw std::runtime_error("I 495 North should be flagged as a route num");

    if (exitsigns[0].text() != "I 495 North")
      throw std::runtime_error("Exitsign text is bad for I 495 North destination ref");
  } else {
    throw std::runtime_error("destination ref I 495 North failed to create exist sign.");
  }

  // Add a ref toward sign
  OSMWay way2{};
  exitsigns.clear();
  auto index2 = osmdata.name_offset_map.index("I 695 North");
  way2.set_destination_ref_to_index(index2);
  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way2, osmdata, fork, forward);
  if (exitsigns.size() == 1) {
    if (exitsigns[0].type() != Sign::Type::kExitToward)
      throw std::runtime_error("I 695 North should be a toward.");

    if (!exitsigns[0].is_route_num())
      throw std::runtime_error("I 695 North should be flagged as a route num");

    if (exitsigns[0].text() != "I 695 North")
      throw std::runtime_error("Exitsign text is bad for I 695 North destination to ref");
  } else {
    throw std::runtime_error("destination ref I 695 North failed to create exist sign.");
  }
}

} // namespace

int main() {
  test::suite suite("signinfo");

  // Test exit to logic.
  suite.test(TEST_CASE(ExitToTest));

  return suite.tear_down();
}
