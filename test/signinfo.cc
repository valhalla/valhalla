#include "baldr/graphid.h"
#include "baldr/signinfo.h"
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

  node.set_exit_to(true);


  osmdata.node_exit_to[node.osmid] = "US 11;To I 81;Carlisle;Harrisburg";

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
  }
  else throw std::runtime_error("US 11/To I 81/Carlisle/Harrisburg failed to be parsed.  " + std::to_string(exitsigns.size()) );

  exitsigns.clear();
  osmdata.node_exit_to[node.osmid] = "US 11;Toward I 81;Carlisle;Harrisburg";

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 4) {
    for (auto& exitsign : exitsigns) {
      if (exitsign.type() != Sign::Type::kExitToward)
        throw std::runtime_error("US 11;Toward I 81;Carlisle;Harrisburg types are not all Toward");
    }
    if (exitsigns[0].text() != "US 11" && exitsigns[1].text() != "I 81" &&
        exitsigns[2].text() != "Carlisle" && exitsigns[3].text() != "Harrisburg")
      throw std::runtime_error("Exitsign text is bad for US 11;To I 81;Carlisle;Harrisburg.");
  }
  else throw std::runtime_error("US 11;Toward I 81;Carlisle;Harrisburg failed to be parsed.");

  exitsigns.clear();
  osmdata.node_exit_to[node.osmid] = "I 95 To I 695";

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 2) {
     if (exitsigns[0].type() != Sign::Type::kExitBranch)
       throw std::runtime_error("I 95 should be a branch.");

     if (exitsigns[1].type() != Sign::Type::kExitToward)
       throw std::runtime_error("I 695 should be a toward.");

     if (exitsigns[0].text() != "I 95" && exitsigns[1].text() != "I 695")
       throw std::runtime_error("Exitsign text is bad for I 95 To I 695");
  }
  else throw std::runtime_error("I 95 To I 695 failed to be parsed.");

  exitsigns.clear();
  osmdata.node_exit_to[node.osmid] = "I 495 Toward I 270";

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 2) {
    if (exitsigns[0].type() != Sign::Type::kExitBranch)
      throw std::runtime_error("I 495 should be a branch.");

    if (exitsigns[1].type() != Sign::Type::kExitToward)
      throw std::runtime_error("I 270 should be a toward.");

    if (exitsigns[0].text() != "I 495" && exitsigns[1].text() != "I 270")
      throw std::runtime_error("Exitsign text is bad for I 495 Toward I 270");
  }
  else throw std::runtime_error("I 495 Toward I 270 failed to be parsed.");

  exitsigns.clear();
  osmdata.node_exit_to[node.osmid] = "I 495 Toward I 270 To I 95";//default to toward.  Punt on parsing.

  exitsigns = GraphBuilder::CreateExitSignInfoList(node, way, osmdata, fork, forward);

  if (exitsigns.size() == 1) {
    if (exitsigns[0].type() != Sign::Type::kExitToward)
      throw std::runtime_error("I 495 Toward I 270 To I 95 should be a toward.");

    if (exitsigns[0].text() != "I 495 Toward I 270 To I 95")
      throw std::runtime_error("Exitsign text is bad for I 495 Toward I 270 To I 95");
  }
  else throw std::runtime_error("I 495 Toward I 270 To I 95 failed to be parsed.");
}

}

int main() {
  test::suite suite("signinfo");

  // Test exit to logic.
  suite.test(TEST_CASE(ExitToTest));

  return suite.tear_down();
}
