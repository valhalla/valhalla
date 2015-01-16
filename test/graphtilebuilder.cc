#include "test.h"

#include "mjolnir/graphtilebuilder.h"
#include <valhalla/baldr/graphid.h>
#include <valhalla/midgard/pointll.h>
#include <string>
#include <vector>
using namespace std;
using namespace valhalla::mjolnir;

namespace {

class test_graph_tile_builder : public GraphTileBuilder {
 public:
  std::unordered_map<edge_tuple, size_t, GraphTileBuilder::EdgeTupleHasher> edge_offset_map;
};

void TestDuplicateEdgeInfo() {
  test_graph_tile_builder test;
  //add edge info for node 0 to node 1
  test.AddEdgeInfo(0, GraphId(0,2,0), GraphId(0,2,1), {{0, 0}, {1, 1}}, {"einzelweg"});
  //add edge info for node 1 to node 0
  test.AddEdgeInfo(0, GraphId(0,2,1), GraphId(0,2,0), {{1, 1}, {0, 0}}, {"einzelweg"});
  if(test.edge_offset_map.size() > 1)
    throw std::runtime_error("Should not allow duplicate edgeinfos (ie the shared edgeinfo for 2 opposing directed edges)");
}

}

int main() {
  test::suite suite("edgeinfobuilder");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestDuplicateEdgeInfo));

  return suite.tear_down();
}
