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
  using GraphTileBuilder::edge_offset_map_;
  using GraphTileBuilder::EdgeTupleHasher;
  using GraphTileBuilder::EdgeTuple;

};


void TestDuplicateEdgeInfo() {
  edge_tuple a = test_graph_tile_builder::EdgeTuple(0, GraphId(0,2,0), GraphId(0,2,1));
  edge_tuple b = test_graph_tile_builder::EdgeTuple(0, GraphId(0,2,0), GraphId(0,2,1));
  if(a != b || !(a == b))
    throw std::runtime_error("Edge tuples should be equivalent");
  std::unordered_map<edge_tuple, size_t, test_graph_tile_builder::EdgeTupleHasher> m;
  m.emplace(a, 0);
  if(m.size() != 1)
    throw std::runtime_error("Why isnt there an item in this map");
  if(m.find(a) == m.end())
    throw std::runtime_error("We should have been able to find the edge tuple");
  const auto success = m.emplace(b, 1);
  if(success.second)
    throw std::runtime_error("Why on earth would it be found but then insert just fine");


  test_graph_tile_builder test;
  //add edge info for node 0 to node 1
  bool added = false;
  test.AddEdgeInfo(0, GraphId(0,2,0), GraphId(0,2,1), 1234, {{0, 0}, {1, 1}}, {"einzelweg"}, added);
  if(test.edge_offset_map_.size() != 1)
    throw std::runtime_error("There should be exactly one of these in here");
  //add edge info for node 1 to node 0
  test.AddEdgeInfo(0, GraphId(0,2,1), GraphId(0,2,0), 1234, {{1, 1}, {0, 0}}, {"einzelweg"}, added);
  if(test.edge_offset_map_.size() != 1)
    throw std::runtime_error("There should still be exactly one of these in here");
}

}

int main() {
  test::suite suite("graphtilebuilder");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestDuplicateEdgeInfo));

  return suite.tear_down();
}
