#include "test.h"

#include "mjolnir/graphtilebuilder.h"
#include <valhalla/baldr/graphid.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <string>
#include <vector>
using namespace std;
using namespace valhalla::mjolnir;

namespace {

class test_graph_tile_builder : public GraphTileBuilder {
 public:
  using GraphTileBuilder::GraphTileBuilder;
  using GraphTileBuilder::edge_offset_map_;
  using GraphTileBuilder::EdgeTupleHasher;
  using GraphTileBuilder::EdgeTuple;

};

bool tile_equalish(const GraphTile a, const GraphTile b, size_t difference, const std::array<std::vector<GraphId>, kBinCount>& bins) {
  //expected size
  if(a.size() + difference != b.size())
    return false;

  //check the first chunk after the header
  if(memcmp(reinterpret_cast<const char*>(a.header()) + sizeof(GraphTileHeader),
      reinterpret_cast<const char*>(b.header()) + sizeof(GraphTileHeader),
      (reinterpret_cast<const char*>(b.GetBin(0, 0).begin()) - reinterpret_cast<const char*>(b.header())) - sizeof(GraphTileHeader)))
    return false;

  //check the stuff after the bins
  if(memcmp(reinterpret_cast<const char*>(a.header()) + a.header()->edgeinfo_offset(),
      reinterpret_cast<const char*>(b.header()) + b.header()->edgeinfo_offset(),
      b.size() - b.header()->edgeinfo_offset()))
    return false;

  //if the header is as expected
  const auto* ah = a.header(), *bh = b.header();
  if(ah->access_restriction_count() == bh->access_restriction_count() &&
     ah->admincount() == bh->admincount() &&
     ah->complex_restriction_offset() == bh->complex_restriction_offset() &&
     ah->date_created() == bh->date_created() &&
     ah->density() == bh->density() &&
     ah->departurecount() == bh->departurecount() &&
     ah->directededgecount() == bh->directededgecount() &&
     ah->edgeinfo_offset() + difference == bh->edgeinfo_offset() &&
     ah->exit_quality() == bh->exit_quality() &&
     ah->graphid() == bh->graphid() &&
     ah->name_quality() == bh->name_quality() &&
     ah->nodecount() == bh->nodecount() &&
     ah->routecount() == bh->routecount() &&
     ah->signcount() == bh->signcount() &&
     ah->speed_quality() == bh->speed_quality() &&
     ah->stopcount() == bh->stopcount() &&
     ah->textlist_offset() + difference== bh->textlist_offset() &&
     ah->transfercount() == bh->transfercount() &&
     ah->version() ==  bh->version()) {
    //make sure the edges' shape and names match
    for(size_t i = 0; i < ah->directededgecount(); ++i) {
      auto a_info = a.edgeinfo(a.directededge(i)->edgeinfo_offset());
      auto b_info = b.edgeinfo(b.directededge(i)->edgeinfo_offset());
      if(a_info->encoded_shape() != b_info->encoded_shape())
        return false;
      if(a_info->GetNames().size() != b_info->GetNames().size())
        return false;
      for(size_t j = 0; j < a_info->GetNames().size(); ++j)
        if(a_info->GetNames()[j] != b_info->GetNames()[j])
          return false;
    }
    //check that the bins contain what was just added to them
    for(size_t i = 0; i < bins.size(); ++i) {
      auto bin = b.GetBin(i % kBinsDim, i / kBinsDim);
      for(size_t j = 0; j < bins[i].size(); ++j) {
        if(bin[j] != bins[i][j])
          return false;
      }
    }
    return true;
  }
  return false;
}

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

  //load a test builder
  test_graph_tile_builder test(TileHierarchy("test/tiles"), GraphId(0,2,0), false);
  //add edge info for node 0 to node 1
  bool added = false;
  test.AddEdgeInfo(0, GraphId(0,2,0), GraphId(0,2,1), 1234, std::list<PointLL>{{0, 0}, {1, 1}}, {"einzelweg"}, added);
  if(test.edge_offset_map_.size() != 1)
    throw std::runtime_error("There should be exactly one of these in here");
  //add edge info for node 1 to node 0
  test.AddEdgeInfo(0, GraphId(0,2,1), GraphId(0,2,0), 1234, std::list<PointLL>{{1, 1}, {0, 0}}, {"einzelweg"}, added);
  if(test.edge_offset_map_.size() != 1)
    throw std::runtime_error("There should still be exactly one of these in here");
}

void TestAddBins() {

  //test a couple of tiles
  for(const auto& test_tile : std::list<std::pair<std::string, size_t> >
      {
        {"609/453.gph", 609453},
        {"762/161.gph", 762161}
      }) {

    //load a tile
    GraphId id(test_tile.second,2,0);
    GraphTile t(TileHierarchy("test/tiles/no_bin"), id);

    //alter the config to point to another dir
    TileHierarchy h("test/tiles/bin");

    //send blank bins
    std::array<std::vector<GraphId>, kBinCount> bins;
    GraphTileBuilder::AddBins(h, &t, bins);

    //check the new tile is the same as the old one
    {
      ifstream o("test/tile/no_bin/2/000/" + test_tile.first, std::ios::binary);
      ifstream n("test/tile/bin/2/000/" + test_tile.first, std::ios::binary);
      char o_c, n_c;
      while(o.read(&o_c, 1) && n.read(&n_c, 1) && !o.eof() && !o.fail() && !n.eof() && !n.fail() && o_c == n_c);
      if(o.eof() != n.eof() || o.fail() != n.fail())
        throw std::logic_error("Old tile and new tile should be the same if not adding any bins");
    }

    //send fake bins, we'll throw one in each bin
    for(auto& bin : bins)
      bin.emplace_back(test_tile.second,2,0);
    GraphTileBuilder::AddBins(h, &t, bins);
    auto increase = bins.size() * sizeof(GraphId);

    //check the new tile isnt broken and is exactly the right size bigger
    if(!tile_equalish(t, GraphTile(h, id), increase, bins))
      throw std::logic_error("New tiles edgeinfo or names arent matching up");

    //append some more
    for(auto& bin : bins)
      bin.emplace_back(test_tile.second,2,1);
    GraphTileBuilder::AddBins(h, &t, bins);
    increase = bins.size() * sizeof(GraphId) * 2;

    //check the new tile isnt broken and is exactly the right size bigger
    if(!tile_equalish(t, GraphTile(h, id), increase, bins))
      throw std::logic_error("New tiles edgeinfo or names arent matching up");

    //check that appending works
    t = GraphTile(TileHierarchy("test/tiles/bin"), id);
    GraphTileBuilder::AddBins(h, &t, bins);
    for(auto& bin : bins)
      bin.insert(bin.end(), bin.begin(), bin.end());

    //check the new tile isnt broken and is exactly the right size bigger
    if(!tile_equalish(t, GraphTile(h, id), increase, bins))
      throw std::logic_error("New tiles edgeinfo or names arent matching up");
  }
}

}

int main() {
  test::suite suite("graphtilebuilder");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestDuplicateEdgeInfo));

  // Add bins to a tile and see if its still ok
  suite.test(TEST_CASE(TestAddBins));

  return suite.tear_down();
}
