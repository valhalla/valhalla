#include "test.h"

#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"
#include <fstream>
#include <streambuf>
#include <string>
#include <vector>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace std;
using namespace valhalla::mjolnir;

namespace {

class test_graph_tile_builder : public GraphTileBuilder {
public:
  using GraphTileBuilder::edge_offset_map_;
  using GraphTileBuilder::EdgeTuple;
  using GraphTileBuilder::EdgeTupleHasher;
  using GraphTileBuilder::GraphTileBuilder;
};

bool tile_equalish(const GraphTile a,
                   const GraphTile b,
                   size_t difference,
                   const std::array<std::vector<GraphId>, kBinCount>& bins) {
  // expected size
  if (a.header()->end_offset() + difference != b.header()->end_offset())
    return false;

  // check the first chunk after the header
  if (memcmp(reinterpret_cast<const char*>(a.header()) + sizeof(GraphTileHeader),
             reinterpret_cast<const char*>(b.header()) + sizeof(GraphTileHeader),
             (reinterpret_cast<const char*>(b.GetBin(0, 0).begin()) -
              reinterpret_cast<const char*>(b.header())) -
                 sizeof(GraphTileHeader)))
    return false;

  // check the stuff after the bins
  if (memcmp(reinterpret_cast<const char*>(a.header()) + a.header()->edgeinfo_offset(),
             reinterpret_cast<const char*>(b.header()) + b.header()->edgeinfo_offset(),
             b.header()->end_offset() - b.header()->edgeinfo_offset()))
    return false;

  // if the header is as expected
  const auto *ah = a.header(), *bh = b.header();
  if (ah->access_restriction_count() == bh->access_restriction_count() &&
      ah->admincount() == bh->admincount() &&
      ah->complex_restriction_forward_offset() + difference ==
          bh->complex_restriction_forward_offset() &&
      ah->complex_restriction_reverse_offset() + difference ==
          bh->complex_restriction_reverse_offset() &&
      ah->date_created() == bh->date_created() && ah->density() == bh->density() &&
      ah->departurecount() == bh->departurecount() &&
      ah->directededgecount() == bh->directededgecount() &&
      ah->edgeinfo_offset() + difference == bh->edgeinfo_offset() &&
      ah->exit_quality() == bh->exit_quality() && ah->graphid() == bh->graphid() &&
      ah->name_quality() == bh->name_quality() && ah->nodecount() == bh->nodecount() &&
      ah->routecount() == bh->routecount() && ah->signcount() == bh->signcount() &&
      ah->speed_quality() == bh->speed_quality() && ah->stopcount() == bh->stopcount() &&
      ah->textlist_offset() + difference == bh->textlist_offset() &&
      ah->schedulecount() == bh->schedulecount() && ah->version() == bh->version()) {
    // make sure the edges' shape and names match
    for (size_t i = 0; i < ah->directededgecount(); ++i) {
      auto a_info = a.edgeinfo(a.directededge(i)->edgeinfo_offset());
      auto b_info = b.edgeinfo(b.directededge(i)->edgeinfo_offset());
      if (a_info.encoded_shape() != b_info.encoded_shape())
        return false;
      if (a_info.GetNames().size() != b_info.GetNames().size())
        return false;
      for (size_t j = 0; j < a_info.GetNames().size(); ++j)
        if (a_info.GetNames()[j] != b_info.GetNames()[j])
          return false;
    }
    // check that the bins contain what was just added to them
    for (size_t i = 0; i < bins.size(); ++i) {
      auto bin = b.GetBin(i % kBinsDim, i / kBinsDim);
      auto offset = bin.size() - bins[i].size();
      for (size_t j = 0; j < bins[i].size(); ++j) {
        if (bin[j + offset] != bins[i][j])
          return false;
      }
    }
    return true;
  }
  return false;
}

void TestDuplicateEdgeInfo() {
  edge_tuple a = test_graph_tile_builder::EdgeTuple(0, GraphId(0, 2, 0), GraphId(0, 2, 1));
  edge_tuple b = test_graph_tile_builder::EdgeTuple(0, GraphId(0, 2, 0), GraphId(0, 2, 1));
  if (a != b || !(a == b))
    throw std::runtime_error("Edge tuples should be equivalent");
  std::unordered_map<edge_tuple, size_t, test_graph_tile_builder::EdgeTupleHasher> m;
  m.emplace(a, 0);
  if (m.size() != 1)
    throw std::runtime_error("Why isnt there an item in this map");
  if (m.find(a) == m.end())
    throw std::runtime_error("We should have been able to find the edge tuple");
  const auto success = m.emplace(b, 1);
  if (success.second)
    throw std::runtime_error("Why on earth would it be found but then insert just fine");

  // load a test builder
  std::string test_dir = "test/data/builder_tiles";
  test_graph_tile_builder test(test_dir, GraphId(0, 2, 0), false);
  // add edge info for node 0 to node 1
  bool added = false;
  test.AddEdgeInfo(0, GraphId(0, 2, 0), GraphId(0, 2, 1), 1234, 555, 0, 120,
                   std::list<PointLL>{{0, 0}, {1, 1}}, {"einzelweg"}, 0, added);
  if (test.edge_offset_map_.size() != 1)
    throw std::runtime_error("There should be exactly one of these in here");
  // add edge info for node 1 to node 0
  test.AddEdgeInfo(0, GraphId(0, 2, 1), GraphId(0, 2, 0), 1234, 555, 0, 120,
                   std::list<PointLL>{{1, 1}, {0, 0}}, {"einzelweg"}, 0, added);
  if (test.edge_offset_map_.size() != 1)
    throw std::runtime_error("There should still be exactly one of these in here");

  test.StoreTileData();
  test_graph_tile_builder test2(test_dir, GraphId(0, 2, 0), false);
  auto ei = test2.edgeinfo(0);
  if (std::abs(ei.mean_elevation() - 555.0f) > kElevationBinSize) {
    throw std::runtime_error("Mean elevation test failed " + std::to_string(ei.mean_elevation()));
  }
  if (ei.speed_limit() != 120) {
    throw std::runtime_error("Speed limit test failed " + std::to_string(ei.speed_limit()));
  }
}

void TestAddBins() {

  // if you update the tile format you must regenerate test tiles. after your tile format change,
  // run valhalla_build_tiles on a reasonable sized extract. when its done do the following:
  /*
    git rm -rf test/data/bin_tiles/no_bin
    for f in $(find /data/valhalla/2 -printf '%s %P\n'| sort -n | head -n 2 | awk '{print $2}'); do
      mkdir -p test/data/bin_tiles/no_bin/2/$(dirname ${f})
      cp -rp /data/valhalla/2/${f} test/data/bin_tiles/no_bin/2/${f}
    done
    git add test/data/bin_tiles/no_bin
    git status
   */
  // this will grab the 2 smallest tiles from you new tile set and make them the new test tiles
  // note the names of the new tiles and update the list with path and index in the list just below
  for (const auto& test_tile :
       std::list<std::pair<std::string, size_t>>{{"744/881.gph", 744881}, {"744/885.gph", 744885}}) {

    // load a tile
    GraphId id(test_tile.second, 2, 0);
    std::string no_bin_dir = VALHALLA_SOURCE_DIR "test/data/bin_tiles/no_bin";
    GraphTile t(no_bin_dir, id);
    if (!t.header())
      throw std::runtime_error("Couldn't load test tile");

    // alter the config to point to another dir
    std::string bin_dir = "test/data/bin_tiles/bin";

    // send blank bins
    std::array<std::vector<GraphId>, kBinCount> bins;
    GraphTileBuilder::AddBins(bin_dir, &t, bins);

    // check the new tile is the same as the old one
    {
      ifstream o;
      o.exceptions(std::ifstream::failbit | std::ifstream::badbit);
      o.open(VALHALLA_SOURCE_DIR "test/data/bin_tiles/no_bin/2/000/" + test_tile.first,
             std::ios::binary);
      std::string obytes((std::istreambuf_iterator<char>(o)), std::istreambuf_iterator<char>());
      ifstream n;
      n.exceptions(std::ifstream::failbit | std::ifstream::badbit);
      n.open("test/data/bin_tiles/bin/2/000/" + test_tile.first, std::ios::binary);
      std::string nbytes((std::istreambuf_iterator<char>(n)), std::istreambuf_iterator<char>());
      if (obytes != nbytes)
        throw std::logic_error("Old tile and new tile should be the same if not adding any bins");
    }

    // send fake bins, we'll throw one in each bin
    for (auto& bin : bins)
      bin.emplace_back(test_tile.second, 2, 0);
    GraphTileBuilder::AddBins(bin_dir, &t, bins);
    auto increase = bins.size() * sizeof(GraphId);

    // check the new tile isnt broken and is exactly the right size bigger
    if (!tile_equalish(t, GraphTile(bin_dir, id), increase, bins))
      throw std::logic_error("New tiles edgeinfo or names arent matching up: 1");

    // append some more
    for (auto& bin : bins)
      bin.emplace_back(test_tile.second, 2, 1);
    GraphTileBuilder::AddBins(bin_dir, &t, bins);
    increase = bins.size() * sizeof(GraphId) * 2;

    // check the new tile isnt broken and is exactly the right size bigger
    if (!tile_equalish(t, GraphTile(bin_dir, id), increase, bins))
      throw std::logic_error("New tiles edgeinfo or names arent matching up: 2");

    // check that appending works
    t = GraphTile(bin_dir, id);
    GraphTileBuilder::AddBins(bin_dir, &t, bins);
    for (auto& bin : bins)
      bin.insert(bin.end(), bin.begin(), bin.end());

    // check the new tile isnt broken and is exactly the right size bigger
    if (!tile_equalish(t, GraphTile(bin_dir, id), increase, bins))
      throw std::logic_error("New tiles edgeinfo or names arent matching up: 3");
  }
}

struct fake_tile : public GraphTile {
public:
  fake_tile(const std::string& plyenc_shape) {
    auto s = valhalla::midgard::decode<std::vector<PointLL>>(plyenc_shape);
    auto e = valhalla::midgard::encode7(s);
    auto l = TileHierarchy::levels().rbegin()->first;
    auto tiles = TileHierarchy::levels().rbegin()->second.tiles;
    auto id = GraphId(tiles.TileId(s.front()), l, 0);
    auto o_id = GraphId(tiles.TileId(s.front()), l, tiles.TileId(s.back()));
    o_id.set_id(o_id == id);
    header_ = new GraphTileHeader();
    header_->set_graphid(id);
    header_->set_directededgecount(1 + (id.tileid() == o_id.tileid()) * 1);

    edgeinfo_ = new char[sizeof(uint64_t) + sizeof(EdgeInfo::PackedItem) + e.size()];
    std::memset(static_cast<void*>(edgeinfo_), 0, sizeof(uint64_t));
    EdgeInfo::PackedItem pi{0, static_cast<uint32_t>(e.size())};
    std::memcpy(static_cast<void*>(edgeinfo_ + sizeof(uint64_t)), static_cast<void*>(&pi),
                sizeof(EdgeInfo::PackedItem));
    textlist_ = edgeinfo_;
    textlist_size_ = 0;
    std::memcpy(static_cast<void*>(edgeinfo_ + sizeof(uint64_t) + sizeof(EdgeInfo::PackedItem)),
                static_cast<void*>(&e[0]), e.size());

    directededges_ = new DirectedEdge[2];
    std::memset(static_cast<void*>(&directededges_[0]), 0,
                sizeof(DirectedEdge) * header_->directededgecount());
    directededges_[0].set_forward(true);
  }
  ~fake_tile() {
    delete header_;
    delete[] edgeinfo_;
    delete[] directededges_;
  }
};

void TestBinEdges() {
  fake_tile fake(
      "gsoyLcpczmFgJOsMzAwGtDmDtEmApG|@tE|EdF~PjKlRjLbKhLrJnTdD`\\oEz`@wAlJKjVnHfMpRbQdQbRvTtNrM~"
      "ShNdZ|HjLfCbPfIbGdNxBjOyBjPOnJm@rDvD~BbFxFzA|IjAdEdFy@tOqBbPv@`HfHj`@"
      "pGxMtNbFlUjBtNvMhLbQfOxLhNzVTrFkGhMsQ|J_N{AqKkBqRxCoZpGu^jLqMz@sQwCmPmJwLgNcHePwIqG{"
      "KOoLvCsLbGeUpGm`@l@}_@jBeZmA}[aQs_@gd@gOyVosAqf@gYkLub@m@{LgCkFz@sBvDKrEjApH~Er[hOha@hIfc@i@"
      "pHaIrPyMng@mF~^kA~\\h@zVtCnHrFzAlUtE~@hBv@rFqBb\\?xVdEjV}@hM?tOvClJmAdEwCvD_b@|SuKbGiGpH_"
      "JtOsOvXyBvMbBtOlAtO}EdF}GjA_QhBiKjBsHxLoGtYkGdd@yJbf@fAxWvEtO]fCcFl@{ZlJcKnI{@lJtDpH`I|J~"
      "GbFtG|@hCz@MtEoCxB_QpGmKdE_KtPyAdO~BvNUdEyBxB_HjB{NxBwOxAuHrFwF~IXjLbDjKbKbFnIzArBvDwAdE{"
      "GtEyE|IElJ~B`H_AvCyDjBgH?mRgDy`@]{OtDaKxLiCvNiFjLgMxL_XdPgr@re@yi@vb@mWrQuFjKqHnSuH|"
      "JiGlJqAfDbAtE~A~GgBdFyKlJy[xWqMvMqTlKoPfCeKiBkHeF}E{KoD{JaLsGwSeEg~BqR");
  GraphTileBuilder::tweeners_t tweeners;
  auto bins = GraphTileBuilder::BinEdges(&fake, tweeners);
  if (tweeners.size() != 1)
    throw std::logic_error("This edge leaves a tile for 1 other tile and comes back.");
}

} // namespace

int main() {
  test::suite suite("graphtilebuilder");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestDuplicateEdgeInfo));

  // Add bins to a tile and see if its still ok
  suite.test(TEST_CASE(TestAddBins));

  // Test bin edges of some tricky edges
  suite.test(TEST_CASE(TestBinEdges));

  return suite.tear_down();
}
