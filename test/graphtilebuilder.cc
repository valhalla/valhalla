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

void assert_tile_equalish(const GraphTile& a,
                          const GraphTile& b,
                          size_t difference,
                          const std::array<std::vector<GraphId>, kBinCount>& bins,
                          const std::string& /*msg*/) {
  // expected size
  ASSERT_EQ(a.header()->end_offset() + difference, b.header()->end_offset());

  // check the first chunk after the header
  ASSERT_EQ(memcmp(reinterpret_cast<const char*>(a.header()) + sizeof(GraphTileHeader),
                   reinterpret_cast<const char*>(b.header()) + sizeof(GraphTileHeader),
                   (reinterpret_cast<const char*>(b.GetBin(0, 0).begin()) -
                    reinterpret_cast<const char*>(b.header())) -
                       sizeof(GraphTileHeader)),
            0);

  // check the stuff after the bins
  ASSERT_EQ(memcmp(reinterpret_cast<const char*>(a.header()) + a.header()->edgeinfo_offset(),
                   reinterpret_cast<const char*>(b.header()) + b.header()->edgeinfo_offset(),
                   b.header()->end_offset() - b.header()->edgeinfo_offset()),
            0);

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
      auto a_info = a.edgeinfo(a.directededge(i));
      auto b_info = b.edgeinfo(b.directededge(i));
      ASSERT_EQ(a_info.encoded_shape(), b_info.encoded_shape());
      ASSERT_EQ(a_info.GetNames().size(), b_info.GetNames().size());
      for (size_t j = 0; j < a_info.GetNames().size(); ++j)
        ASSERT_EQ(a_info.GetNames()[j], b_info.GetNames()[j]);
    }
    // check that the bins contain what was just added to them
    for (size_t i = 0; i < bins.size(); ++i) {
      auto bin = b.GetBin(i % kBinsDim, i / kBinsDim);
      auto offset = bin.size() - bins[i].size();
      for (size_t j = 0; j < bins[i].size(); ++j) {
        ASSERT_EQ(bin[j + offset], bins[i][j]);
      }
    }
  } else {
    FAIL() << "not equal";
  }
}

TEST(GraphTileBuilder, TestDuplicateEdgeInfo) {
  edge_tuple a = test_graph_tile_builder::EdgeTuple(0, GraphId(0, 2, 0), GraphId(0, 2, 1));
  edge_tuple b = test_graph_tile_builder::EdgeTuple(0, GraphId(0, 2, 0), GraphId(0, 2, 1));
  EXPECT_EQ(a, b);
  EXPECT_TRUE(a == b) << "Edge tuples should be equivalent";

  std::unordered_map<edge_tuple, size_t, test_graph_tile_builder::EdgeTupleHasher> m;
  m.emplace(a, 0);
  EXPECT_EQ(m.size(), 1) << "Why isnt there an item in this map";
  ASSERT_NE(m.find(a), m.end()) << "We should have been able to find the edge tuple";

  const auto success = m.emplace(b, 1);
  EXPECT_FALSE(success.second) << "Why on earth would it be found but then insert just fine";

  // load a test builder
  std::string test_dir = "test/data/builder_tiles";
  test_graph_tile_builder test(test_dir, GraphId(0, 2, 0), false);
  test.directededges().emplace_back();
  // add edge info for node 0 to node 1
  bool added = false;
  test.AddEdgeInfo(0, GraphId(0, 2, 0), GraphId(0, 2, 1), 1234, 555, 0, 120,
                   std::list<PointLL>{{0, 0}, {1, 1}}, {"einzelweg"}, {"1xyz tunnel"}, 0, added);
  EXPECT_EQ(test.edge_offset_map_.size(), 1) << "There should be exactly two of these in here";

  // add edge info for node 1 to node 0
  test.AddEdgeInfo(0, GraphId(0, 2, 1), GraphId(0, 2, 0), 1234, 555, 0, 120,
                   std::list<PointLL>{{1, 1}, {0, 0}}, {"einzelweg"}, {"1xyz tunnel"}, 0, added);
  EXPECT_EQ(test.edge_offset_map_.size(), 1) << "There should still be exactly two of these in here";

  test.StoreTileData();
  test_graph_tile_builder test2(test_dir, GraphId(0, 2, 0), false);
  auto ei = test2.edgeinfo(&test2.directededge(0));
  EXPECT_NEAR(ei.mean_elevation(), 555.0f, kElevationBinSize);
  EXPECT_EQ(ei.speed_limit(), 120);

  auto n1 = ei.GetNames();
  EXPECT_EQ(n1.size(), 1);
  EXPECT_EQ(n1.at(0), "einzelweg");

  auto n2 = ei.GetNames(); // defaults to false
  EXPECT_EQ(n2.size(), 1);
  EXPECT_EQ(n2.at(0), "einzelweg");

  auto n3 = ei.GetTaggedValues();
  EXPECT_EQ(n3.size(), 1);
  EXPECT_EQ(n3.at(0), "1xyz tunnel"); // we always return the tag type in getnames

  auto names_and_types = ei.GetNamesAndTypes(true);
  EXPECT_EQ(names_and_types.size(), 2);

  auto n4 = names_and_types.at(0);
  EXPECT_EQ(n4.first, "einzelweg");
  EXPECT_EQ(n4.second, false);

  auto n5 = names_and_types.at(1);
  EXPECT_EQ(n5.first, "xyz tunnel"); // no tag type in GetNamesAndTypes
  EXPECT_EQ(n5.second, false);

  names_and_types = ei.GetNamesAndTypes(false);
  EXPECT_EQ(names_and_types.size(), 1);

  n4 = names_and_types.at(0);
  EXPECT_EQ(n4.first, "einzelweg");
  EXPECT_EQ(n4.second, false);

  names_and_types = ei.GetNamesAndTypes(); // defaults to false
  EXPECT_EQ(names_and_types.size(), 1);

  n4 = names_and_types.at(0);
  EXPECT_EQ(n4.first, "einzelweg");
  EXPECT_EQ(n4.second, false);

  const auto& tags = ei.GetTags();
  EXPECT_EQ(tags.size(), 1);
  EXPECT_EQ(tags.find(TaggedValue::kTunnel)->second, "xyz tunnel");
}

TEST(GraphTileBuilder, TestAddBins) {

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
    auto t = GraphTile::Create(no_bin_dir, id);
    ASSERT_TRUE(t && t->header()) << "Couldn't load test tile";

    // alter the config to point to another dir
    std::string bin_dir = "test/data/bin_tiles/bin";

    // send blank bins
    std::array<std::vector<GraphId>, kBinCount> bins;
    GraphTileBuilder::AddBins(bin_dir, t, bins);

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
      EXPECT_EQ(obytes, nbytes) << "Old tile and new tile should be the same if not adding any bins";
    }

    // send fake bins, we'll throw one in each bin
    for (auto& bin : bins)
      bin.emplace_back(test_tile.second, 2, 0);
    GraphTileBuilder::AddBins(bin_dir, t, bins);
    auto increase = bins.size() * sizeof(GraphId);

    // check the new tile isnt broken and is exactly the right size bigger
    assert_tile_equalish(*t, *GraphTile::Create(bin_dir, id), increase, bins,
                         "New tiles edgeinfo or names arent matching up: 1");

    // append some more
    for (auto& bin : bins)
      bin.emplace_back(test_tile.second, 2, 1);
    GraphTileBuilder::AddBins(bin_dir, t, bins);
    increase = bins.size() * sizeof(GraphId) * 2;

    // check the new tile isnt broken and is exactly the right size bigger
    assert_tile_equalish(*t, *GraphTile::Create(bin_dir, id), increase, bins,
                         "New tiles edgeinfo or names arent matching up: 2");

    // check that appending works
    t = GraphTile::Create(bin_dir, id);
    GraphTileBuilder::AddBins(bin_dir, t, bins);
    for (auto& bin : bins)
      bin.insert(bin.end(), bin.begin(), bin.end());

    // check the new tile isnt broken and is exactly the right size bigger
    assert_tile_equalish(*t, *GraphTile::Create(bin_dir, id), increase, bins,
                         "New tiles edgeinfo or names arent matching up: 3");
  }
}

struct fake_tile : public GraphTile {
public:
  fake_tile(const std::string& plyenc_shape) {
    auto s = valhalla::midgard::decode<std::vector<PointLL>>(plyenc_shape);
    auto e = valhalla::midgard::encode7(s);
    auto l = TileHierarchy::levels().back().level;
    auto tiles = TileHierarchy::levels().back().tiles;
    auto id = GraphId(tiles.TileId(s.front()), l, 0);
    auto o_id = GraphId(tiles.TileId(s.front()), l, tiles.TileId(s.back()));
    o_id.set_id(o_id == id);
    header_ = new GraphTileHeader();
    header_->set_graphid(id);
    header_->set_directededgecount(1 + (id.tileid() == o_id.tileid()) * 1);

    auto ei_size = sizeof(EdgeInfo::EdgeInfoInner) + e.size();
    edgeinfo_ = new char[ei_size];
    EdgeInfo::EdgeInfoInner pi{0, 0, 0, 0, 0, 0, static_cast<uint32_t>(e.size())};
    std::memcpy(static_cast<void*>(edgeinfo_), static_cast<void*>(&pi),
                sizeof(EdgeInfo::EdgeInfoInner));
    textlist_ = edgeinfo_;
    textlist_size_ = 0;
    std::memcpy(static_cast<void*>(edgeinfo_ + sizeof(EdgeInfo::EdgeInfoInner)),
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

TEST(GraphTileBuilder, TestBinEdges) {
  std::string encoded_shape5 =
      "gsoyLcpczmFgJOsMzAwGtDmDtEmApG|@tE|EdF~PjKlRjLbKhLrJnTdD`\\oEz`@wAlJKjVnHfMpRbQdQbRvTtNrM~"
      "ShNdZ|HjLfCbPfIbGdNxBjOyBjPOnJm@rDvD~BbFxFzA|IjAdEdFy@tOqBbPv@`HfHj`@"
      "pGxMtNbFlUjBtNvMhLbQfOxLhNzVTrFkGhMsQ|J_N{AqKkBqRxCoZpGu^jLqMz@sQwCmPmJwLgNcHePwIqG{"
      "KOoLvCsLbGeUpGm`@l@}_@jBeZmA}[aQs_@gd@gOyVosAqf@gYkLub@m@{LgCkFz@sBvDKrEjApH~Er[hOha@hIfc@i@"
      "pHaIrPyMng@mF~^kA~\\h@zVtCnHrFzAlUtE~@hBv@rFqBb\\?xVdEjV}@hM?tOvClJmAdEwCvD_b@|SuKbGiGpH_"
      "JtOsOvXyBvMbBtOlAtO}EdF}GjA_QhBiKjBsHxLoGtYkGdd@yJbf@fAxWvEtO]fCcFl@{ZlJcKnI{@lJtDpH`I|J~"
      "GbFtG|@hCz@MtEoCxB_QpGmKdE_KtPyAdO~BvNUdEyBxB_HjB{NxBwOxAuHrFwF~IXjLbDjKbKbFnIzArBvDwAdE{"
      "GtEyE|IElJ~B`H_AvCyDjBgH?mRgDy`@]{OtDaKxLiCvNiFjLgMxL_XdPgr@re@yi@vb@mWrQuFjKqHnSuH|"
      "JiGlJqAfDbAtE~A~GgBdFyKlJy[xWqMvMqTlKoPfCeKiBkHeF}E{KoD{JaLsGwSeEg~BqR";
  auto decoded_shape = valhalla::midgard::decode<std::vector<PointLL>>(encoded_shape5);
  auto encoded_shape7 = valhalla::midgard::encode7(decoded_shape);
  graph_tile_ptr fake{new fake_tile(encoded_shape5)};
  auto info = fake->edgeinfo(fake->directededge(0));
  EXPECT_EQ(info.encoded_shape(), encoded_shape7);
  GraphTileBuilder::tweeners_t tweeners;
  auto bins = GraphTileBuilder::BinEdges(fake, tweeners);
  EXPECT_EQ(tweeners.size(), 1) << "This edge leaves a tile for 1 other tile and comes back.";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
