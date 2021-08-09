#include "loki/node_search.h"
#include <cstdint>

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>
#include <unordered_set>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/tilehierarchy.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"

#include "test.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

#include "filesystem.h"
#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"

namespace vj = valhalla::mjolnir;

namespace {

const std::string test_tile_dir = "test/node_search_tiles";

struct graph_writer {
  graph_writer(uint8_t level);

  vj::GraphTileBuilder& builder(vb::GraphId tile_id);

  inline vm::PointLL node_latlng(vb::GraphId node_id) {
    auto& b = builder(node_id.Tile_Base());
    return b.nodes()[node_id.id()].latlng(b.header_builder().base_ll());
  }

  void write_tiles();

private:
  const uint8_t m_level;
  std::unordered_map<vb::GraphId, std::shared_ptr<vj::GraphTileBuilder>> m_builders;
};

graph_writer::graph_writer(uint8_t level) : m_level(level) {
}

vj::GraphTileBuilder& graph_writer::builder(vb::GraphId tile_id) {
  auto itr = m_builders.find(tile_id);

  if (itr == m_builders.end()) {
    bool inserted = false;
    auto builder = std::make_shared<vj::GraphTileBuilder>(test_tile_dir, tile_id, false);

    std::tie(itr, inserted) = m_builders.emplace(tile_id, builder);
    // should be new, since itr == end.
    assert(inserted);
  }

  assert(itr != m_builders.end());
  return *(itr->second);
}

void graph_writer::write_tiles() {
  using vj::GraphTileBuilder;

  GraphTileBuilder::tweeners_t all_tweeners;

  for (auto& entry : m_builders) {
    auto tile_id = entry.first;
    auto& tile = entry.second;

    // set the base lat,lng in the header builder
    PointLL base_ll = TileHierarchy::get_tiling(tile_id.level()).Base(tile_id.tileid());
    tile->header_builder().set_base_ll(base_ll);

    // write the tile
    tile->StoreTileData();

    // drop the pointer, so that the only copy of the data is on disk.
    tile.reset();

    // write the bin data
    GraphTileBuilder::tweeners_t tweeners;
    auto reloaded = GraphTile::Create(test_tile_dir, tile_id);
    auto bins = GraphTileBuilder::BinEdges(reloaded, tweeners);
    GraphTileBuilder::AddBins(test_tile_dir, reloaded, bins);

    // merge tweeners into global
    for (const auto& entry : tweeners) {
      auto status = all_tweeners.insert(entry);
      if (!status.second) {
        const auto& bins = entry.second;
        auto itr = status.first;

        for (size_t i = 0; i < vb::kBinCount; ++i) {
          auto& target_bin = itr->second[i];
          target_bin.insert(target_bin.end(), bins[i].cbegin(), bins[i].cend());
        }
      }
    }
  }

  for (const auto& entry : all_tweeners) {
    // re-open tiles to add tweeners back in.
    auto tile = vb::GraphTile::Create(test_tile_dir, entry.first);
    vj::GraphTileBuilder::AddBins(test_tile_dir, tile, entry.second);
  }
}

struct edge_count_tracker {
  uint32_t update(vb::GraphId tile_id, uint32_t count) {
    auto itr = m_counts.find(tile_id);
    if (itr == m_counts.end()) {
      bool inserted;
      std::tie(itr, inserted) = m_counts.emplace(tile_id, 0);
    }
    uint32_t index = itr->second;
    itr->second += count;
    return index;
  }
  void clear() {
    m_counts.clear();
  }

private:
  std::unordered_map<vb::GraphId, uint32_t> m_counts;
};

// functor to sort GraphId objects by level, tile then id within the tile.
struct sort_by_tile {
  inline bool operator()(vb::GraphId a, vb::GraphId b) const {
    return ((a.level() < b.level()) ||
            ((a.level() == b.level()) &&
             ((a.tileid() < b.tileid()) || ((a.tileid() == b.tileid()) && (a.id() < b.id())))));
  }
};

// functor to sort pairs of GraphId objects by the functor above
struct sort_pair_by_tile {
  typedef std::pair<vb::GraphId, vb::GraphId> value_type;
  static const sort_by_tile sort_tile;

  inline bool operator()(const value_type& a, const value_type& b) {
    return (sort_tile(a.first, b.first) || ((a.first == b.first) && sort_tile(a.second, b.second)));
  }
};

const sort_by_tile sort_pair_by_tile::sort_tile = {};

// temporary structure for holding a bunch of nodes and edges until they can be
// renumbered to the format needed for storing in tiles.
struct graph_builder {
  std::vector<vm::PointLL> nodes;
  std::vector<std::pair<size_t, size_t>> edges;
  void write_tiles(uint8_t level) const;
};

void graph_builder::write_tiles(uint8_t level) const {
  using namespace valhalla::mjolnir;

  const size_t num_nodes = nodes.size();
  const size_t num_edges = edges.size();

  graph_writer writer(level);
  edge_count_tracker edge_counts;

  // count the number of edges originating at a node
  std::vector<uint32_t> edges_from_node(num_nodes, 0);
  for (const auto& e : edges) {
    edges_from_node[e.first] += 1;
  }

  // renumber nodes into tiles
  std::vector<vb::GraphId> node_ids;
  node_ids.reserve(num_nodes);
  for (size_t i = 0; i < num_nodes; ++i) {
    auto coord = nodes[i];
    auto tile_id = TileHierarchy::GetGraphId(coord, level);
    PointLL base_ll = TileHierarchy::get_tiling(tile_id.level()).Base(tile_id.tileid());
    uint32_t n = edges_from_node[i];

    NodeInfo node_builder;
    node_builder.set_latlng(base_ll, coord);
    node_builder.set_edge_index(edge_counts.update(tile_id, n));
    node_builder.set_edge_count(n);

    auto& tile = writer.builder(tile_id);
    node_ids.emplace_back(tile_id.tileid(), level, tile.nodes().size());
    tile.nodes().emplace_back(std::move(node_builder));
  }

  // don't need these any more
  edges_from_node.clear();
  edge_counts.clear();

  // renumber the nodes of all the edges
  typedef std::vector<std::pair<vb::GraphId, vb::GraphId>> edge_vector_t;
  edge_vector_t renumbered_edges;
  renumbered_edges.reserve(num_edges);
  for (auto e : edges) {
    renumbered_edges.emplace_back(node_ids[e.first], node_ids[e.second]);
  }

  // sort edges so that they come in tile, node order. this allows us to figure
  // out which edges start at which nodes in the tile, to assign them. it also
  // allows us to easily look up the opposing edges by binary search.
  std::sort(renumbered_edges.begin(), renumbered_edges.end(), sort_pair_by_tile());

  // find the first renumbered edge for each tile. this allows us to easily
  // calculate the index of the edge in the tile from the offset of the two
  // iterators.
  std::unordered_map<vb::GraphId, edge_vector_t::iterator> tile_bases;
  vb::GraphId last_tile_id;
  for (edge_vector_t::iterator itr = renumbered_edges.begin(); itr != renumbered_edges.end(); ++itr) {
    auto tile_id = itr->first.Tile_Base();
    if (last_tile_id != tile_id) {
      last_tile_id = tile_id;
      tile_bases[tile_id] = itr;
    }
  }

  for (auto e : renumbered_edges) {
    auto tile_id = e.first.Tile_Base();
    auto& tile = writer.builder(tile_id);

    bool forward = e.first < e.second;
    vm::PointLL start_point = writer.node_latlng(e.first);
    vm::PointLL end_point = writer.node_latlng(e.second);

    DirectedEdgeBuilder edge_builder({}, e.second, forward, start_point.Distance(end_point), 1, 1, {},
                                     {}, 0, false, false, false, false, 0, 0, false);

    auto opp = std::make_pair(e.second, e.first);
    auto itr =
        std::lower_bound(renumbered_edges.begin(), renumbered_edges.end(), opp, sort_pair_by_tile());

    // check that we found the opposite edge, which should always exist.
    assert(itr != renumbered_edges.end() && *itr == opp);

    uint32_t opp_index = std::distance(tile_bases[e.second.Tile_Base()], itr);
    edge_builder.set_opp_index(opp_index);

    uint32_t edge_index = tile.directededges().size();
    uint32_t edge_info_offset = 0;
    if ((opp_index < edge_index) && (e.second.Tile_Base() == tile_id)) {
      // opp edge already exists in this tile, so use its edgeinfo
      edge_info_offset = tile.directededges()[opp_index].edgeinfo_offset();

    } else {
      // make an edgeinfo
      std::vector<PointLL> shape = {start_point, end_point};
      if (!forward)
        std::reverse(shape.begin(), shape.end());

      bool add;
      // make more complex edge geom so that there are 3 segments, affine
      // combination doesnt properly handle arcs but who cares
      edge_info_offset = tile.AddEdgeInfo(edge_index, e.first, e.second, 123, 456, 0, 55, shape,
                                          {std::to_string(edge_index)}, {}, 0, add);
    }
    edge_builder.set_edgeinfo_offset(edge_info_offset);

    tile.directededges().emplace_back(std::move(edge_builder));
  }

  writer.write_tiles();
}

void make_tile() {
  // make sure that all the old tiles are gone before trying to make new ones.
  if (filesystem::is_directory(test_tile_dir)) {
    filesystem::remove_all(test_tile_dir);
  }

  graph_builder builder;

  vm::AABB2<vm::PointLL> box{{0.0, 0.0}, {0.5, 0.5}};
  const uint32_t rows = 100, cols = 100;

  float row_stride = box.Height() / (rows - 1);
  float col_stride = box.Width() / (cols - 1);

  for (uint32_t row = 0; row < rows; ++row) {
    for (uint32_t col = 0; col < cols; ++col) {
      builder.nodes.emplace_back(box.minx() + col_stride * col, box.miny() + row_stride * row);
    }
  }

  // add horizontal edges
  for (uint32_t row = 0; row < rows; ++row) {
    for (uint32_t col = 1; col < cols; ++col) {
      uint32_t index = row * cols + col - 1;
      // add edge and its opposite explicitly
      builder.edges.emplace_back(index, index + 1);
      builder.edges.emplace_back(index + 1, index);
    }
  }

  // add vertical edges
  for (uint32_t row = 1; row < rows; ++row) {
    for (uint32_t col = 0; col < cols; ++col) {
      uint32_t index = (row - 1) * cols + col;
      // add edge and its opposite explicitly
      builder.edges.emplace_back(index, index + cols);
      builder.edges.emplace_back(index + cols, index);
    }
  }

  uint8_t level = 2;
  builder.write_tiles(level);

  // run the validator to create opposing edges and check connectivity
  std::stringstream json;
  json << "{ \"mjolnir\": { \"tile_dir\": \"" << test_tile_dir << "\" }";
  json << ", \"concurrency\": 1";
  json << " }";
  boost::property_tree::ptree conf;
  rapidjson::read_json(json, conf);

  vj::GraphValidator::Validate(conf);
}

TEST(Search, test_single_node) {
  // make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"" << test_tile_dir << "\" }";
  boost::property_tree::ptree conf;
  rapidjson::read_json(json, conf);

  vb::GraphReader reader(conf);
  // this should only find the node a 0,0
  vm::AABB2<vm::PointLL> box{{-0.0025, -0.0025}, {0.0025, 0.0025}};

  auto nodes = valhalla::loki::nodes_in_bbox(box, reader);

  EXPECT_EQ(nodes.size(), 1) << "Expecting to find one node";
}

TEST(Search, test_small_node_block) {
  // make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"" << test_tile_dir << "\" }";
  boost::property_tree::ptree conf;
  rapidjson::read_json(json, conf);

  vb::GraphReader reader(conf);
  // this should find the four nodes which form a square at the lower left of
  // the grid. note that the definition of AABB2::Contains would exclude nodes
  // which lie on the right or top boundaries.
  vm::AABB2<vm::PointLL> box{{0.0, 0.0}, {0.0051, 0.0051}};

  auto nodes = valhalla::loki::nodes_in_bbox(box, reader);

  EXPECT_EQ(nodes.size(), 4) << "Expecting to find four nodes";
}

TEST(Search, test_node_at_tile_boundary) {
  // make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"" << test_tile_dir << "\" }";
  boost::property_tree::ptree conf;
  rapidjson::read_json(json, conf);

  vb::GraphReader reader(conf);
  // this should find node which is at the tile boundary
  vm::AABB2<vm::PointLL> box{{0.0, 0.250}, {0.001, 0.253}};

  auto nodes = valhalla::loki::nodes_in_bbox(box, reader);

  EXPECT_EQ(nodes.size(), 1) << "Expecting to find one node";
}

TEST(Search, test_opposite_in_another_tile) {
  /* this tests that the code is able to find the start node of an edge which is
   * a "tweener" and for which the end node is in another tile. this requires
   * that the search look in the end node's tile to look up the opposite edge in
   * order to find its end node, which is the start node of the original edge.
   *
   * this ASCII art diagram will not make it any clearer:
   *
   *   tile 1            tile 2
   *  +-----------------+-----------------+
   *  | node 1/0        |        node 2/0 |
   *  |        edge 1/0 |                 |
   *  |  /-------------->--------------\  |
   *  | o               |               o |
   *  |  \--------------<--------------/  |
   *  |                 | edge 2/0        |
   *  +-----------------+-----------------+
   *
   * this happens at the top right corner of the grid of tiles, where the final
   * node is the only node in the tile and has two edges, both of which go into
   * neighbouring tiles.
   */

  // make the config file
  std::stringstream json;
  json << "{ \"tile_dir\": \"" << test_tile_dir << "\" }";
  boost::property_tree::ptree conf;
  rapidjson::read_json(json, conf);

  vb::GraphReader reader(conf);
  // this should find the four nodes which form a square at the lower left of
  // the grid. note that the definition of AABB2::Contains would exclude nodes
  // which lie on the right or top boundaries.
  vm::AABB2<vm::PointLL> box{{0.5, 0.5}, {0.51, 0.51}};

  auto nodes = valhalla::loki::nodes_in_bbox(box, reader);

  EXPECT_EQ(nodes.size(), 1) << "Expecting to find one node";
}

// Setup and tearown will be called only once for the entire suite
class Env : public ::testing::Environment {
public:
  void SetUp() override {
    make_tile();
  }

  void TearDown() override {
  }
};

} // namespace

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new Env);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
