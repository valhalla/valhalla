#include "test.h"
#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

#include "baldr/directededge.h"
#include "baldr/graphreader.h"
#include "baldr/merge.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"

namespace vb = valhalla::baldr;

namespace {

struct graph_tile_builder {
  void append_node(valhalla::midgard::PointLL& base_ll,
                   float lon,
                   float lat,
                   uint32_t edge_count,
                   uint32_t first_edge) {
    nodes.push_back(vb::NodeInfo(base_ll, std::make_pair(lon, lat), vb::RoadClass::kResidential,
                                 vb::kAllAccess, vb::NodeType::kStreetIntersection, false));
    nodes.back().set_edge_count(edge_count);
    nodes.back().set_edge_index(first_edge);
  }

  void append_edge(vb::GraphId endnode, uint32_t length) {
    edges.emplace_back();
    edges.back().set_endnode(endnode);
    edges.back().set_length(length);
    edges.back().set_forwardaccess(vb::kAllAccess);
    edges.back().set_reverseaccess(vb::kAllAccess);
    edges.back().set_classification(vb::RoadClass::kResidential);
  }

  void commit_tile(vb::GraphId id) {
    const size_t nodes_size = nodes.size() * sizeof(vb::NodeInfo);
    const size_t edges_size = edges.size() * sizeof(vb::DirectedEdge);

    std::vector<char> mem(sizeof(vb::GraphTileHeader) + nodes_size + edges_size);
    char* ptr = reinterpret_cast<char*>(mem.data());

    vb::GraphTileHeader* header = new (ptr) vb::GraphTileHeader;
    header->set_graphid(id);
    header->set_nodecount(nodes.size());
    header->set_directededgecount(edges.size());
    header->set_end_offset(mem.size());

    ptr += sizeof(vb::GraphTileHeader);
    memcpy(ptr, nodes.data(), nodes_size);
    ptr += nodes_size;
    memcpy(ptr, edges.data(), edges_size);

    auto res = memory.emplace(id, std::move(mem));
    auto& mem2 = res.first->second;
    tiles.emplace(id, vb::GraphTile(id, mem2.data(), mem2.size()));
    nodes.clear();
    edges.clear();
  }

  std::vector<vb::NodeInfo> nodes;
  std::vector<vb::DirectedEdge> edges;

  std::unordered_map<vb::GraphId, std::vector<char>> memory;
  std::unordered_map<vb::GraphId, vb::GraphTile> tiles;
};

boost::property_tree::ptree read_json(const std::string& json) {
  boost::property_tree::ptree p;
  std::istringstream istr(json);
  rapidjson::read_json(istr, p);
  return p;
}

const boost::property_tree::ptree fake_config = read_json("{\"tile_dir\": \"/file/does/not/exist\"}");

struct test_graph_reader : public vb::GraphReader {
  test_graph_reader(const std::unordered_map<vb::GraphId, vb::GraphTile>& tiles)
      : GraphReader(fake_config) {
    for (const auto& it : tiles)
      cache_->Put(it.first, it.second, 0);
  }
};

void TestCollapseEdgeSimple() {
  vb::GraphId base_id = vb::TileHierarchy::GetGraphId(valhalla::midgard::PointLL(0, 0), 0);

  // simplest graph with a collapsible node:
  //
  //          /---(edge 0)-->\       /---(edge 1)-->\
  //  (node 0)                (node 1)              (node 2)
  //          \<--(edge 2)---/       \<--(edge 3)---/
  //
  valhalla::midgard::PointLL base_ll(0.0f, 0.0f);
  graph_tile_builder builder;
  builder.append_node(base_ll, 0.00f, 0.0f, 1, 0);
  builder.append_node(base_ll, 0.01f, 0.0f, 2, 1);
  builder.append_node(base_ll, 0.02f, 0.0f, 1, 3);

  builder.append_edge(base_id + uint64_t(1), 1113);
  builder.append_edge(base_id + uint64_t(2), 1113);
  builder.append_edge(base_id + uint64_t(0), 1113);
  builder.append_edge(base_id + uint64_t(1), 1113);

  builder.commit_tile(base_id);
  assert(builder.tiles.size() == 1);

  test_graph_reader reader(std::move(builder.tiles));

  size_t count = 0;
  std::set<vb::GraphId> edges;
  for (uint64_t i = 0; i < 4; ++i) {
    edges.insert(base_id + i);
  }

  std::vector<vb::GraphId> tiles;
  tiles.push_back(base_id);

  vb::merge::merge(tiles, reader, [](const vb::DirectedEdge*) -> bool { return true; },
                   [](const vb::DirectedEdge*) -> bool { return true; },
                   [&](const vb::merge::path& p) {
                     count += 1;
                     for (auto id : p.m_edges) {
                       if (edges.count(id) != 1) {
                         throw std::runtime_error("Edge not found - either invalid or duplicate!");
                       }
                       edges.erase(id);
                     }
                   });

  if (count != 2) {
    throw std::runtime_error("Should have collapsed to 2 paths.");
  }
  if (!edges.empty()) {
    throw std::runtime_error("Some edges left over!");
  }
}

void TestCollapseEdgeJunction() {
  vb::GraphId base_id = vb::TileHierarchy::GetGraphId(valhalla::midgard::PointLL(0, 0), 0);

  // simplest graph with a non-collapsible node:
  //
  //          /---(edge 0)-->\       /---(edge 1)-->\
  //  (node 0)                (node 1)              (node 2)
  //          \<--(edge 2)---/  /  \ \<--(edge 4)---/
  //                           |   |
  //                  (edge 5) ^   v (edge 3)
  //                           |   |
  //                            \  /
  //                          (node 3)
  valhalla::midgard::PointLL base_ll(0.0f, 0.0f);
  graph_tile_builder builder;
  builder.append_node(base_ll, 0.00f, 0.00f, 1, 0);
  builder.append_node(base_ll, 0.01f, 0.00f, 3, 1);
  builder.append_node(base_ll, 0.02f, 0.00f, 1, 4);
  builder.append_node(base_ll, 0.00f, 0.01f, 1, 5);

  builder.append_edge(base_id + uint64_t(1), 1113);
  builder.append_edge(base_id + uint64_t(2), 1113);
  builder.append_edge(base_id + uint64_t(0), 1113);
  builder.append_edge(base_id + uint64_t(3), 1113);
  builder.append_edge(base_id + uint64_t(1), 1113);
  builder.append_edge(base_id + uint64_t(1), 1113);

  builder.commit_tile(base_id);
  assert(builder.tiles.size() == 1);

  test_graph_reader reader(std::move(builder.tiles));

  size_t count = 0;
  std::set<vb::GraphId> edges;
  for (uint64_t i = 0; i < 6; ++i) {
    edges.insert(base_id + i);
  }

  std::vector<vb::GraphId> tiles;
  tiles.push_back(base_id);

  vb::merge::merge(tiles, reader, [](const vb::DirectedEdge*) -> bool { return true; },
                   [](const vb::DirectedEdge*) -> bool { return true; },
                   [&](const vb::merge::path& p) {
                     count += 1;
                     for (auto id : p.m_edges) {
                       if (edges.count(id) != 1) {
                         throw std::runtime_error("Edge not found - either invalid or duplicate!");
                       }
                       edges.erase(id);
                     }
                   });

  if (count != 6) {
    throw std::runtime_error("Should have not collapsed, leaving 6 original paths.");
  }
  if (!edges.empty()) {
    throw std::runtime_error("Some edges left over!");
  }
}

void TestCollapseEdgeChain() {
  vb::GraphId base_id = vb::TileHierarchy::GetGraphId(valhalla::midgard::PointLL(0, 0), 0);

  // graph with 3 collapsible edges, all chained together. (e.g: think of the
  // middle segment as a bridge).
  //
  //      /---(e0)-->\    /---(e1)-->\    /---(e3)-->\
  //  (n0)            (n1)            (n2)            (n3)
  //      \<--(e2)---/    \<--(e4)---/    \<--(e5)---/
  //
  valhalla::midgard::PointLL base_ll(0.0f, 0.0f);
  graph_tile_builder builder;
  builder.append_node(base_ll, 0.00f, 0.0f, 1, 0);
  builder.append_node(base_ll, 0.01f, 0.0f, 2, 1);
  builder.append_node(base_ll, 0.02f, 0.0f, 2, 3);
  builder.append_node(base_ll, 0.03f, 0.0f, 1, 5);

  builder.append_edge(base_id + uint64_t(1), 1113);
  builder.append_edge(base_id + uint64_t(2), 1113);
  builder.append_edge(base_id + uint64_t(0), 1113);
  builder.append_edge(base_id + uint64_t(3), 1113);
  builder.append_edge(base_id + uint64_t(1), 1113);
  builder.append_edge(base_id + uint64_t(2), 1113);

  builder.commit_tile(base_id);
  assert(builder.tiles.size() == 1);

  test_graph_reader reader(std::move(builder.tiles));

  size_t count = 0;
  std::set<vb::GraphId> edges;
  for (uint64_t i = 0; i < 6; ++i) {
    edges.insert(base_id + i);
  }

  std::vector<vb::GraphId> tiles;
  tiles.push_back(base_id);

  vb::merge::merge(tiles, reader, [](const vb::DirectedEdge*) -> bool { return true; },
                   [](const vb::DirectedEdge*) -> bool { return true; },
                   [&](const vb::merge::path& p) {
                     count += 1;
                     for (auto id : p.m_edges) {
                       if (edges.count(id) != 1) {
                         throw std::runtime_error("Edge not found - either invalid or duplicate!");
                       }
                       edges.erase(id);
                     }
                   });

  if (count != 2) {
    throw std::runtime_error("Should have collapsed to 2 paths.");
  }
  if (!edges.empty()) {
    throw std::runtime_error("Some edges left over!");
  }
}

} // anonymous namespace

int main() {
  test::suite suite("edgecollapser");

  suite.test(TEST_CASE(TestCollapseEdgeSimple));
  suite.test(TEST_CASE(TestCollapseEdgeJunction));
  suite.test(TEST_CASE(TestCollapseEdgeChain));

  return suite.tear_down();
}
