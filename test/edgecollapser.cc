#include "test.h"

#include "baldr/graphreader.h"
#include "baldr/nodeinfo.h"
#include "baldr/directededge.h"
#include "baldr/merge.h"

namespace vb = valhalla::baldr;

namespace {

struct graph_tile_builder {
  void add_tile(vb::GraphId id) {
    const size_t nodes_size = nodes.size() * sizeof(vb::NodeInfo);
    const size_t edges_size = edges.size() * sizeof(vb::DirectedEdge);

    std::vector<char> mem(sizeof(vb::GraphTileHeader) + nodes_size + edges_size);
    char *ptr = reinterpret_cast<char *>(mem.data());

    vb::GraphTileHeader *header = new(ptr) vb::GraphTileHeader;
    header->set_graphid(id);
    header->set_nodecount(nodes.size());
    header->set_directededgecount(edges.size());

    ptr += sizeof(vb::GraphTileHeader);
    memcpy(ptr, nodes.data(), nodes_size);
    ptr += nodes_size;
    memcpy(ptr, edges.data(), edges_size);

    auto res = memory.emplace(id, std::move(mem));
    auto &mem2 = res.first->second;
    tiles.emplace(id, vb::GraphTile(id, mem2.data(), mem2.size()));
    nodes.clear();
    edges.clear();
  }

  std::vector<vb::NodeInfo> nodes;
  std::vector<vb::DirectedEdge> edges;

  std::unordered_map<vb::GraphId, std::vector<char> > memory;
  std::unordered_map<vb::GraphId, vb::GraphTile> tiles;
};

boost::property_tree::ptree read_json(const std::string &json) {
  boost::property_tree::ptree p;
  std::istringstream istr(json);
  boost::property_tree::json_parser::read_json(istr, p);
  return p;
}

const boost::property_tree::ptree fake_config =
  read_json("{\"tile_dir\": \"/file/does/not/exist\"}");

struct test_graph_reader : public vb::GraphReader {
  test_graph_reader(std::unordered_map<vb::GraphId, vb::GraphTile> &&tiles)
    : GraphReader(fake_config) {
    cache_ = std::move(tiles);
  }
};

void TestCollapseEdge() {
  vb::TileHierarchy hier("");
  vb::GraphId base_id = hier.GetGraphId(valhalla::midgard::PointLL(0, 0), 0);

  // simplest graph with a collapsible node:
  //
  //          /---(edge 0)-->\       /---(edge 1)-->\
  //  (node 0)                (node 1)              (node 2)
  //          \<--(edge 2)---/       \<--(edge 3)---/
  //
  graph_tile_builder builder;
  builder.nodes.push_back(
    vb::NodeInfo(
      std::make_pair(0.00f, 0.0f),
      vb::RoadClass::kResidential,
      vb::kAllAccess,
      vb::NodeType::kStreetIntersection,
      false));
  builder.nodes.back().set_edge_count(1);
  builder.nodes.back().set_edge_index(0);

  builder.nodes.push_back(
    vb::NodeInfo(
      std::make_pair(0.01f, 0.0f),
      vb::RoadClass::kResidential,
      vb::kAllAccess,
      vb::NodeType::kStreetIntersection,
      false));
  builder.nodes.back().set_edge_count(2);
  builder.nodes.back().set_edge_index(1);

  builder.nodes.push_back(
    vb::NodeInfo(
      std::make_pair(0.02f, 0.0f),
      vb::RoadClass::kResidential,
      vb::kAllAccess,
      vb::NodeType::kStreetIntersection,
      false));
  builder.nodes.back().set_edge_count(1);
  builder.nodes.back().set_edge_index(3);

  builder.edges.emplace_back();
  builder.edges.back().set_endnode(base_id + uint64_t(1));
  builder.edges.back().set_length(1113);
  builder.edges.back().set_forwardaccess(vb::kAllAccess);
  builder.edges.back().set_reverseaccess(vb::kAllAccess);
  builder.edges.back().set_classification(vb::RoadClass::kResidential);

  builder.edges.emplace_back();
  builder.edges.back().set_endnode(base_id + uint64_t(2));
  builder.edges.back().set_length(1113);
  builder.edges.back().set_forwardaccess(vb::kAllAccess);
  builder.edges.back().set_reverseaccess(vb::kAllAccess);
  builder.edges.back().set_classification(vb::RoadClass::kResidential);

  builder.edges.emplace_back();
  builder.edges.back().set_endnode(base_id + uint64_t(0));
  builder.edges.back().set_length(1113);
  builder.edges.back().set_forwardaccess(vb::kAllAccess);
  builder.edges.back().set_reverseaccess(vb::kAllAccess);
  builder.edges.back().set_classification(vb::RoadClass::kResidential);

  builder.edges.emplace_back();
  builder.edges.back().set_endnode(base_id + uint64_t(1));
  builder.edges.back().set_length(1113);
  builder.edges.back().set_forwardaccess(vb::kAllAccess);
  builder.edges.back().set_reverseaccess(vb::kAllAccess);
  builder.edges.back().set_classification(vb::RoadClass::kResidential);

  builder.add_tile(base_id);
  assert(builder.tiles.size() == 1);

  test_graph_reader reader(std::move(builder.tiles));

  size_t count = 0;
  std::set<vb::GraphId> edges;
  for (uint64_t i = 0; i < 4; ++i) {
    edges.insert(base_id + i);
  }

  vb::merge::merge(reader, [&](const vb::merge::path &p) {
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

  suite.test(TEST_CASE(TestCollapseEdge));

  return suite.tear_down();
}
