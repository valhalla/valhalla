#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "gurka.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace valhalla;

namespace {

// gurka names a node with a single byte, so the chain runs over the graphic ones: ASCII 0x21-0x7E
// and Latin-1 0xA1-0xFF. The walk takes one step per edge, so the length is what drives its depth
std::vector<std::string> chain_node_names() {
  std::vector<std::string> names;
  for (int b = 0x21; b <= 0x7E; ++b) {
    names.emplace_back(1, static_cast<char>(b));
  }
  for (int b = 0xA1; b <= 0xFF; ++b) {
    names.emplace_back(1, static_cast<char>(b));
  }
  return names;
}

gurka::map build_chain_map(const std::vector<std::string>& names) {
  // A straight west-to-east chain, roughly 40 m per edge
  gurka::nodelayout layout;
  for (size_t i = 0; i < names.size(); ++i) {
    layout[names[i]] = {5.0 + static_cast<double>(i) * 0.0005, 45.0};
  }
  gurka::ways ways;
  for (size_t i = 0; i + 1 < names.size(); ++i) {
    ways[names[i] + names[i + 1]] = {{"highway", "primary"}};
  }
  return gurka::buildtiles(layout, ways, {}, {}, "test/data/route_matcher_deep");
}

// Reaching edge_walk through gurka's request builder leaves two shape_match keys and relies on the
// first winning; nothing asserted below would notice if the walk stopped running
std::string build_edge_walk_request(const gurka::map& map, const std::vector<std::string>& names) {
  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  rapidjson::Value shape(rapidjson::kArrayType);
  for (const auto& name : names) {
    const auto& ll = map.nodes.at(name);
    rapidjson::Value point(rapidjson::kObjectType);
    point.AddMember("lon", ll.lng(), allocator);
    point.AddMember("lat", ll.lat(), allocator);
    shape.PushBack(point, allocator);
  }
  doc.AddMember("shape", shape, allocator);
  doc.AddMember("costing", "auto", allocator);
  doc.AddMember("shape_match", "edge_walk", allocator);
  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);
  return sb.GetString();
}

// The node at a given location on a given level, with the tile holding it
std::pair<baldr::GraphId, baldr::graph_tile_ptr>
find_node(baldr::GraphReader& reader, uint32_t level, const midgard::PointLL& ll) {
  for (auto tile_id : reader.GetTileSet(level)) {
    auto tile = reader.GetGraphTile(tile_id);
    for (auto id = tile_id; id.id() < tile->header()->nodecount(); ++id) {
      if (tile->get_node_ll(id).ApproximatelyEqual(ll)) {
        return {id, tile};
      }
    }
  }
  return {{}, nullptr};
}

// The edge walk must have matched every edge of the chain
void assert_full_chain(const std::string& trace_json, size_t expected_edges) {
  rapidjson::Document result;
  result.Parse(trace_json.c_str());
  ASSERT_FALSE(result.HasParseError());
  ASSERT_TRUE(result.HasMember("edges"));
  ASSERT_EQ(result["edges"].GetArray().Size(), expected_edges);
}

} // namespace

TEST(RouteMatcher, LongChainEdgeWalk) {
  auto names = chain_node_names();
  auto map = build_chain_map(names);

  std::string trace_json;
  gurka::do_action(valhalla::Options::trace_attributes, map, build_edge_walk_request(map, names), {},
                   &trace_json);
  assert_full_chain(trace_json, names.size() - 1);
}

// A scan resumed after a failed branch can sit one past a node's last edge or transition. Both
// nodes below own their tile's last, where a one-past index and a one-past pointer differ
TEST(RouteMatcher, BacktrackPastLastEdgeOfTile) {
  gurka::nodelayout layout;
  layout["A"] = {5.10, 45.09};
  layout["B"] = {5.09, 45.09};
  layout["C"] = {5.09, 45.09};
  layout["D"] = {5.08, 45.08};
  layout["N"] = {5.10, 45.10};
  const gurka::ways ways = {
      {"AN", {{"highway", "residential"}}},
      {"NB", {{"highway", "residential"}}},
      {"NCD", {{"highway", "primary"}}},
  };
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/route_matcher_backtrack_edge");

  // The trace only exercises what it is meant to if N is where the tile's edges end
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const auto n = find_node(reader, 2, layout["N"]);
  const auto* nodeinfo = n.second->node(n.first);
  ASSERT_EQ(nodeinfo->edge_index() + nodeinfo->edge_count(), n.second->header()->directededgecount());
  ASSERT_EQ(nodeinfo->edge_count(), 2u);
  ASSERT_GT(nodeinfo->transition_count(), 0u);

  // N->B matches then fails, so the walk must come back and take N's transition to reach D
  std::string trace_json;
  gurka::do_action(valhalla::Options::trace_attributes, map,
                   build_edge_walk_request(map, {"A", "N", "B", "D"}), {}, &trace_json);
  assert_full_chain(trace_json, 2);
}

TEST(RouteMatcher, BacktrackPastLastTransitionOfTile) {
  gurka::nodelayout layout;
  layout["A"] = {5.10, 45.11};
  layout["B"] = {5.10, 45.10};
  layout["C"] = {5.09, 45.09};
  layout["D"] = {5.07, 45.07};
  layout["E"] = {5.08, 45.10};
  layout["N"] = {5.09, 45.09};
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BN", {{"highway", "residential"}}},
      {"BCD", {{"highway", "primary"}}},
      {"NE", {{"highway", "primary"}}},
  };
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/route_matcher_backtrack_transition");

  // One edge, so the edge scan cannot be what runs past the end, and the tile's transitions end here
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const auto n = find_node(reader, 2, layout["N"]);
  const auto* nodeinfo = n.second->node(n.first);
  ASSERT_EQ(nodeinfo->edge_count(), 1u);
  ASSERT_EQ(nodeinfo->transition_index() + nodeinfo->transition_count(),
            n.second->header()->transitioncount());

  // N's only edge and only transition both fail, so B has to fall back on its own transition
  std::string trace_json;
  gurka::do_action(valhalla::Options::trace_attributes, map,
                   build_edge_walk_request(map, {"A", "B", "N", "D"}), {}, &trace_json);
  assert_full_chain(trace_json, 2);
}
