#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "gurka.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <climits>
#include <string>
#include <vector>

#ifndef _WIN32
#include <pthread.h>
#endif

using namespace valhalla;

namespace {

// gurka names a node with a single byte, so the chain runs over the graphic byte values: ASCII
// 0x21-0x7E and Latin-1 0xA1-0xFF, which leaves out space, DEL, the C1 block and NBSP. The edge
// walk takes one step per edge, so the chain length is what drives its depth.
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

// gurka's request builder appends its own "shape_match": "map_snap" after it applies the options,
// so reaching the edge walk through it leaves two keys in the request and relies on the first one
// winning. Nothing asserted below would notice if the walk stopped running, so the request is built
// here rather than resting on that.
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

#ifndef _WIN32

struct TraceCall {
  const gurka::map* map;
  std::string request;
  std::string json;
  std::string error;
};

void* run_trace(void* arg) {
  auto* call = static_cast<TraceCall*>(arg);
  try {
    gurka::do_action(valhalla::Options::trace_attributes, *call->map, call->request, {}, &call->json);
  } catch (const std::exception& e) { call->error = e.what(); } catch (...) {
    call->error = "unknown exception";
  }
  return nullptr;
}

#if defined(__SANITIZE_ADDRESS__)
#define ROUTE_MATCHER_DEEP_ASAN 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
#define ROUTE_MATCHER_DEEP_ASAN 1
#endif
#endif

size_t small_stack_bytes() {
#ifdef ROUTE_MATCHER_DEEP_ASAN
  // Redzones inflate every frame, the pipeline outside the walk included, so the stack is raised to
  // keep instrumentation alone from failing this. A per-edge recursion is caught by the
  // uninstrumented builds, not by this one
  constexpr size_t kSmallStack = 512 * 1024;
#else
  // Below what one frame per chain edge would need, above what the request needs without that: on
  // arm64 the whole call runs in under 24 KB here at -O0, while walking the same chain one frame
  // per edge needs more than 192 KB
  constexpr size_t kSmallStack = 48 * 1024;
#endif
  // A platform with a larger floor turns this into a plain smoke test rather than an EINVAL failure
  return std::max<size_t>(kSmallStack, static_cast<size_t>(PTHREAD_STACK_MIN));
}

#endif // _WIN32

} // namespace

TEST(RouteMatcher, LongChainEdgeWalk) {
  auto names = chain_node_names();
  auto map = build_chain_map(names);

  std::string trace_json;
  gurka::do_action(valhalla::Options::trace_attributes, map, build_edge_walk_request(map, names), {},
                   &trace_json);
  assert_full_chain(trace_json, names.size() - 1);
}

#ifndef _WIN32

TEST(RouteMatcher, LongChainEdgeWalkOnSmallThreadStack) {
  auto names = chain_node_names();
  auto map = build_chain_map(names);

  TraceCall call{&map, build_edge_walk_request(map, names), {}, {}};
  pthread_attr_t attr;
  ASSERT_EQ(pthread_attr_init(&attr), 0);
  ASSERT_EQ(pthread_attr_setstacksize(&attr, small_stack_bytes()), 0);
  pthread_t thread;
  ASSERT_EQ(pthread_create(&thread, &attr, run_trace, &call), 0);
  pthread_attr_destroy(&attr);
  ASSERT_EQ(pthread_join(thread, nullptr), 0);

  ASSERT_TRUE(call.error.empty()) << call.error;
  assert_full_chain(call.json, names.size() - 1);
}

#endif // _WIN32

// A node's edges and transitions are visited from a bookmark, so a scan that resumes after a failed
// branch can sit one past the last of them. Both nodes below own the last edge, respectively the
// last transition, of their tile, which is where a one-past index and a one-past pointer differ:
// the index is rejected by the bounds-checked accessors, the pointer only by the loop condition.
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

  // A->N matches, N->B matches and then fails, and the walk has to come back and take N's
  // transition to reach D
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
