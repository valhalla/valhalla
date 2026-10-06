#include "midgard/sequence.h"
#include "mjolnir/osmdata.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

using namespace valhalla::midgard;
using valhalla::mjolnir::OSMWayNode;

namespace {

struct osm_node {
  uint64_t id;
  float lng;
  float lat;
  uint32_t attributes;
};

std::string write_nodes(const uint64_t count) {
  std::string file_name = "nodes.nd";
  sequence<osm_node> sequence(file_name, true, 512);
  for (uint64_t i = count - 1; i < count; --i)
    sequence.push_back({i, 0.f, 0.f, 0});
  return file_name;
}

void sort_nodes(const std::string& file_name) {
  sequence<osm_node> sequence(file_name, false, 512);
  sequence.sort([](const osm_node& a, const osm_node& b) { return a.id < b.id; }, 1);
}

void read_nodes(const std::string& file_name, const uint64_t count) {
  sequence<osm_node> sequence(file_name, false, 512);
  auto less_than = [](const osm_node& a, const osm_node& b) { return a.id < b.id; };
  for (uint64_t i = 0; i < count; ++i) {
    // grab an element
    auto element = sequence[i];
    osm_node node = *element;
    ASSERT_EQ(node.id, i) << "Found wrong node at: " + std::to_string(i);
    // change it
    node.lat = 7;
    node.lng = 7;
    // write it back
    element = node;
    // find the same element with binary search (far slower)
    osm_node target{i, 0.f, 0.f, 0};
    ASSERT_NE(sequence.find(target, less_than), sequence.end())
        << "Didn't find node " + std::to_string(i);
  }
}

TEST(Sequence, ReadWrite) {
  size_t count = 1024;
  auto file_name = write_nodes(count);
  sort_nodes(file_name);
  read_nodes(file_name, count);
}

// deterministic bit mix so every element's payload is derivable from its insertion index
uint64_t mix(uint64_t x) {
  x ^= x >> 33;
  x *= 0xff51afd7ed558ccd;
  x ^= x >> 29;
  x *= 0xc4ceb9fe1a85ec53;
  x ^= x >> 32;
  return x;
}

TEST(Sequence, ParallelSort) {
  constexpr size_t count = 1'000'001;
  // random but deterministic osmid
  const auto osmid = [](uint64_t i) { return 12'000'000'000ULL + mix(i) % (count * 8); };
  const auto by_osmid = [](const OSMWayNode& a, const OSMWayNode& b) {
    return a.node.osmid_ < b.node.osmid_;
  };
  const auto by_way_shape = [](const OSMWayNode& a, const OSMWayNode& b) {
    if (a.way_index == b.way_index) {
      return a.way_shape_node_index < b.way_shape_node_index;
    }
    return a.way_index < b.way_index;
  };

  std::filesystem::create_directories(VALHALLA_BUILD_DIR "test/data");
  const std::string file_name = VALHALLA_BUILD_DIR "test/data/sequence_way_nodes.bin";
  {
    sequence<OSMWayNode> way_nodes(file_name, true);
    uint32_t way = 0, shape = 0, shape_length = 2;
    for (uint64_t i = 0; i < count; ++i) {
      OSMWayNode way_node{};
      way_node.node.osmid_ = osmid(i);
      way_node.way_index = way;
      way_node.way_shape_node_index = shape;
      way_nodes.push_back(way_node);

      // simulate ways that have between 2 and 22 nodes each, cycling
      shape += 1;
      if (shape == shape_length) {
        way += 1;
        shape = 0;
        shape_length = (way % 21) + 2;
      }
    }
  }

  // a full cycle of 21 ways holds 252 nodes, so way w's first node index has a closed form
  const auto way_offset = [](uint64_t w) { return w / 21 * 252 + (w % 21) * (w % 21 + 3) / 2; };

  // way and shape indexes follow the insertion order, so sorting by them must restore it exactly
  const auto verify_insertion_order = [&]() {
    sequence<OSMWayNode> way_nodes(file_name, false);
    ASSERT_EQ(way_nodes.size(), count);
    for (size_t i = 0; i < count; ++i) {
      const OSMWayNode way_node = *way_nodes[i];
      ASSERT_EQ(way_offset(way_node.way_index) + way_node.way_shape_node_index, i)
          << "Out of order at " << i;
    }
  };

  // sort back and forth between the two orders, small buffers force the chunked parallel path
  const std::pair<size_t, uint32_t> configs[] = {
      {100000, 4},  // 11 chunks, the last one holds a single element
      {33333, 8},   // 31 chunks, the last one holds only 11 elements
      {300000, 16}, // more workers than chunks, clamped in both phases
      {100000, 1},  // single worker still takes the chunked path
  };
  for (const auto& [buffer_size, concurrency] : configs) {
    SCOPED_TRACE("buffer_size=" + std::to_string(buffer_size) +
                 " concurrency=" + std::to_string(concurrency));

    sequence<OSMWayNode>(file_name, false).sort(by_osmid, concurrency, buffer_size);

    // check the order and that no element was lost, duplicated or torn
    {
      sequence<OSMWayNode> way_nodes(file_name, false);
      ASSERT_EQ(way_nodes.size(), count);
      std::vector<bool> seen(count, false);
      uint64_t prev = 0;
      for (size_t i = 0; i < count; ++i) {
        const OSMWayNode way_node = *way_nodes[i];
        ASSERT_LE(prev, way_node.node.osmid_) << "Out of order at " << i;
        prev = way_node.node.osmid_;
        const uint64_t index = way_offset(way_node.way_index) + way_node.way_shape_node_index;
        ASSERT_LT(index, count) << "Torn element at " << i;
        ASSERT_FALSE(seen[index]) << "Duplicated element at " << i;
        seen[index] = true;
        ASSERT_EQ(way_node.node.osmid_, osmid(index)) << "Torn element at " << i;
      }
    }

    sequence<OSMWayNode>(file_name, false).sort(by_way_shape, concurrency, buffer_size);
    verify_insertion_order();
  }

  // sorting an already sorted file must preserve the order
  sequence<OSMWayNode>(file_name, false).sort(by_way_shape, 4, 100000);
  verify_insertion_order();

  // a failed ASSERT returns early and leaves the file behind for inspection
  std::filesystem::remove(file_name);
}

// duplicate-heavy node ids stress the splitters: equal samples collapse partitions to empty ones
TEST(Sequence, ParallelSortDuplicateKeys) {
  constexpr size_t count = 500'000;
  const auto by_osmid = [](const OSMWayNode& a, const OSMWayNode& b) {
    return a.node.osmid_ < b.node.osmid_;
  };

  std::filesystem::create_directories(VALHALLA_BUILD_DIR "test/data");
  const std::string file_name = VALHALLA_BUILD_DIR "test/data/sequence_way_nodes_duplicates.bin";
  for (const uint64_t distinct : {uint64_t(1), uint64_t(16)}) {
    SCOPED_TRACE("distinct=" + std::to_string(distinct));
    {
      sequence<OSMWayNode> way_nodes(file_name, true);
      for (uint64_t i = 0; i < count; ++i) {
        OSMWayNode way_node{};
        way_node.node.osmid_ = 12'000'000'000ULL + mix(i) % distinct;
        way_node.way_index = static_cast<uint32_t>(i);
        way_node.way_shape_node_index = static_cast<uint32_t>(mix(i) >> 32);
        way_nodes.push_back(way_node);
      }
    }

    sequence<OSMWayNode>(file_name, false).sort(by_osmid, 8, 33333);

    // check the order and that no element was lost, duplicated or torn
    sequence<OSMWayNode> way_nodes(file_name, false);
    ASSERT_EQ(way_nodes.size(), count);
    std::vector<bool> seen(count, false);
    uint64_t prev = 0;
    for (size_t i = 0; i < count; ++i) {
      const OSMWayNode way_node = *way_nodes[i];
      ASSERT_LE(prev, way_node.node.osmid_) << "Out of order at " << i;
      prev = way_node.node.osmid_;
      ASSERT_LT(way_node.way_index, count);
      ASSERT_FALSE(seen[way_node.way_index]) << "Duplicated element at " << i;
      seen[way_node.way_index] = true;
      ASSERT_EQ(way_node.node.osmid_, 12'000'000'000ULL + mix(way_node.way_index) % distinct)
          << "Torn element at " << i;
      ASSERT_EQ(way_node.way_shape_node_index, static_cast<uint32_t>(mix(way_node.way_index) >> 32))
          << "Torn element at " << i;
    }
  }
  std::filesystem::remove(file_name);
}

TEST(Sequence, Iterator) {
  sequence<osm_node> sequence("nodes.nd", false, 512);
  auto i = sequence.begin();
  auto j = i + 1;
  EXPECT_EQ(j.position(), 1) << "Plus operator wasn't right";

  ++i;
  EXPECT_EQ(i.position(), 1) << "Pre-increment operator wasn't right";

  auto k = i - 1;
  EXPECT_EQ(k.position(), 0) << "Minus operator wasn't right";

  --i;
  EXPECT_EQ(i.position(), 0) << "Pre-decrement operator wasn't right";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
