#include "midgard/sequence.h"
#include "midgard/sequence_writer.h"
#include <cstdint>

#include "test.h"

using namespace valhalla::midgard;

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
  sequence.sort([](const osm_node& a, const osm_node& b) { return a.id < b.id; });
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
    osm_node target{i};
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

TEST(SequenceWriter, Write) {
  std::string filename = "nodes_writer.nd";

  // Destructor should flush
  {
    sequence_writer<osm_node> writer(filename, 512);
    writer.push_back({1, 2.0, 3.0});
  }
  {
    sequence<osm_node> reader(filename, false, 512);
    ASSERT_EQ(reader.size(), 1);

    auto node = reader.back();
    ASSERT_EQ(node.id, 1);
    ASSERT_EQ(node.lng, 2.0);
    ASSERT_EQ(node.lat, 3.0);
  }

  // Manual flush, also new writer overwrites the file
  {
    sequence_writer<osm_node> writer(filename, 512);
    ASSERT_EQ(writer.size(), 0);
    writer.push_back({4, 5.0, 6.0});
    ASSERT_EQ(writer.size(), 1);
    writer.flush();
    ASSERT_EQ(writer.size(), 1);

    sequence<osm_node> reader(filename, false, 512);
    ASSERT_EQ(reader.size(), 1);

    auto node = reader.back();
    ASSERT_EQ(node.id, 4);
    ASSERT_EQ(node.lng, 5.0);
    ASSERT_EQ(node.lat, 6.0);
  }

  // Writes that exceed buffer size, flushed in destructor
  {
    sequence_writer<osm_node> writer(filename, 64);
    for (size_t i = 0; i < 30; ++i) {
      writer.push_back({i, 8.0, 9.0});
    }
    ASSERT_EQ(writer.size(), 30);
  }
  {
    sequence<osm_node> reader(filename, false, 512);
    ASSERT_EQ(reader.size(), 30);

    size_t pos = 0;
    for (auto node : reader) {
      ASSERT_EQ(node.id, pos++);
      ASSERT_EQ(node.lng, 8.0);
      ASSERT_EQ(node.lat, 9.0);
    }
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
