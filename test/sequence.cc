#include "midgard/sequence.h"
#include "test.h"
#include <cstdint>

using namespace valhalla::midgard;

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
    if (node.id != i)
      throw std::runtime_error("Found wrong node at: " + std::to_string(i));
    // change it
    node.lat = 7;
    node.lng = 7;
    // write it back
    element = node;
    // find the same element with binary search (far slower)
    osm_node target{i};
    if (sequence.find(target, less_than) == sequence.end())
      throw std::runtime_error("Didn't find node " + std::to_string(i));
  }
}

void test_read_write() {
  size_t count = 1024;
  auto file_name = write_nodes(count);
  sort_nodes(file_name);
  read_nodes(file_name, count);
}

void test_iterator() {
  sequence<osm_node> sequence("nodes.nd", false, 512);
  auto i = sequence.begin();
  auto j = i + 1;
  if (j.position() != 1)
    throw std::runtime_error("Plus operator wasn't right");

  ++i;
  if (i.position() != 1)
    throw std::runtime_error("Pre-increment operator wasn't right");

  auto k = i - 1;
  if (k.position() != 0)
    throw std::runtime_error("Minus operator wasn't right");

  --i;
  if (i.position() != 0)
    throw std::runtime_error("Pre-decrement operator wasn't right");
}

int main() {
  test::suite suite("sequence");

  suite.test(TEST_CASE(test_read_write));

  suite.test(TEST_CASE(test_iterator));

  return suite.tear_down();
}
