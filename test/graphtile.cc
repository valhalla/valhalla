#include "test.h"
#include <cstdint>

#include "baldr/graphtile.h"

#include <vector>

using namespace valhalla::baldr;

namespace {

struct testable_graphtile : public valhalla::baldr::GraphTile {
  testable_graphtile(const uint32_t (&offsets)[kBinCount], std::vector<GraphId>& bins) {
    header_ = new GraphTileHeader();
    header_->set_edge_bin_offsets(offsets);
    edge_bins_ = bins.data();
  }
};

void file_suffix() {

  if (GraphTile::FileSuffix(GraphId(2, 2, 0)) != "2/000/000/002.gph")
    throw std::logic_error("Unexpected graphtile suffix");

  if (GraphTile::FileSuffix(GraphId(4, 2, 0)) != "2/000/000/004.gph")
    throw std::logic_error("Unexpected graphtile suffix");

  if (GraphTile::FileSuffix(GraphId(1197468, 2, 0)) != "2/001/197/468.gph")
    throw std::logic_error("Unexpected graphtile suffix");

  if (GraphTile::FileSuffix(GraphId(64799, 1, 0)) != "1/064/799.gph")
    throw std::logic_error("Unexpected graphtile suffix");

  if (GraphTile::FileSuffix(GraphId(49, 0, 0)) != "0/000/049.gph")
    throw std::logic_error("Unexpected graphtile suffix");

  if (GraphTile::FileSuffix(GraphId(1000000, 3, 1)) != "3/001/000/000.gph")
    throw std::logic_error("Unexpected graphtile suffix");
}

void id_from_string() {

  if (GraphTile::GetTileId("foo/bar/baz/qux/corge/1/000/002.gph") != GraphId(2, 1, 0))
    throw std::logic_error("Unexpected graphtile id");

  if (GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/1/000/002.gph") != GraphId(2, 1, 0))
    throw std::logic_error("Unexpected graphtile id");

  if (GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/2/001/000/002.gph") !=
      GraphId(1000002, 2, 0))
    throw std::logic_error("Unexpected graphtile id");

  if (GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/3/001/000/002.gph") !=
      GraphId(1000002, 3, 0))
    throw std::logic_error("Unexpected graphtile id");

  if (GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/3/001/000/002") !=
      GraphId(1000002, 3, 0))
    throw std::logic_error("Unexpected graphtile id");

  if (GraphTile::GetTileId("2/000/791/317.gph.gz") != GraphId(791317, 2, 0))
    throw std::logic_error("Unexpected graphtile id");

  try {
    GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/1/000/002/.gph");
    throw std::logic_error("Should fail to get graphtile id");
  } catch (const std::runtime_error&) {}

  try {
    GraphTile::GetTileId("foo2/8675309/bar/1baz2/qux42corge/0/004/050.gph");
    throw std::logic_error("Should fail to get graphtile id");
  } catch (const std::runtime_error&) {}

  try {
    GraphTile::GetTileId("foo/bar/0/004/0-1.gph");
    throw std::logic_error("Should fail to get graphtile id");
  } catch (const std::runtime_error&) {}

  try {
    GraphTile::GetTileId("foo/bar/0/004//001.gph");
    throw std::logic_error("Should fail to get graphtile id");
  } catch (const std::runtime_error&) {}

  try {
    GraphTile::GetTileId("foo/bar/1/000/004/001.gph");
    throw std::logic_error("Should fail to get graphtile id");
  } catch (const std::runtime_error&) {}

  try {
    GraphTile::GetTileId("00/002.gph");
    throw std::logic_error("Should fail to get graphtile id");
  } catch (const std::runtime_error&) {}
}

void bin() {
  uint32_t offsets[kBinCount] = {1, 2, 3, 0, 1, 2, 3, 1, 1, 2, 3, 2, 1,
                                 2, 3, 3, 1, 2, 3, 4, 1, 2, 3, 5, 1};
  std::vector<uint32_t> offs = {0};
  std::vector<GraphId> bins;
  uint32_t offset = 0;
  for (size_t i = 0, j; i < kBinCount; ++i) {
    offset += offsets[i];
    offs.push_back(offset);
    offsets[i] = offset;
    for (size_t k = 0; k < offsets[i]; ++k)
      bins.emplace_back(j++);
  }
  testable_graphtile t(offsets, bins);
  for (size_t i = 0; i < kBinCount; ++i) {
    valhalla::midgard::iterable_t<GraphId> itr(bins.data() + offs[i], bins.data() + offs[i + 1]);
    auto idx_itr = t.GetBin(i);
    auto rc_itr = t.GetBin(i % kBinsDim, i / kBinsDim);
    if (itr.size() != idx_itr.size() || itr.size() != rc_itr.size())
      throw std::logic_error("Wrong bin!");
    for (auto j = itr.begin(), k = idx_itr.begin(), l = rc_itr.begin(); j != itr.end(); ++j, ++k, ++l)
      if (*j != *k || *j != *l)
        throw std::logic_error("Wrong edge found in bin");
  }
}

void integrity() {
  auto test_tile_from_array = [&](size_t tile_size) {
    std::vector<char> tile_data(tile_size);
    GraphTile tile(GraphId(), tile_data.data(), tile_size);
  };

  try {
    test_tile_from_array(0);
    throw std::logic_error("Should fail to initialize graphitle");
  } catch (const std::runtime_error&) {}

  try {
    test_tile_from_array(sizeof(GraphTileHeader) - 1);
    throw std::logic_error("Should fail to initialize graphitle");
  } catch (const std::runtime_error&) {}

  {
    size_t tile_size = 10000;

    GraphTileHeader header;
    // set offset not equal to data size
    header.set_end_offset(tile_size - 1);

    std::vector<char> tile_data(tile_size);
    memcpy(tile_data.data(), &header, sizeof(header));

    try {
      GraphTile tile(GraphId(), tile_data.data(), tile_size);
      throw std::logic_error("Should fail to initialize graphitle");
    } catch (const std::runtime_error&) {}
  }
}

} // namespace

int main() {
  test::suite suite("graphtile");

  suite.test(TEST_CASE(file_suffix));

  suite.test(TEST_CASE(id_from_string));

  suite.test(TEST_CASE(bin));

  suite.test(TEST_CASE(integrity));

  return suite.tear_down();
}
