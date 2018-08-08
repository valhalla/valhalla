#include "skadi/sample.h"
#include "pixels.h"
#include "test.h"

#include "baldr/compression_utils.h"
#include "midgard/sequence.h"
#include "midgard/util.h"

#include <cmath>
#include <fstream>
#include <list>

using namespace valhalla;

namespace {

void no_data() {
  // check the no data value
  skadi::sample s("test/data/this_is_not_a_directory");
  if (skadi::sample::get_no_data_value() != -32768)
    throw std::logic_error("No data value should be -32768");

  if (s.get(std::make_pair(0.0, 0.0)) != skadi::sample::get_no_data_value())
    throw std::logic_error("Asked for point with no data should be no data value");

  if (s.get(std::make_pair(200.0, 200.0)) != skadi::sample::get_no_data_value())
    throw std::logic_error("Asked for point outside of valid range");
}

void create_tile() {
  // its annoying to have to get actual data but its also very boring to test with fake data
  // so we get some real data build the tests and then create the data on the fly
  // get the real tile and run the tests against it uncommenting the log infos in src/sample.cc
  // wget -q -O - http://s3.amazonaws.com/mapzen.valhalla/elevation/N40/N40W077.hgt.gz | gunzip >
  // test/data/N40W077.hgt hack the tests to run against that and turn that into something we can
  // build a tile out of grep -E '{[0-9,]+}' test/*.log | sed -e "s/.*{/{/g" | sort -n | tr '\n' ','
  // | sed -e "s/^/#include<cstdint>\n#include<unordered_map>\nstd::unordered_map<size_t,int16_t>
  // pixels {/g" -e "s/$/};/g" > test/pixels.h
  std::vector<int16_t> tile(3601 * 3601, 0);
  for (const auto& p : pixels) {
    tile[p.first] = p.second;
  }
  std::ofstream file("test/data/sample/N40/N40W077.hgt", std::ios::binary | std::ios::trunc);
  file.write(static_cast<const char*>(static_cast<void*>(tile.data())),
             sizeof(int16_t) * tile.size());

  // input for gzip
  auto src_func = [&tile](z_stream& s) -> int {
    s.next_in = static_cast<Byte*>(static_cast<void*>(tile.data()));
    s.avail_in = static_cast<unsigned int>(tile.size() * sizeof(decltype(tile)::value_type));
    return Z_FINISH;
  };

  // output for gzip
  std::vector<char> dst_buffer(13000, 0);
  std::ofstream gzfile("test/data/samplegz/N40/N40W077.hgt.gz", std::ios::binary | std::ios::trunc);
  auto dst_func = [&dst_buffer, &gzfile](z_stream& s) -> void {
    // move these bytes to their final resting place
    auto chunk = s.total_out - gzfile.tellp();
    gzfile.write(static_cast<const char*>(static_cast<void*>(dst_buffer.data())), chunk);
    // if more input is coming
    if (s.avail_in > 0) {
      s.next_out = static_cast<Byte*>(static_cast<void*>(dst_buffer.data()));
      s.avail_out = dst_buffer.size();
    }
  };

  // gzip it
  if (!baldr::deflate(src_func, dst_func))
    throw std::logic_error("Can't write gzipped elevation tile");
}

void _get(const std::string& location) {
  // check a single point
  skadi::sample s(location);
  if (std::fabs(490 - s.get(std::make_pair(-76.503915, 40.678783))) > 1.0)
    throw std::runtime_error("Wrong value at location: " +
                             std::to_string(s.get(std::make_pair(-76.503915, 40.678783))));

  // check a point near the edge of a tile
  if (std::fabs(134 - s.get(std::make_pair(-76.9, 40.0))) > 1.0)
    throw std::runtime_error("Wrong value at edge of tile: " +
                             std::to_string(s.get(std::make_pair(-76.9, 40.0))));

  // check a bunch
  std::list<std::pair<double, double>> postings = {
      {-76.537011, 40.723872}, // 300m
      {-76.537011, 40.726872}, // 350m
      {-76.537011, 40.729872}, // 450m
      {-76.537011, 40.732872}, // 325m
      {-76.537011, 40.735872}  // 250m
  };
  double riemann_sum = 0;
  auto heights = s.get_all(postings);
  for (const auto height : heights) {
    if (height == skadi::sample::get_no_data_value())
      throw std::runtime_error("Should have heights for all of these points");
    riemann_sum += height;
  }
  if (std::fabs(riemann_sum - 1675) > 100)
    throw std::runtime_error("Area under discretized curve isn't right");
}
void get() {
  _get("test/data/sample");
};
void getgz() {
  _get("test/data/samplegz");
};

struct testable_sample_t : public skadi::sample {
  testable_sample_t(const std::string& dir) : sample(dir) {
    {
      midgard::sequence<int16_t> s("test/data/blah.hgt", true);
      // first row is 1's second is 2's
      for (size_t i = 0; i < 3601 * 2; ++i)
        s.push_back(((i / 3601 + 1) & 0xFF) << 8);
      for (size_t i = 0; i < 3601 * 4; ++i)
        s.push_back(((-32768 & 0xFF) << 8) | ((-32768 >> 8) & 0xFF));
    }
    mapped_cache.front().first = format_t::RAW;
    mapped_cache.front().second.map("test/data/blah.hgt", 3601 * 6);
  }
};

void edges() {
  testable_sample_t s("/dev/null");

  // check 4 pixels
  auto n = .5f / 3600;
  float v = s.get(std::make_pair(-180.f + n, -89.f - n));
  if (!midgard::equal(v, 1.5f, 0.1f))
    throw std::runtime_error("Wrong value at location");

  // check 2 pixels
  v = s.get(std::make_pair(-180.f + n, -89.f - n * 3));
  if (!midgard::equal(v, 2.f, 0.1f))
    throw std::runtime_error("Wrong value at location");

  // check 0 pixels
  v = s.get(std::make_pair(-180.f + n, -89.f - n * 5));
  if (v != skadi::sample::get_no_data_value())
    throw std::runtime_error("Wrong value at location");
}

void lazy_load() {
  // make sure there is no data there
  { std::ofstream file("test/data/sample/N00/N00E000.hgt", std::ios::binary | std::ios::trunc); }
  skadi::sample s("test/data/sample");
  if (s.get(std::make_pair(0.503915, 0.678783)) != skadi::sample::get_no_data_value())
    throw std::logic_error("Asked for point with no data should be no data value");

  // put data there
  {
    std::vector<int16_t> tile(3601 * 3601, 0);
    for (const auto& p : pixels)
      tile[p.first] = p.second;
    std::ofstream file("test/data/sample/N00/N00E000.hgt", std::ios::binary | std::ios::trunc);
    file.write(static_cast<const char*>(static_cast<void*>(tile.data())),
               sizeof(int16_t) * tile.size());
  }

  // make sure there is now data there
  if (std::fabs(490 - s.get(std::make_pair(1 - 0.503915, 0.678783))) > 1.0)
    throw std::runtime_error("Wrong value at location: " +
                             std::to_string(s.get(std::make_pair(1 - 0.503915, 0.678783))));
}

} // namespace

int main() {
  test::suite suite("sample");

  suite.test(TEST_CASE(no_data));

  suite.test(TEST_CASE(create_tile));

  suite.test(TEST_CASE(get));

  suite.test(TEST_CASE(edges));

  suite.test(TEST_CASE(getgz));

  suite.test(TEST_CASE(lazy_load));

  return suite.tear_down();
}
