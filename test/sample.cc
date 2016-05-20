#include "test.h"
#include "skadi/sample.h"
#include "pixels.h"
using namespace valhalla;

#include <cmath>
#include <list>
#include <fstream>

namespace {

void no_data() {
  //check the no data value
  skadi::sample s("test/data/this_is_not_a_directory");
  if(s.get_no_data_value() != -32768)
    throw std::logic_error("No data value should be -32768");

  if(s.get(std::make_pair(0.0,0.0)) != s.get_no_data_value())
    throw std::logic_error("Asked for point with no data should be no data value");

  if(s.get(std::make_pair(200.0, 200.0)) != s.get_no_data_value())
    throw std::logic_error("Asked for point outside of valid range");
}

void create_tile() {
  //its annoying to have to get actual data but its also very boring to test with fake data
  //so we get some real data build the tests and then create the data on the fly
  //get the real tile and run the tests against it uncommenting the log infos in src/sample.cc
  //wget -q -O - http://s3.amazonaws.com/mapzen.valhalla/elevation/N40/N40W077.hgt.gz | gunzip > test/data/N40W077.hgt
  //hack the tests to run against that and turn that into something we can build a tile out of
  //grep -E '{[0-9,]+}' test/*.log | sed -e "s/.*{/{/g" | sort -n | tr '\n' ',' | sed -e "s/^/#include<cstdint>\n#include<unordered_map>\nstd::unordered_map<size_t,int16_t> pixels {/g" -e "s/$/};/g" > test/pixels.h
  int16_t tile[3601 * 3601];
  for (const auto& p : pixels)
    tile[p.first] = p.second;
  std::ofstream file("test/data/sample/N40W077.hgt", std::ios::binary | std::ios::trunc);
  file.write(static_cast<const char*>(static_cast<void*>(&tile)), sizeof(tile));
}

void get() {
  //check a single point
  skadi::sample s("test/data/sample");
  if(std::fabs(490 - s.get(std::make_pair(-76.503915, 40.678783))) > 1.0)
    throw std::runtime_error("Wrong value at location");

  //check a point near the edge of a tile
  if(std::fabs(134 - s.get(std::make_pair(-76.9, 40.0))) > 1.0)
    throw std::runtime_error("Wrong value at edge of tile");

  //check a bunch
  std::list<std::pair<double, double> > postings =  {
    {-76.537011, 40.723872},  // 300m
    {-76.537011, 40.726872},  // 350m
    {-76.537011, 40.729872},  // 450m
    {-76.537011, 40.732872},  // 325m
    {-76.537011, 40.735872}   // 250m
  };
  double riemann_sum = 0;
  auto heights = s.get_all(postings);
  for(const auto height : heights) {
    if(height == s.get_no_data_value())
      throw std::runtime_error("Should have heights for all of these points");
    riemann_sum += height;
  }
  if(std::fabs(riemann_sum - 1675) > 100)
    throw std::runtime_error("Area under discretized curve isn't right");
}

}

int main() {
  test::suite suite("sample");

  suite.test(TEST_CASE(no_data));

  suite.test(TEST_CASE(create_tile));

  suite.test(TEST_CASE(get));

  return suite.tear_down();
}
