#include "test.h"

#include <fstream>

#include "test/pixels.h"


namespace {

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
    std::ofstream file("test/N40W077.hgt", std::ios::binary | std::ios::trunc);
    file.write(static_cast<const char*>(static_cast<void*>(&tile)), sizeof(tile));
  }

}

int main(void) {
  test::suite suite("Create Elevation Test Tiles");

  suite.test(TEST_CASE(create_tile));

  return suite.tear_down();
}
