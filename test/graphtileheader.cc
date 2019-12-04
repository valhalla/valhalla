#include "test.h"

#include "baldr/graphtileheader.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 256. We want to alert if somehow any change grows this structure
// size as that indicates incompatible tiles.
constexpr size_t kGraphTileHeaderExpectedSize = 272;

namespace {

void test_sizeof() {
  if (sizeof(GraphTileHeader) != kGraphTileHeaderExpectedSize)
    throw std::runtime_error("GraphTileHeader size should be " +
                             std::to_string(kGraphTileHeaderExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(GraphTileHeader)));
}

void TestWriteRead() {
  // Test building a directed edge and reading back values
  GraphTileHeader hdr;

  GraphId tile_id(2555, 2, 0);
  hdr.set_graphid(tile_id);
  if (hdr.graphid() != tile_id) {
    throw runtime_error("Header graphId test failed");
  }
  hdr.set_date_created(12345);
  if (hdr.date_created() != 12345) {
    throw runtime_error("Header date created test failed");
  }
  hdr.set_base_ll({-76.5f, 39.5f});
  if (hdr.base_ll().lng() != -76.5f && hdr.base_ll().lat() != 39.5f) {
    throw runtime_error("Header base LL test failed");
  }
  std::string ver = "v1.5";
  hdr.set_version(ver);
  if (hdr.version() != ver) {
    throw runtime_error("Header version test failed");
  }
  hdr.set_dataset_id(5678);
  if (hdr.dataset_id() != 5678) {
    throw runtime_error("Header dataset Id test failed");
  }
  hdr.set_density(5);
  if (hdr.density() != 5) {
    throw runtime_error("Header density test failed");
  }
  hdr.set_density(kMaxDensity + 10);
  if (hdr.density() != kMaxDensity) {
    throw runtime_error("Header density (bounds check) test failed");
  }
  hdr.set_name_quality(5);
  if (hdr.name_quality() != 5) {
    throw runtime_error("Header name quality test failed");
  }
  hdr.set_name_quality(kMaxQualityMeasure + 10);
  if (hdr.name_quality() != kMaxQualityMeasure) {
    throw runtime_error("Header name quality (bounds check) test failed");
  }
  hdr.set_speed_quality(5);
  if (hdr.speed_quality() != 5) {
    throw runtime_error("Header speed quality test failed");
  }
  hdr.set_speed_quality(kMaxQualityMeasure + 10);
  if (hdr.speed_quality() != kMaxQualityMeasure) {
    throw runtime_error("Header speed quality (bounds check) test failed");
  }
  hdr.set_exit_quality(5);
  if (hdr.exit_quality() != 5) {
    throw runtime_error("Header exit quality test failed");
  }
  hdr.set_exit_quality(kMaxQualityMeasure + 10);
  if (hdr.exit_quality() != kMaxQualityMeasure) {
    throw runtime_error("Header exit quality (bounds check) test failed");
  }
  hdr.set_nodecount(55511);
  if (hdr.nodecount() != 55511) {
    throw runtime_error("Header node count test failed");
  }
  hdr.set_transitioncount(555);
  if (hdr.transitioncount() != 555) {
    throw runtime_error("Header node transition count test failed");
  }
  hdr.set_directededgecount(55511);
  if (hdr.directededgecount() != 55511) {
    throw runtime_error("Header directed edge count test failed");
  }
  hdr.set_signcount(55511);
  if (hdr.signcount() != 55511) {
    throw runtime_error("Header sign count test failed");
  }
  hdr.set_access_restriction_count(55511);
  if (hdr.access_restriction_count() != 55511) {
    throw runtime_error("Header access restriction count test failed");
  }
  hdr.set_admincount(55511);
  if (hdr.admincount() != 55511) {
    throw runtime_error("Header admin count test failed");
  }
  hdr.set_departurecount(555);
  if (hdr.departurecount() != 555) {
    throw runtime_error("Header transit departure count test failed");
  }
  try {
    hdr.set_departurecount(kMaxTransitDepartures + 1);
    throw runtime_error("Header transit departure count bounds check failed");
  } catch (const std::runtime_error& e) {
    // Error thrown as it should be
  }
  hdr.set_stopcount(555);
  if (hdr.stopcount() != 555) {
    throw runtime_error("Header transit stop count test failed");
  }
  try {
    hdr.set_stopcount(kMaxTransitStops + 1);
    throw runtime_error("Header transit stop count bounds check failed");
  } catch (const std::runtime_error& e) {
    // Error thrown as it should be
  }
  hdr.set_routecount(555);
  if (hdr.routecount() != 555) {
    throw runtime_error("Header transit route count test failed");
  }
  try {
    hdr.set_routecount(kMaxTransitRoutes + 1);
    throw runtime_error("Header transit route count bounds check failed");
  } catch (const std::runtime_error& e) {
    // Error thrown as it should be
  }
  hdr.set_schedulecount(555);
  if (hdr.schedulecount() != 555) {
    throw runtime_error("Header transit schedule count test failed");
  }
  try {
    hdr.set_schedulecount(kMaxTransitSchedules + 1);
    throw runtime_error("Header transit schedule count bounds check failed");
  } catch (const std::runtime_error& e) {
    // Error thrown as it should be
  }
  hdr.set_transfercount(555);
  if (hdr.transfercount() != 555) {
    throw runtime_error("Header transit transfer count test failed");
  }
  try {
    hdr.set_transfercount(kMaxTransfers + 1);
    throw runtime_error("Header transit transfer count bounds check failed");
  } catch (const std::runtime_error& e) {
    // Error thrown as it should be
  }
  hdr.set_complex_restriction_forward_offset(55511);
  if (hdr.complex_restriction_forward_offset() != 55511) {
    throw runtime_error("Header complex restriction forward offset test failed");
  }
  hdr.set_complex_restriction_reverse_offset(55511);
  if (hdr.complex_restriction_reverse_offset() != 55511) {
    throw runtime_error("Header complex restriction reverse offset test failed");
  }
  hdr.set_edgeinfo_offset(55511);
  if (hdr.edgeinfo_offset() != 55511) {
    throw runtime_error("Header edgeinfo offset test failed");
  }
  hdr.set_textlist_offset(55511);
  if (hdr.textlist_offset() != 55511) {
    throw runtime_error("Header textlist offset test failed");
  }

  // TODO - add tests for edge bin offsets
  uint32_t offsets[kBinCount];
  offsets[10] = 66666;
  hdr.set_edge_bin_offsets(offsets);
  auto offset = hdr.bin_offset(10);
  if (offset.second != 66666) {
    throw runtime_error("Header edge bin offset test failed");
  }

  // Test for trying to access outside the bin index list
  try {
    hdr.bin_offset(kBinCount + 1);
    throw runtime_error("Header bin index bounds check failed");
  } catch (const std::runtime_error& e) {
    // Error thrown as it should be
  }
}
} // namespace

int main(void) {
  test::suite suite("graphtileheader");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into GraphTileHeader
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
