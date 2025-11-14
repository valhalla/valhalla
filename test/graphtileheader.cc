#include "baldr/graphtileheader.h"
#include "midgard/pointll.h"

#include <gtest/gtest.h>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

TEST(GraphtileHeader, TestWriteRead) {
  // Test building a directed edge and reading back values
  GraphTileHeader hdr;

  GraphId tile_id(2555, 2, 0);
  hdr.set_graphid(tile_id);
  EXPECT_EQ(hdr.graphid(), tile_id);

  hdr.set_date_created(12345);
  EXPECT_EQ(hdr.date_created(), 12345);

  PointLL base{(tile_id.tileid() % 1440) * .25 - 180, (tile_id.tileid() / 1440) * .25 - 90};
  hdr.set_base_ll(base);
  EXPECT_EQ(hdr.base_ll(), base);

  std::string version = "3.99.99-3a4fe6b";
  hdr.set_version(version + "more_characters"); // should be truncated
  EXPECT_EQ(hdr.version(), version);

  hdr.set_dataset_id(5678);
  EXPECT_EQ(hdr.dataset_id(), 5678);

  hdr.set_density(5);
  EXPECT_EQ(hdr.density(), 5);

  hdr.set_density(kMaxDensity + 10);
  EXPECT_EQ(hdr.density(), kMaxDensity);

  hdr.set_name_quality(5);
  EXPECT_EQ(hdr.name_quality(), 5);

  hdr.set_name_quality(kMaxQualityMeasure + 10);
  EXPECT_EQ(hdr.name_quality(), kMaxQualityMeasure);

  hdr.set_speed_quality(5);
  EXPECT_EQ(hdr.speed_quality(), 5);

  hdr.set_speed_quality(kMaxQualityMeasure + 10);
  EXPECT_EQ(hdr.speed_quality(), kMaxQualityMeasure);

  hdr.set_exit_quality(5);
  EXPECT_EQ(hdr.exit_quality(), 5);

  hdr.set_exit_quality(kMaxQualityMeasure + 10);
  EXPECT_EQ(hdr.exit_quality(), kMaxQualityMeasure);

  hdr.set_nodecount(55511);
  EXPECT_EQ(hdr.nodecount(), 55511);

  hdr.set_transitioncount(555);
  EXPECT_EQ(hdr.transitioncount(), 555);

  hdr.set_directededgecount(55511);
  EXPECT_EQ(hdr.directededgecount(), 55511);

  hdr.set_signcount(55511);
  EXPECT_EQ(hdr.signcount(), 55511);

  hdr.set_access_restriction_count(55511);
  EXPECT_EQ(hdr.access_restriction_count(), 55511);

  hdr.set_admincount(55511);
  EXPECT_EQ(hdr.admincount(), 55511);

  hdr.set_departurecount(555);
  EXPECT_EQ(hdr.departurecount(), 555);

  EXPECT_THROW(hdr.set_departurecount(kMaxTransitDepartures + 1), std::runtime_error);

  hdr.set_stopcount(555);
  EXPECT_EQ(hdr.stopcount(), 555);

  EXPECT_THROW(hdr.set_stopcount(kMaxTransitStops + 1), std::runtime_error);

  hdr.set_routecount(555);
  EXPECT_EQ(hdr.routecount(), 555) << "Header transit route count test failed";

  EXPECT_THROW(hdr.set_routecount(kMaxTransitRoutes + 1), std::runtime_error);

  hdr.set_schedulecount(555);
  EXPECT_EQ(hdr.schedulecount(), 555) << "Header transit schedule count test failed";

  EXPECT_THROW(hdr.set_schedulecount(kMaxTransitSchedules + 1), std::runtime_error);

  hdr.set_transfercount(555);
  EXPECT_EQ(hdr.transfercount(), 555) << "Header transit transfer count test failed";

  EXPECT_THROW(hdr.set_transfercount(kMaxTransfers + 1), std::runtime_error);

  hdr.set_complex_restriction_forward_offset(55511);
  EXPECT_EQ(hdr.complex_restriction_forward_offset(), 55511);

  hdr.set_complex_restriction_reverse_offset(55511);
  EXPECT_EQ(hdr.complex_restriction_reverse_offset(), 55511);

  hdr.set_edgeinfo_offset(55511);
  EXPECT_EQ(hdr.edgeinfo_offset(), 55511);

  hdr.set_textlist_offset(55511);
  EXPECT_EQ(hdr.textlist_offset(), 55511);

  // TODO - add tests for edge bin offsets
  uint32_t offsets[kBinCount];
  offsets[10] = 66666;
  hdr.set_edge_bin_offsets(offsets);
  auto offset = hdr.bin_offset(10);
  EXPECT_EQ(offset.second, 66666) << "Header edge bin offset test failed";

  // Test for trying to access outside the bin index list
  EXPECT_THROW(hdr.bin_offset(kBinCount + 1), std::runtime_error);

  uint64_t checksum = 24189014;
  hdr.set_checksum(checksum);
  EXPECT_EQ(hdr.checksum(), checksum);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
