
#include "baldr/complexrestriction.h"
#include "baldr/graphid.h"
#include "mjolnir/complexrestrictionbuilder.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

// Expected size is 48 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kComplexRestrictionExpectedSize = 24;

namespace {

TEST(ComplexRestriction, Sizeof) {
  EXPECT_EQ(sizeof(ComplexRestriction), kComplexRestrictionExpectedSize);
}

TEST(ComplexRestriction, WalkViasBuilder) {
  ComplexRestrictionBuilder builder;

  std::vector<GraphId> expected_vias;
  builder.set_via_list(expected_vias);

  // Ensure the builder cannot walk (throws logic_error to avoid accidentally trying
  // to walk a builder which likely is not what was intended)
  std::vector<GraphId> walked_vias;
  ASSERT_THROW(builder.WalkVias([&walked_vias](const GraphId* via) {
    walked_vias.push_back(*via);
    return WalkingVia::KeepWalking;
  }),
               std::logic_error)
      << "Did not walk the expected vias";
}

TEST(ComplexRestriction, WriteRead) {
  // Test building a ComplexRestriction and reading back values
  ComplexRestriction r;
  EXPECT_FALSE(r.from_graphid().Is_Valid())
      << "ComplexRestriction from Id should be invalid with default constructor";

  // Test set method (complex restriction builder) and get methods
  ComplexRestrictionBuilder res;
  res.set_from_id(GraphId(1234, 1, 111));
  EXPECT_EQ(res.from_graphid(), GraphId(1234, 1, 111))
      << "ComplexRestriction from GraphId get failed";

  res.set_to_id(GraphId(2345, 1, 2222));
  EXPECT_EQ(res.to_graphid(), GraphId(2345, 1, 2222)) << "ComplexRestriction to GraphId get failed";

  res.set_via_list(std::vector<GraphId>(5, GraphId{}));
  EXPECT_EQ(res.via_count(), 5) << "ComplexRestriction via count failed";

  res.set_via_list(std::vector<GraphId>(kMaxViasPerRestriction + 7, GraphId{}));
  EXPECT_EQ(res.via_count(), 5) << "ComplexRestriction via count limit check failed";

  res.set_type(RestrictionType::kNoLeftTurn);
  EXPECT_EQ(res.type(), RestrictionType::kNoLeftTurn) << "ComplexRestriction type failed";

  res.set_modes(2224);
  EXPECT_EQ(res.modes(), 2224) << "ComplexRestriction modes failed";

  res.set_dt(true);
  EXPECT_TRUE(res.has_dt()) << "ComplexRestriction has_dt (DateTime) failed";

  res.set_begin_day_dow(3);
  EXPECT_EQ(res.begin_day_dow(), 3) << "ComplexRestriction begin_day_dow failed";

  res.set_begin_month(7);
  EXPECT_EQ(res.begin_month(), 7) << "ComplexRestriction begin_day_dow failed";

  res.set_begin_week(4);
  EXPECT_EQ(res.begin_week(), 4) << "ComplexRestriction begin_week failed";

  res.set_begin_hrs(5);
  EXPECT_EQ(res.begin_hrs(), 5) << "ComplexRestriction begin_hrs failed";

  res.set_dt_type(true);
  EXPECT_TRUE(res.dt_type()) << "ComplexRestriction dt_type failed";

  res.set_end_day_dow(2);
  EXPECT_EQ(res.end_day_dow(), 2) << "ComplexRestriction end_day_dow failed";

  res.set_end_month(4);
  EXPECT_EQ(res.end_month(), 4) << "ComplexRestriction end_day_dow failed";

  res.set_end_week(5);
  EXPECT_EQ(res.end_week(), 5) << "ComplexRestriction end_week failed";

  res.set_end_hrs(15);
  EXPECT_EQ(res.end_hrs(), 15) << "ComplexRestriction end_hrs failed";

  res.set_dow(53);
  EXPECT_EQ(res.dow(), 53) << "ComplexRestriction dow failed";

  res.set_begin_mins(5);
  EXPECT_EQ(res.begin_mins(), 5) << "ComplexRestriction begin_mins failed";

  res.set_end_mins(55);
  EXPECT_EQ(res.end_mins(), 55) << "ComplexRestriction end_mins failed";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
