#include "test.h"

#include "baldr/complexrestriction.h"
#include "mjolnir/complexrestrictionbuilder.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

// Expected size is 48 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kComplexRestrictionExpectedSize = 24;

namespace {

void test_sizeof() {
  if (sizeof(ComplexRestriction) != kComplexRestrictionExpectedSize)
    throw std::runtime_error("ComplexRestriction size should be " +
                             std::to_string(kComplexRestrictionExpectedSize) + " bytes" + " but is " +
                             std::to_string(sizeof(ComplexRestriction)));
}

void TestWriteRead() {
  // Test building a ComplexRestriction and reading back values
  ComplexRestriction r;
  if (r.from_graphid().Is_Valid()) {
    throw runtime_error("ComplexRestriction from Id should be invalid with default constructor");
  }

  // Test set method (complex restriction builder) and get methods
  ComplexRestrictionBuilder res;
  res.set_from_id(GraphId(1234, 1, 111));
  if (!(res.from_graphid() == GraphId(1234, 1, 111))) {
    throw runtime_error("ComplexRestriction from GraphId get failed");
  }

  res.set_to_id(GraphId(2345, 1, 2222));
  if (!(res.to_graphid() == GraphId(2345, 1, 2222))) {
    throw runtime_error("ComplexRestriction to GraphId get failed");
  }

  res.set_via_count(5);
  if (res.via_count() != 5) {
    throw runtime_error("ComplexRestriction via count failed");
  }

  res.set_via_count(kMaxViasPerRestriction + 5);
  if (res.via_count() != kMaxViasPerRestriction) {
    throw runtime_error("ComplexRestriction via count limit check failed");
  }

  res.set_type(RestrictionType::kNoLeftTurn);
  if (res.type() != RestrictionType::kNoLeftTurn) {
    throw runtime_error("ComplexRestriction type failed");
  }

  res.set_modes(2224);
  if (res.modes() != 2224) {
    throw runtime_error("ComplexRestriction modes failed");
  }

  res.set_dt(true);
  if (!res.has_dt()) {
    throw runtime_error("ComplexRestriction has_dt (DateTime) failed");
  }

  res.set_begin_day_dow(3);
  if (res.begin_day_dow() != 3) {
    throw runtime_error("ComplexRestriction begin_day_dow failed");
  }

  res.set_begin_month(7);
  if (res.begin_month() != 7) {
    throw runtime_error("ComplexRestriction begin_day_dow failed");
  }

  res.set_begin_week(4);
  if (res.begin_week() != 4) {
    throw runtime_error("ComplexRestriction begin_week failed");
  }

  res.set_begin_hrs(5);
  if (res.begin_hrs() != 5) {
    throw runtime_error("ComplexRestriction begin_hrs failed");
  }

  res.set_dt_type(true);
  if (!res.dt_type()) {
    throw runtime_error("ComplexRestriction dt_type failed");
  }

  res.set_end_day_dow(2);
  if (res.end_day_dow() != 2) {
    throw runtime_error("ComplexRestriction end_day_dow failed");
  }

  res.set_end_month(4);
  if (res.end_month() != 4) {
    throw runtime_error("ComplexRestriction end_day_dow failed");
  }

  res.set_end_week(5);
  if (res.end_week() != 5) {
    throw runtime_error("ComplexRestriction end_week failed");
  }

  res.set_end_hrs(15);
  if (res.end_hrs() != 15) {
    throw runtime_error("ComplexRestriction end_hrs failed");
  }

  res.set_dow(53);
  if (res.dow() != 53) {
    throw runtime_error("ComplexRestriction dow failed");
  }

  res.set_begin_mins(5);
  if (res.begin_mins() != 5) {
    throw runtime_error("ComplexRestriction begin_mins failed");
  }

  res.set_end_mins(55);
  if (res.end_mins() != 55) {
    throw runtime_error("ComplexRestriction end_mins failed");
  }
}
} // namespace

int main(void) {
  test::suite suite("complexrestriction");

  suite.test(TEST_CASE(test_sizeof));

  // Write to file and read into ComplexRestriction
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
