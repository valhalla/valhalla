#include "test.h"

#include <fstream>
#include <iostream>
#include <vector>

#include "baldr/admin.h"
#include <memory>

using namespace std;
using namespace valhalla::baldr;

// Expected size is 16 bytes. We want to alert if somehow any change grows
// this structure size as that indicates incompatible tiles.
constexpr size_t kAdminExpectedSize = 16;

namespace {

void test_sizeof() {
  if (sizeof(Admin) != kAdminExpectedSize)
    throw std::runtime_error("Admin size should be " + std::to_string(kAdminExpectedSize) + " bytes" +
                             " but is " + std::to_string(sizeof(Admin)));
}

void TestWriteRead() {
  // Make an admin record
  Admin ai(5, 6, "US", "PA");

  if (ai.country_offset() != 5)
    throw runtime_error("Admin country_offset incorrect.");

  if (ai.state_offset() != 6)
    throw runtime_error("Admin state_offset incorrect.");

  if (ai.country_iso() != "US")
    throw runtime_error("Admin country_iso incorrect.");

  if (ai.state_iso() != "PA")
    throw runtime_error("Admin state_iso incorrect.");

  Admin aiStateISO(5, 6, "GB", "WLS");

  if (aiStateISO.state_iso() != "WLS")
    throw runtime_error("Admin 3 char state_iso incorrect.");

  Admin aiEmptyStrings(5, 6, "", "");

  if (aiEmptyStrings.country_iso() != "" && aiEmptyStrings.state_iso() != "")
    throw runtime_error("Admin empty strings test failed.");
}

} // namespace

int main() {
  test::suite suite("admin");

  // Test sizeof the structure
  suite.test(TEST_CASE(test_sizeof));

  // Test structure size
  suite.test(TEST_CASE(TestWriteRead));

  // Write to file and read into Admin records
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
