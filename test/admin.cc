#include "test.h"

#include <vector>
#include <iostream>
#include <fstream>

#include "baldr/admin.h"
#include <memory>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TestWriteRead() {
  // Make an admin recrod
  Admin ai(5, 6,"US", "PA");

  if (ai.country_offset() != 5)
    throw runtime_error("Admin country_offset incorrect.");

  if (ai.state_offset() != 6)
    throw runtime_error("Admin state_offset incorrect.");

  if (ai.country_iso() != "US")
    throw runtime_error("Admin country_iso incorrect.");

  if (ai.state_iso() != "PA")
    throw runtime_error("Admin state_iso incorrect.");

  Admin aiStateISO(5, 6,"GB", "WLS");

  if (aiStateISO.state_iso() != "WLS")
    throw runtime_error("Admin 3 char state_iso incorrect.");

  Admin aiEmptyStrings(5, 6,"", "");

  if (aiEmptyStrings.country_iso() != "" && aiEmptyStrings.state_iso() != "")
    throw runtime_error("Admin empty strings test failed.");
}

}

int main() {
  test::suite suite("admin");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
