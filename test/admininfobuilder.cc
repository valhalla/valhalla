#include "test.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <valhalla/baldr/admininfo.h>
#include "mjolnir/admininfobuilder.h"
#include <boost/shared_array.hpp>
#include <memory>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

void TestWriteRead() {
  // Make a builder
  AdminInfoBuilder aibuilder(5,6,"US","PA");

  if (aibuilder.country_offset() != 5)
    throw runtime_error("AdminInfoBuilder country_offset incorrect.");

  if (aibuilder.state_offset() != 6)
    throw runtime_error("AdminInfoBuilder state_offset incorrect.");

  if (aibuilder.country_iso() != "US")
    throw runtime_error("AdminInfoBuilder country_iso incorrect.");

  if (aibuilder.state_iso() != "PA")
    throw runtime_error("AdminInfoBuilder state_iso incorrect.");

  AdminInfoBuilder aibuilderStateISO(5,6,"GB","WLS");

  if (aibuilderStateISO.state_iso() != "WLS")
    throw runtime_error("AdminInfoBuilder 3 char state_iso incorrect.");

  AdminInfoBuilder aibuilderEmptyStrings(5,6,"","");

  if (aibuilderEmptyStrings.country_iso() != "" && aibuilderEmptyStrings.state_iso() != "")
    throw runtime_error("AdminInfoBuilder empty strings test failed.");
}

}

int main() {
  test::suite suite("admininfobuilder");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
