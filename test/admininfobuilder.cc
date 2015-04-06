#include "test.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/admininfo.h>
#include "mjolnir/admininfobuilder.h"
#include <boost/shared_array.hpp>
#include <valhalla/baldr/sign.h>
#include <memory>
#include "mjolnir/signbuilder.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

void TestWriteRead() {
  // Make a builder
  AdminInfoBuilder aibuilder(5,6,"US","PA","20150308","20151101");

  if (aibuilder.country_offset() != 5)
    throw runtime_error("AdminInfoBuilder country_offset incorrect.");

  if (aibuilder.state_offset() != 6)
    throw runtime_error("AdminInfoBuilder state_offset incorrect.");

  if (aibuilder.country_iso() != "US")
    throw runtime_error("AdminInfoBuilder country_iso incorrect.");

  if (aibuilder.state_iso() != "PA")
    throw runtime_error("AdminInfoBuilder state_iso incorrect.");

  if (aibuilder.start_dst() != "20150308")
    throw runtime_error("AdminInfoBuilder start_dst incorrect.");

  if (aibuilder.end_dst() != "20151101")
    throw runtime_error("AdminInfoBuilder end_dst incorrect.");

  AdminInfoBuilder aibuilderStateISO(5,6,"GB","WLS","20150308","20151101");

  if (aibuilderStateISO.state_iso() != "WLS")
    throw runtime_error("AdminInfoBuilder 3 char state_iso incorrect.");

  AdminInfoBuilder aibuilderEmptyStrings(5,6,"","","","");

  if (aibuilderEmptyStrings.country_iso() != "" && aibuilderEmptyStrings.state_iso() != "" &&
      aibuilderEmptyStrings.start_dst() != "" &&  aibuilderEmptyStrings.end_dst() != "")
    throw runtime_error("AdminInfoBuilder empty strings test failed.");
}

}

int main() {
  test::suite suite("admininfobuilder");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
