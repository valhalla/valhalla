#include "test.h"
#include <valhalla/odin/streetnames.h>

#include <google/protobuf/repeated_field.h>

#include <vector>
#include <algorithm>

using namespace std;
using namespace valhalla::odin;

namespace {

void TryListCtor(const std::vector<std::string>& names) {
  ::google::protobuf::RepeatedPtrField<::std::string> name_list;
  for (const auto& name : names) {
    name_list.Add()->assign(name);
  }
  StreetNames street_names(name_list);

  int x = 0;
  for (const auto& street_name : street_names) {
    if (name_list.Get(x++) != street_name.value())
      throw std::runtime_error("Incorrect street name value");
  }

}

void TestListCtor() {
  TryListCtor( { "Main Street" });
  TryListCtor( { "Hershey Road", "PA 743 North" });
}

}

int main() {
  test::suite suite("streetnames");

  // Constructor with list argument
  suite.test(TEST_CASE(TestListCtor));

  return suite.tear_down();
}
