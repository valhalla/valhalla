#include "test.h"
#include "valhalla/odin/streetname.h"
#include "valhalla/odin/streetnames.h"

#include <google/protobuf/repeated_field.h>

#include <vector>
#include <algorithm>

using namespace std;
using namespace valhalla::odin;

namespace {

void TryListCtor(const std::vector<::std::string>& names) {
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

StreetNames GetStreetNames(const std::vector<std::string>& names) {
  StreetNames street_names;
  for (const auto& name : names) {
    street_names.emplace_back(name);
  }
  return street_names;
}
void TryFindCommonStreetNames(const StreetNames& lhs, const StreetNames& rhs,
                              const StreetNames& expected) {
  StreetNames computed = lhs.FindCommonStreetNames(rhs);
  if (computed != expected) {
    throw std::runtime_error(
        "Incorrect street names returned from FindCommonStreetNames");
  }
}

void TestFindCommonStreetNames() {
  TryFindCommonStreetNames(GetStreetNames( { "Hershey Road", "PA 743 North" }),
                           GetStreetNames( { "Fishburn Road", "PA 743 North" }),
                           GetStreetNames( { "PA 743 North" }));

  TryFindCommonStreetNames(GetStreetNames( { "Hershey Road", "PA 743 North" }),
                           GetStreetNames( { "Fishburn Road", "PA 743" }),
                           GetStreetNames( { }));

  TryFindCommonStreetNames(GetStreetNames( { "Capital Beltway", "I 95 South", "I 495 South" }),
                           GetStreetNames( { "I 95 South" }),
                           GetStreetNames( { "I 95 South" }));

}

}

int main() {
  test::suite suite("sign");

  // Constructor with list argument
  suite.test(TEST_CASE(TestListCtor));

  // FindCommonStreetNames
  suite.test(TEST_CASE(TestFindCommonStreetNames));

  return suite.tear_down();
}
