#include "test.h"
#include "valhalla/baldr/streetname.h"
#include "valhalla/baldr/streetnames.h"

#include <vector>
#include <algorithm>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::string>& names) {
  StreetNames street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    if (names.at(x++) != street_name.value())
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
    throw std::runtime_error(expected.ToString() +
        ": Incorrect street names returned from FindCommonStreetNames");
  }
}

void TestFindCommonStreetNames() {
  TryFindCommonStreetNames(GetStreetNames( { "Hershey Road", "PA 743 North" }),
                           GetStreetNames( { "Fishburn Road", "PA 743 North" }),
                           GetStreetNames( { "PA 743 North" }));

  TryFindCommonStreetNames(GetStreetNames( { "Hershey Road", "PA 743 North" }),
                           GetStreetNames( { "Fishburn Road", "PA 743" }),
                           GetStreetNames( { }));

  TryFindCommonStreetNames(GetStreetNames( { "Capital Beltway", "I 95 South",
      "I 495 South" }),
                           GetStreetNames( { "I 95 South" }), GetStreetNames( {
                               "I 95 South" }));

}

void TryFindCommonBaseNames(const StreetNames& lhs, const StreetNames& rhs,
                            const StreetNames& expected) {
  StreetNames computed = lhs.FindCommonBaseNames(rhs);
  if (computed != expected) {
    throw std::runtime_error(expected.ToString() +
        ": Incorrect street names returned from FindCommonBaseNames");
  }
}

void TestFindCommonBaseNames() {
  TryFindCommonBaseNames(GetStreetNames( { "Hershey Road", "PA 743 North" }),
                         GetStreetNames( { "Fishburn Road", "PA 743 North" }),
                         GetStreetNames( { "PA 743 North" }));

  TryFindCommonBaseNames(GetStreetNames( { "Hershey Road", "PA 743 North" }),
                         GetStreetNames( { "Fishburn Road", "PA 743" }),
                         GetStreetNames( { "PA 743 North" }));

  TryFindCommonBaseNames(GetStreetNames( { "Hershey Road", "PA 743" }),
                         GetStreetNames( { "Fishburn Road", "PA 743 North" }),
                         GetStreetNames( { "PA 743 North" }));

  TryFindCommonBaseNames(GetStreetNames( { "Hershey Road", "PA 743" }),
                         GetStreetNames( { "Fishburn Road", "PA 743" }),
                         GetStreetNames( { "PA 743" }));

  TryFindCommonBaseNames(GetStreetNames( { "Capital Beltway", "I 95 South",
      "I 495 South" }),
                         GetStreetNames( { "I 95 South" }), GetStreetNames( {
                             "I 95 South" }));

}

}

int main() {
  test::suite suite("streetnames");

  // Constructor with list argument
  suite.test(TEST_CASE(TestListCtor));

  // FindCommonStreetNames
  suite.test(TEST_CASE(TestFindCommonStreetNames));

  // FindCommonBaseNames
  suite.test(TEST_CASE(TestFindCommonBaseNames));

  return suite.tear_down();
}
