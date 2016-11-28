#include "test.h"
#include "baldr/streetname.h"
#include "baldr/streetnames.h"

#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::string>& names) {
  StreetNames street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    if (names.at(x++) != street_name->value())
      throw std::runtime_error("Incorrect street name value");
  }

}

void TestListCtor() {
  TryListCtor( { "Main Street" });
  TryListCtor( { "Hershey Road", "PA 743 North" });
  TryListCtor( { "Unter den Linden", "B 2", "B 5" });
}

void TryFindCommonStreetNames(const StreetNames& lhs,
                              const StreetNames& rhs,
                              const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonStreetNames(rhs);
  if (computed->ToString() != expected.ToString()) {
    throw std::runtime_error(
        expected.ToString()
            + ": Incorrect street names returned from FindCommonStreetNames");
  }
}

void TestFindCommonStreetNames() {
  TryFindCommonStreetNames(StreetNames( { "Hershey Road", "PA 743 North" }),
                           StreetNames( { "Fishburn Road", "PA 743 North" }),
                           StreetNames( { "PA 743 North" }));

  TryFindCommonStreetNames(StreetNames( { "Hershey Road", "PA 743 North" }),
                           StreetNames( { "Fishburn Road", "PA 743" }),
                           StreetNames());

  TryFindCommonStreetNames(StreetNames( { "Capital Beltway", "I 95 South",
      "I 495 South" }),
                           StreetNames( { "I 95 South" }), StreetNames( {
                               "I 95 South" }));

  TryFindCommonStreetNames(StreetNames( { "Unter den Linden", "B 2", "B 5" }),
                           StreetNames( { "B 2", "B 5" }),
                           StreetNames( { "B 2", "B 5" }));

}

void TryFindCommonBaseNames(const StreetNames& lhs, const StreetNames& rhs,
                            const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonBaseNames(rhs);
  if (computed->ToString() != expected.ToString()) {
    throw std::runtime_error(
        expected.ToString()
            + ": Incorrect street names returned from FindCommonBaseNames");
  }
}

void TestFindCommonBaseNames() {
  TryFindCommonBaseNames(StreetNames( { "Hershey Road", "PA 743 North" }),
                         StreetNames( { "Fishburn Road", "PA 743 North" }),
                         StreetNames( { "PA 743 North" }));

  TryFindCommonBaseNames(StreetNames( { "Unter den Linden", "B 2", "B 5" }),
                         StreetNames( { "B 2", "B 5" }),
                         StreetNames( { "B 2", "B 5" }));

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
