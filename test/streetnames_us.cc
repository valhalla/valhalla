#include "test.h"
#include "baldr/streetname_us.h"
#include "baldr/streetnames_us.h"

#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::string>& names) {
  StreetNamesUs street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    if (names.at(x++) != street_name->value())
      throw std::runtime_error("Incorrect street name value");
  }

}

void TestListCtor() {
  TryListCtor( { "Main Street" });
  TryListCtor( { "Hershey Road", "PA 743 North" });
}

void TryFindCommonStreetNames(const StreetNamesUs& lhs,
                              const StreetNamesUs& rhs,
                              const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonStreetNames(rhs);
  if (computed->ToString() != expected.ToString()) {
    throw std::runtime_error(
        expected.ToString()
            + ": Incorrect street names returned from FindCommonStreetNames");
  }
}

void TestFindCommonStreetNames() {
  TryFindCommonStreetNames(StreetNamesUs( { "Hershey Road", "PA 743 North" }),
                           StreetNamesUs( { "Fishburn Road", "PA 743 North" }),
                           StreetNamesUs( { "PA 743 North" }));

  TryFindCommonStreetNames(StreetNamesUs( { "Hershey Road", "PA 743 North" }),
                           StreetNamesUs( { "Fishburn Road", "PA 743" }),
                           StreetNamesUs());

  TryFindCommonStreetNames(StreetNamesUs( { "Capital Beltway", "I 95 South",
      "I 495 South" }),
                           StreetNamesUs( { "I 95 South" }), StreetNamesUs( {
                               "I 95 South" }));

}

void TryFindCommonBaseNames(const StreetNamesUs& lhs, const StreetNamesUs& rhs,
                            const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonBaseNames(rhs);
  if (computed->ToString() != expected.ToString()) {
    throw std::runtime_error(
        expected.ToString()
            + ": Incorrect street names returned from FindCommonBaseNames");
  }
}

void TestFindCommonBaseNames() {
  TryFindCommonBaseNames(StreetNamesUs( { "Hershey Road", "PA 743 North" }),
                         StreetNamesUs( { "Fishburn Road", "PA 743 North" }),
                         StreetNamesUs( { "PA 743 North" }));

  TryFindCommonBaseNames(StreetNamesUs( { "Hershey Road", "PA 743 North" }),
                         StreetNamesUs( { "Fishburn Road", "PA 743" }),
                         StreetNamesUs( { "PA 743 North" }));

  TryFindCommonBaseNames(StreetNamesUs( { "Hershey Road", "PA 743" }),
                         StreetNamesUs( { "Fishburn Road", "PA 743 North" }),
                         StreetNamesUs( { "PA 743 North" }));

  TryFindCommonBaseNames(StreetNamesUs( { "Hershey Road", "PA 743" }),
                         StreetNamesUs( { "Fishburn Road", "PA 743" }),
                         StreetNamesUs( { "PA 743" }));

  TryFindCommonBaseNames(StreetNamesUs( { "Capital Beltway", "I 95 South",
      "I 495 South" }),
                         StreetNamesUs( { "I 95 South" }), StreetNamesUs( {
                             "I 95 South" }));

}

}

int main() {
  test::suite suite("streetnames_us");

  // Constructor with list argument
  suite.test(TEST_CASE(TestListCtor));

  // FindCommonStreetNames
  suite.test(TEST_CASE(TestFindCommonStreetNames));

  // FindCommonBaseNames
  suite.test(TEST_CASE(TestFindCommonBaseNames));

  return suite.tear_down();
}
