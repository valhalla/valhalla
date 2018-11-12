#include "baldr/streetnames_us.h"
#include "baldr/streetname_us.h"
#include "test.h"

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::pair<std::string, bool>>& names) {
  StreetNamesUs street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    if (names.at(x).first != street_name->value())
      throw std::runtime_error("Incorrect street name value");
    if (names.at(x).second != street_name->is_route_number())
      throw std::runtime_error("Incorrect street name is_route_number");
    ++x;
  }
}

void TestListCtor() {
  TryListCtor({{"Main Street", false}});
  TryListCtor({{"Hershey Road", false}, {"PA 743 North", true}});
}

void TryFindCommonStreetNames(const StreetNamesUs& lhs,
                              const StreetNamesUs& rhs,
                              const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonStreetNames(rhs);
  if (computed->ToString() != expected.ToString()) {
    throw std::runtime_error(expected.ToString() +
                             ": Incorrect street names returned from FindCommonStreetNames");
  }
}

void TestFindCommonStreetNames() {
  TryFindCommonStreetNames(StreetNamesUs({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNamesUs({{"Fishburn Road", false}, {"PA 743 North", true}}),
                           StreetNamesUs({{"PA 743 North", true}}));

  TryFindCommonStreetNames(StreetNamesUs({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNamesUs({{"Fishburn Road", false}, {"PA 743", true}}),
                           StreetNamesUs());

  TryFindCommonStreetNames(StreetNamesUs({{"Capital Beltway", false},
                                          {"I 95 South", true},
                                          {"I 495 South", true}}),
                           StreetNamesUs({{"I 95 South", true}}),
                           StreetNamesUs({{"I 95 South", true}}));
}

void TryFindCommonBaseNames(const StreetNamesUs& lhs,
                            const StreetNamesUs& rhs,
                            const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonBaseNames(rhs);
  if (computed->ToString() != expected.ToString()) {
    throw std::runtime_error(expected.ToString() +
                             ": Incorrect street names returned from FindCommonBaseNames");
  }
}

void TestFindCommonBaseNames() {
  TryFindCommonBaseNames(StreetNamesUs({{"Hershey Road", false}, {"PA 743 North", true}}),
                         StreetNamesUs({{"Fishburn Road", false}, {"PA 743 North", true}}),
                         StreetNamesUs({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNamesUs({{"Hershey Road", false}, {"PA 743 North", true}}),
                         StreetNamesUs({{"Fishburn Road", false}, {"PA 743", true}}),
                         StreetNamesUs({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNamesUs({{"Hershey Road", false}, {"PA 743", true}}),
                         StreetNamesUs({{"Fishburn Road", false}, {"PA 743 North", true}}),
                         StreetNamesUs({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNamesUs({{"Hershey Road", false}, {"PA 743", true}}),
                         StreetNamesUs({{"Fishburn Road", false}, {"PA 743", true}}),
                         StreetNamesUs({{"PA 743", true}}));

  TryFindCommonBaseNames(StreetNamesUs({{"Capital Beltway", false},
                                        {"I 95 South", true},
                                        {"I 495 South", true}}),
                         StreetNamesUs({{"I 95 South", true}}),
                         StreetNamesUs({{"I 95 South", true}}));
}

} // namespace

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
