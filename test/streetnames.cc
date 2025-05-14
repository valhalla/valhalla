#include "baldr/streetnames.h"
#include "baldr/streetname.h"
#include "test.h"

#include <vector>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::pair<std::string, bool>>& names) {
  StreetNames street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    EXPECT_EQ(names.at(x).first, street_name->value());
    EXPECT_EQ(names.at(x).second, street_name->is_route_number());
    ++x;
  }
}

TEST(Streetnames, TestListCtor) {
  TryListCtor({{"Main Street", false}});
  TryListCtor({{"Hershey Road", false}, {"PA 743 North", true}});
  TryListCtor({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}});
}

void TryFindCommonStreetNames(const StreetNames& lhs,
                              const StreetNames& rhs,
                              const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonStreetNames(rhs);
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect street names returned from FindCommonStreetNames";
}

TEST(Streetnames, TestFindCommonStreetNames) {
  TryFindCommonStreetNames(StreetNames({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNames({{"Fishburn Road", false}, {"PA 743 North", true}}),
                           StreetNames({{"PA 743 North", true}}));

  TryFindCommonStreetNames(StreetNames({{"Hershey Road", false}, {"PA 743 North", true}}),
                           StreetNames({{"Fishburn Road", false}, {"PA 743", true}}), StreetNames());

  TryFindCommonStreetNames(StreetNames({{"Capital Beltway", false},
                                        {"I 95 South", true},
                                        {"I 495 South", true}}),
                           StreetNames({{"I 95 South", true}}), StreetNames({{"I 95 South", true}}));

  TryFindCommonStreetNames(StreetNames({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                           StreetNames({{"B 2", true}, {"B 5", true}}),
                           StreetNames({{"B 2", true}, {"B 5", true}}));
}

void TryFindCommonBaseNames(const StreetNames& lhs,
                            const StreetNames& rhs,
                            const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonBaseNames(rhs);
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect street names returned from FindCommonBaseNames";
}

TEST(Streetnames, TestFindCommonBaseNames) {
  TryFindCommonBaseNames(StreetNames({{"Hershey Road", false}, {"PA 743 North", true}}),
                         StreetNames({{"Fishburn Road", false}, {"PA 743 North", true}}),
                         StreetNames({{"PA 743 North", true}}));

  TryFindCommonBaseNames(StreetNames({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                         StreetNames({{"B 2", true}, {"B 5", true}}),
                         StreetNames({{"B 2", true}, {"B 5", true}}));
}

void TryGetRouteNumbers(const StreetNames& street_names, const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect values returned from GetRouteNumbers";
}

TEST(Streetnames, TestGetRouteNumbers) {
  TryGetRouteNumbers(StreetNames({{"Hershey Road", false}, {"PA 743 North", true}}),
                     StreetNames({{"PA 743 North", true}}));

  TryGetRouteNumbers(StreetNames({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                     StreetNames({{"B 2", true}, {"B 5", true}}));

  TryGetRouteNumbers(StreetNames({{"I 95 South", true}}), StreetNames({{"I 95 South", true}}));

  TryGetRouteNumbers(StreetNames({{"Sheridan Circle", false}}), StreetNames());
}

void TryGetNonRouteNumbers(const StreetNames& street_names, const StreetNames& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetNonRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString())
      << "Incorrect values returned from GetNonRouteNumbers";
}

TEST(Streetnames, TestGetNonRouteNumbers) {
  TryGetNonRouteNumbers(StreetNames({{"Hershey Road", false}, {"PA 743 North", true}}),
                        StreetNames({{"Hershey Road", false}}));

  TryGetNonRouteNumbers(StreetNames({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                        StreetNames({{"Unter den Linden", false}}));

  TryGetNonRouteNumbers(StreetNames({{"I 95 South", true}}), StreetNames());

  TryGetNonRouteNumbers(StreetNames({{"Sheridan Circle", false}}),
                        StreetNames({{"Sheridan Circle", false}}));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
