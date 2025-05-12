#include "baldr/streetnames_us.h"
#include "baldr/streetname_us.h"

#include <vector>

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryListCtor(const std::vector<std::pair<std::string, bool>>& names) {
  StreetNamesUs street_names(names);

  int x = 0;
  for (const auto& street_name : street_names) {
    EXPECT_EQ(names.at(x).first, street_name->value());
    EXPECT_EQ(names.at(x).second, street_name->is_route_number());
    ++x;
  }
}

TEST(StreetnamesUs, TestListCtor) {
  TryListCtor({{"Main Street", false}});
  TryListCtor({{"Hershey Road", false}, {"PA 743 North", true}});
}

void TryFindCommonStreetNames(const StreetNamesUs& lhs,
                              const StreetNamesUs& rhs,
                              const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = lhs.FindCommonStreetNames(rhs);
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "FindCommonStreetNames";
}

TEST(StreetnamesUs, TestFindCommonStreetNames) {
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
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "FindCommonBaseNames";
}

TEST(StreetnamesUs, TestFindCommonBaseNames) {
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

void TryGetRouteNumbers(const StreetNamesUs& street_names, const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "GetRouteNumbers";
}

TEST(StreetnamesUs, TestGetRouteNumbers) {
  TryGetRouteNumbers(StreetNamesUs({{"Hershey Road", false}, {"PA 743 North", true}}),
                     StreetNamesUs({{"PA 743 North", true}}));

  TryGetRouteNumbers(StreetNamesUs({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                     StreetNamesUs({{"B 2", true}, {"B 5", true}}));

  TryGetRouteNumbers(StreetNamesUs({{"I 95 South", true}}), StreetNamesUs({{"I 95 South", true}}));

  TryGetRouteNumbers(StreetNamesUs({{"Sheridan Circle", false}}), StreetNamesUs());

  TryGetRouteNumbers(StreetNamesUs(
                         {{"Capital Beltway", false}, {"I 95 South", true}, {"I 495 South", true}}),
                     StreetNamesUs({{"I 95 South", true}, {"I 495 South", true}}));
}

void TryGetNonRouteNumbers(const StreetNamesUs& street_names, const StreetNamesUs& expected) {
  std::unique_ptr<StreetNames> computed = street_names.GetNonRouteNumbers();
  EXPECT_EQ(computed->ToString(), expected.ToString()) << "GetNonRouteNumbers";
}

TEST(StreetnamesUs, TestGetNonRouteNumbers) {
  TryGetNonRouteNumbers(StreetNamesUs({{"Hershey Road", false}, {"PA 743 North", true}}),
                        StreetNamesUs({{"Hershey Road", false}}));

  TryGetNonRouteNumbers(StreetNamesUs({{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}),
                        StreetNamesUs({{"Unter den Linden", false}}));

  TryGetNonRouteNumbers(StreetNamesUs({{"I 95 South", true}}), StreetNamesUs());

  TryGetNonRouteNumbers(StreetNamesUs({{"Sheridan Circle", false}}),
                        StreetNamesUs({{"Sheridan Circle", false}}));

  TryGetNonRouteNumbers(StreetNamesUs({{"Capital Beltway", false},
                                       {"I 95 South", true},
                                       {"I 495 South", true}}),
                        StreetNamesUs({{"Capital Beltway", false}}));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
