#include "baldr/streetname_us.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryCtor(const std::string& text, const bool is_route_number) {
  StreetNameUs street_name(text, is_route_number);
  EXPECT_EQ(text, street_name.value());
  EXPECT_EQ(is_route_number, street_name.is_route_number());
}

TEST(StreetnameUs, TestCtor) {
  // Street name
  TryCtor("Main Street", false);

  // Ref
  TryCtor("PA 743", true);

  // Ref with post modifier
  TryCtor("US 220 Business", true);

  // Ref with directional
  TryCtor("I 81 South", true);
}

void TryEquals(const std::string& text, const bool is_route_number) {
  StreetNameUs lhs(text, is_route_number);
  StreetNameUs rhs(text, is_route_number);
  EXPECT_EQ(lhs, rhs);
}

TEST(StreetnameUs, TestEquals) {
  TryEquals("Main Street", false);
  TryEquals("PA 743", true);
  TryEquals("US 220 Business", true);
  TryEquals("I 81 South", true);
}

void TryStartsWith(const StreetNameUs& street_name, const std::string& prefix) {
  EXPECT_TRUE(street_name.StartsWith(prefix)) << street_name.value() + " : " + prefix;
}

TEST(StreetnameUs, TestStartsWith) {
  TryStartsWith(StreetNameUs("I 81 South", true), "I ");
  TryStartsWith(StreetNameUs("North Main Street", false), "North");
}

void TryEndsWith(const StreetNameUs& street_name, const std::string& suffix) {
  EXPECT_TRUE(street_name.EndsWith(suffix)) << street_name.value() + " : " << suffix;
}

TEST(StreetnameUs, TestEndsWith) {
  TryEndsWith(StreetNameUs("I 81 South", true), "South");
  TryEndsWith(StreetNameUs("Main Street", false), "Street");
}

void TryGetPreDir(const StreetNameUs& street_name, const std::string& pre_dir) {
  EXPECT_EQ(pre_dir, street_name.GetPreDir()) << street_name.value() + ": Incorrect GetPreDir";
}

TEST(StreetnameUs, TestGetPreDir) {
  TryGetPreDir(StreetNameUs("North Main Street", false), "North ");
  TryGetPreDir(StreetNameUs("East Chestnut Avenue", false), "East ");
  TryGetPreDir(StreetNameUs("South Main Street", false), "South ");
  TryGetPreDir(StreetNameUs("West 26th Street", false), "West ");
  TryGetPreDir(StreetNameUs("Main Street", false), "");
}

void TryGetPostDir(const StreetNameUs& street_name, const std::string& post_dir) {
  EXPECT_EQ(post_dir, street_name.GetPostDir()) << street_name.value() + ": Incorrect GetPostDir";
}

TEST(StreetnameUs, TestGetPostDir) {
  TryGetPostDir(StreetNameUs("US 220 North", true), " North");
  TryGetPostDir(StreetNameUs("US 22 East", true), " East");
  TryGetPostDir(StreetNameUs("I 81 South", true), " South");
  TryGetPostDir(StreetNameUs("PA 283 West", true), " West");
  TryGetPostDir(StreetNameUs("Constitution Avenue Northeast", false), " Northeast");
  TryGetPostDir(StreetNameUs("Constitution Avenue Northwest", false), " Northwest");
  TryGetPostDir(StreetNameUs("Independence Avenue Southeast", false), " Southeast");
  TryGetPostDir(StreetNameUs("Independence Avenue Southwest", false), " Southwest");
  TryGetPostDir(StreetNameUs("Main Street", false), "");
}

void TryGetPostCardinalDir(const StreetNameUs& street_name, const std::string& post_dir) {
  EXPECT_EQ(post_dir, street_name.GetPostCardinalDir())
      << street_name.value() + ": Incorrect GetPostCardinalDir";
}

TEST(StreetnameUs, TestGetPostCardinalDir) {
  TryGetPostCardinalDir(StreetNameUs("US 220 North", true), " North");
  TryGetPostCardinalDir(StreetNameUs("US 22 East", true), " East");
  TryGetPostCardinalDir(StreetNameUs("I 81 South", true), " South");
  TryGetPostCardinalDir(StreetNameUs("PA 283 West", true), " West");
  TryGetPostCardinalDir(StreetNameUs("Main Street", false), "");
}

void TryGetBaseName(const StreetNameUs& street_name, const std::string& base_name) {
  EXPECT_EQ(base_name, street_name.GetBaseName()) << street_name.value() + ": Incorrect GetBaseName";
}

TEST(StreetnameUs, TestGetBaseName) {
  TryGetBaseName(StreetNameUs("North Main Street", false), "Main Street");
  TryGetBaseName(StreetNameUs("East Chestnut Avenue", false), "Chestnut Avenue");
  TryGetBaseName(StreetNameUs("South Main Street", false), "Main Street");
  TryGetBaseName(StreetNameUs("West 26th Street", false), "26th Street");
  TryGetBaseName(StreetNameUs("US 220 North", true), "US 220");
  TryGetBaseName(StreetNameUs("US 22 East", true), "US 22");
  TryGetBaseName(StreetNameUs("I 81 South", true), "I 81");
  TryGetBaseName(StreetNameUs("PA 283 West", true), "PA 283");
  TryGetBaseName(StreetNameUs("Constitution Avenue Northeast", false), "Constitution Avenue");
  TryGetBaseName(StreetNameUs("Constitution Avenue Northwest", false), "Constitution Avenue");
  TryGetBaseName(StreetNameUs("Independence Avenue Southeast", false), "Independence Avenue");
  TryGetBaseName(StreetNameUs("Independence Avenue Southwest", false), "Independence Avenue");
  TryGetBaseName(StreetNameUs("North South Street Northwest", false), "South Street");
  TryGetBaseName(StreetNameUs("East North Avenue Southwest", false), "North Avenue");
  TryGetBaseName(StreetNameUs("Main Street", false), "Main Street");
  TryGetBaseName(StreetNameUs("Broadway", false), "Broadway");
  TryGetBaseName(StreetNameUs("", false), "");
}

void TryHasSameBaseName(const StreetNameUs& street_name, const StreetNameUs& rhs) {
  EXPECT_TRUE(street_name.HasSameBaseName(rhs))
      << street_name.value() + ": Incorrect HasSameBaseName";
}

TEST(StreetnameUs, TestHasSameBaseName) {
  TryHasSameBaseName(StreetNameUs("North Main Street", false), StreetNameUs("Main Street", false));
  TryHasSameBaseName(StreetNameUs("East Chestnut Avenue", false),
                     StreetNameUs("Chestnut Avenue", false));
  TryHasSameBaseName(StreetNameUs("South Main Street", false), StreetNameUs("Main Street", false));
  TryHasSameBaseName(StreetNameUs("West 26th Street", false),
                     StreetNameUs("East 26th Street", false));
  TryHasSameBaseName(StreetNameUs("I 695 West", true), StreetNameUs("I 695 South", true));
  TryHasSameBaseName(StreetNameUs("US 220 North", true), StreetNameUs("US 220", true));
  TryHasSameBaseName(StreetNameUs("US 22 East", true), StreetNameUs("US 22", true));
  TryHasSameBaseName(StreetNameUs("I 81 South", true), StreetNameUs("I 81", true));
  TryHasSameBaseName(StreetNameUs("PA 283 West", true), StreetNameUs("PA 283", true));
  TryHasSameBaseName(StreetNameUs("Constitution Avenue Northeast", false),
                     StreetNameUs("Constitution Avenue", false));
  TryHasSameBaseName(StreetNameUs("Constitution Avenue Northwest", false),
                     StreetNameUs("Constitution Avenue", false));
  TryHasSameBaseName(StreetNameUs("Constitution Avenue Northwest", false),
                     StreetNameUs("Constitution Avenue Northeast", false));
  TryHasSameBaseName(StreetNameUs("Independence Avenue Southeast", false),
                     StreetNameUs("Independence Avenue", false));
  TryHasSameBaseName(StreetNameUs("Independence Avenue Southwest", false),
                     StreetNameUs("Independence Avenue", false));
  TryHasSameBaseName(StreetNameUs("Independence Avenue Southwest", false),
                     StreetNameUs("Independence Avenue Southeast", false));
  TryHasSameBaseName(StreetNameUs("Main Street", false), StreetNameUs("Main Street", false));
  TryHasSameBaseName(StreetNameUs("Broadway", false), StreetNameUs("Broadway", false));
  TryHasSameBaseName(StreetNameUs("", false), StreetNameUs("", false));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
