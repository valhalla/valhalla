#include "baldr/streetname.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryCtor(const std::string& text, const bool is_route_number) {
  StreetName street_name(text, is_route_number);
  EXPECT_EQ(text, street_name.value());
  EXPECT_EQ(is_route_number, street_name.is_route_number());
}

TEST(Streetname, TestCtor) {
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
  StreetName lhs(text, is_route_number);
  StreetName rhs(text, is_route_number);
  EXPECT_EQ(lhs, rhs);
}

TEST(Streetname, TestEquals) {
  TryEquals("Main Street", false);
  TryEquals("PA 743", true);
  TryEquals("US 220 Business", true);
  TryEquals("I 81 South", true);
  TryEquals("Mittelstraße", false);
}

void TryStartsWith(const StreetName& street_name, const std::string& prefix) {
  EXPECT_TRUE(street_name.StartsWith(prefix)) << street_name.value() + ": Incorrect StartsWith";
}

TEST(Streetname, TestStartsWith) {
  TryStartsWith(StreetName("I 81 South", true), "I ");
  TryStartsWith(StreetName("North Main Street", false), "North");
}

void TryEndsWith(const StreetName& street_name, const std::string& suffix) {
  EXPECT_TRUE(street_name.EndsWith(suffix)) << street_name.value() + ": Incorrect EndsWith";
}

TEST(Streetname, TestEndsWith) {
  TryEndsWith(StreetName("I 81 South", true), "South");
  TryEndsWith(StreetName("Main Street", false), "Street");
}

void TryGetPreDir(const StreetName& street_name, const std::string& pre_dir) {
  EXPECT_EQ(pre_dir, street_name.GetPreDir()) << street_name.value() + ": Incorrect GetPreDir";
}

TEST(Streetname, TestGetPreDir) {
  TryGetPreDir(StreetName("North Main Street", false), "");
  TryGetPreDir(StreetName("Main Street", false), "");
}

void TryGetPostDir(const StreetName& street_name, const std::string& post_dir) {
  EXPECT_EQ(post_dir, street_name.GetPostDir()) << street_name.value() + ": Incorrect GetPostDir";
}

TEST(Streetname, TestGetPostDir) {
  TryGetPostDir(StreetName("I 81 South", true), "");
  TryGetPostDir(StreetName("Main Street", true), "");
}

void TryGetPostCardinalDir(const StreetName& street_name, const std::string& post_dir) {
  EXPECT_EQ(post_dir, street_name.GetPostCardinalDir())
      << street_name.value() + ": Incorrect GetPostCardinalDir";
}

TEST(Streetname, TestGetPostCardinalDir) {
  TryGetPostCardinalDir(StreetName("US 220 North", true), "");
  TryGetPostCardinalDir(StreetName("Main Street", false), "");
}

void TryGetBaseName(const StreetName& street_name, const std::string& base_name) {
  EXPECT_EQ(base_name, street_name.GetBaseName()) << street_name.value() + ": Incorrect GetBaseName";
}

TEST(Streetname, TestGetBaseName) {
  TryGetBaseName(StreetName("North Main Street", false), "North Main Street");
  TryGetBaseName(StreetName("Main Street", false), "Main Street");
  TryGetBaseName(StreetName("Broadway", false), "Broadway");
  TryGetBaseName(StreetName("", false), "");
}

void TryHasSameBaseName(const StreetName& street_name, const StreetName& rhs) {
  EXPECT_TRUE(street_name.HasSameBaseName(rhs))
      << street_name.value() + ": Incorrect HasSameBaseName";
}

TEST(Streetname, TestHasSameBaseName) {
  TryHasSameBaseName(StreetName("North Main Street", false), StreetName("North Main Street", false));
  TryHasSameBaseName(StreetName("I 81 South", true), StreetName("I 81 South", true));
  TryHasSameBaseName(StreetName("PA 283 West", true), StreetName("PA 283 West", true));
  TryHasSameBaseName(StreetName("Constitution Avenue Northeast", false),
                     StreetName("Constitution Avenue Northeast", false));
  TryHasSameBaseName(StreetName("Main Street", false), StreetName("Main Street", false));
  TryHasSameBaseName(StreetName("Broadway", false), StreetName("Broadway", false));
  TryHasSameBaseName(StreetName("Mittelstraße", false), StreetName("Mittelstraße", false));
  TryHasSameBaseName(StreetName("", false), StreetName("", false));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
