#include "baldr/streetnames_factory.h"
#include "baldr/streetnames.h"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace valhalla::baldr;

namespace {

void TryCreate(const std::string& country_code,
               const std::vector<std::pair<std::string, bool>>& names,
               const std::string& expected) {
  std::unique_ptr<StreetNames> street_names = StreetNamesFactory::Create(country_code, names);

  auto& value = *street_names.get();
  std::string rtti(typeid(value).name());
  EXPECT_EQ(rtti, expected) << "Incorrect object type";
}

TEST(StreetnamesFactory, Create) {
#ifdef _MSC_VER
  // MSVC's typeid().name() returns a demangled string, not the Itanium ABI mangling
  const std::string us_name = "class valhalla::baldr::StreetNamesUs";
  const std::string de_name = "class valhalla::baldr::StreetNames";
#else
  const std::string us_name = "N8valhalla5baldr13StreetNamesUsE";
  const std::string de_name = "N8valhalla5baldr11StreetNamesE";
#endif

  // US - should be StreetNamesUs
  TryCreate("US", {{"Main Street", false}}, us_name);
  TryCreate("US", {{"Hershey Road", false}, {"PA 743 North", true}}, us_name);

  // DE - should be default StreetNames
  TryCreate("DE", {{"Mittelstraße", false}}, de_name);
  TryCreate("DE", {{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}}, de_name);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
