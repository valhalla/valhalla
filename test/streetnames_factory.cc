#include "baldr/streetnames_factory.h"
#include "baldr/streetnames.h"
#include "baldr/streetnames_us.h"
#include "test.h"

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
  // US - should be StreetNamesUs
  TryCreate("US", {{"Main Street", false}}, "N8valhalla5baldr13StreetNamesUsE");
  TryCreate("US", {{"Hershey Road", false}, {"PA 743 North", true}},
            "N8valhalla5baldr13StreetNamesUsE");

  // DE - should be default StreetNames
  TryCreate("DE", {{"Mittelstraße", false}}, "N8valhalla5baldr11StreetNamesE");
  TryCreate("DE", {{"Unter den Linden", false}, {"B 2", true}, {"B 5", true}},
            "N8valhalla5baldr11StreetNamesE");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
