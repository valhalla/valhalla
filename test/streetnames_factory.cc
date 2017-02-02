#include "test.h"
#include "baldr/streetnames.h"
#include "baldr/streetnames_us.h"
#include "baldr/streetnames_factory.h"

#include <vector>
#include <memory>
#include <iostream>
#include <typeinfo>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryCreate(const std::string& country_code,
                 const std::vector<std::string>& names,
                 const std::string& expected) {
  std::unique_ptr<StreetNames> street_names = StreetNamesFactory::Create(
        country_code, names);

  std::string rtti(typeid(*street_names).name());
    if (rtti != expected) {
      throw std::runtime_error(rtti + ": Incorrect object type - expected: " + expected);
  }

}

void TestCreate() {
  // US - should be StreetNamesUs
  TryCreate("US", { "Main Street" }, "N8valhalla5baldr13StreetNamesUsE");
  TryCreate("US", { "Hershey Road", "PA 743 North" },
            "N8valhalla5baldr13StreetNamesUsE");

  // DE - should be default StreetNames
  TryCreate("DE", { "Mittelstraße", }, "N8valhalla5baldr11StreetNamesE");
  TryCreate("DE", { "Unter den Linden", "B 2", "B 5" },
            "N8valhalla5baldr11StreetNamesE");

}

}

int main() {
  test::suite suite("streetnames_factory");

  // Constructor with list argument
  suite.test(TEST_CASE(TestCreate));

  return suite.tear_down();
}
