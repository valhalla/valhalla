#include "test.h"
#include "valhalla/baldr/streetnames.h"
#include "valhalla/baldr/streetnames_us.h"
#include "valhalla/baldr/streetnames_factory.h"

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
  TryCreate("US", { "Main Street" }, "N8valhalla5baldr13StreetNamesUsE");
  TryCreate("US", { "Hershey Road", "PA 743 North" },
            "N8valhalla5baldr13StreetNamesUsE");
}

}

int main() {
  test::suite suite("streetnames_us");

  // Constructor with list argument
  suite.test(TEST_CASE(TestCreate));

  return suite.tear_down();
}
