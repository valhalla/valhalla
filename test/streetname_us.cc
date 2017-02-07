#include "test.h"
#include "baldr/streetname_us.h"

#include <vector>
#include <algorithm>
#include <utility>
#include <memory>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryCtor(const std::string& text) {
  StreetNameUs street_name(text);

  if (text != street_name.value())
    throw std::runtime_error("Incorrect street name text");

}

void TestCtor() {
  // Street name
  TryCtor("Main Street");

  // Ref
  TryCtor("PA 743");

  // Ref with post modifier
  TryCtor("US 220 Business");

  // Ref with directional
  TryCtor("I 81 South");

}

void TryEquals(const std::string& text) {
  StreetNameUs lhs(text);
  StreetNameUs rhs(text);

  if (!(lhs == rhs))
    throw std::runtime_error("Incorrect equals return");

}

void TestEquals() {
  TryEquals("Main Street");
  TryEquals("PA 743");
  TryEquals("US 220 Business");
  TryEquals("I 81 South");

}

void TryStartsWith(const StreetNameUs& street_name, const std::string& prefix) {
  if (!street_name.StartsWith(prefix))
    throw std::runtime_error(street_name.value() + ": Incorrect StartsWith");
}

void TestStartsWith() {
  TryStartsWith(StreetNameUs("I 81 South"), "I ");
  TryStartsWith(StreetNameUs("North Main Street"), "North");
}

void TryEndsWith(const StreetNameUs& street_name, const std::string& suffix) {
  if (!street_name.EndsWith(suffix))
    throw std::runtime_error(street_name.value() + ": Incorrect EndsWith");
}

void TestEndsWith() {
  TryEndsWith(StreetNameUs("I 81 South"), "South");
  TryEndsWith(StreetNameUs("Main Street"), "Street");
}

void TryGetPreDir(const StreetNameUs& street_name, const std::string& pre_dir) {
  if (pre_dir != street_name.GetPreDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPreDir");
}

void TestGetPreDir() {
  TryGetPreDir(StreetNameUs("North Main Street"), "North ");
  TryGetPreDir(StreetNameUs("East Chestnut Avenue"), "East ");
  TryGetPreDir(StreetNameUs("South Main Street"), "South ");
  TryGetPreDir(StreetNameUs("West 26th Street"), "West ");
  TryGetPreDir(StreetNameUs("Main Street"), "");
}

void TryGetPostDir(const StreetNameUs& street_name, const std::string& post_dir) {
  if (post_dir != street_name.GetPostDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPostDir");
}

void TestGetPostDir() {
  TryGetPostDir(StreetNameUs("US 220 North"), " North");
  TryGetPostDir(StreetNameUs("US 22 East"), " East");
  TryGetPostDir(StreetNameUs("I 81 South"), " South");
  TryGetPostDir(StreetNameUs("PA 283 West"), " West");
  TryGetPostDir(StreetNameUs("Constitution Avenue Northeast"), " Northeast");
  TryGetPostDir(StreetNameUs("Constitution Avenue Northwest"), " Northwest");
  TryGetPostDir(StreetNameUs("Independence Avenue Southeast"), " Southeast");
  TryGetPostDir(StreetNameUs("Independence Avenue Southwest"), " Southwest");
  TryGetPostDir(StreetNameUs("Main Street"), "");
}

void TryGetPostCardinalDir(const StreetNameUs& street_name,
                           const std::string& post_dir) {
  if (post_dir != street_name.GetPostCardinalDir())
    throw std::runtime_error(
        street_name.value() + ": Incorrect GetPostCardinalDir");
}

void TestGetPostCardinalDir() {
  TryGetPostCardinalDir(StreetNameUs("US 220 North"), " North");
  TryGetPostCardinalDir(StreetNameUs("US 22 East"), " East");
  TryGetPostCardinalDir(StreetNameUs("I 81 South"), " South");
  TryGetPostCardinalDir(StreetNameUs("PA 283 West"), " West");
  TryGetPostCardinalDir(StreetNameUs("Main Street"), "");
}

void TryGetBaseName(const StreetNameUs& street_name,
                    const std::string& base_name) {
  if (base_name != street_name.GetBaseName())
    throw std::runtime_error(street_name.value() + ": Incorrect GetBaseName");
}

void TestGetBaseName() {
  TryGetBaseName(StreetNameUs("North Main Street"), "Main Street");
  TryGetBaseName(StreetNameUs("East Chestnut Avenue"), "Chestnut Avenue");
  TryGetBaseName(StreetNameUs("South Main Street"), "Main Street");
  TryGetBaseName(StreetNameUs("West 26th Street"), "26th Street");
  TryGetBaseName(StreetNameUs("US 220 North"), "US 220");
  TryGetBaseName(StreetNameUs("US 22 East"), "US 22");
  TryGetBaseName(StreetNameUs("I 81 South"), "I 81");
  TryGetBaseName(StreetNameUs("PA 283 West"), "PA 283");
  TryGetBaseName(StreetNameUs("Constitution Avenue Northeast"),
                 "Constitution Avenue");
  TryGetBaseName(StreetNameUs("Constitution Avenue Northwest"),
                 "Constitution Avenue");
  TryGetBaseName(StreetNameUs("Independence Avenue Southeast"),
                 "Independence Avenue");
  TryGetBaseName(StreetNameUs("Independence Avenue Southwest"),
                 "Independence Avenue");
  TryGetBaseName(StreetNameUs("North South Street Northwest"), "South Street");
  TryGetBaseName(StreetNameUs("East North Avenue Southwest"), "North Avenue");
  TryGetBaseName(StreetNameUs("Main Street"), "Main Street");
  TryGetBaseName(StreetNameUs("Broadway"), "Broadway");
  TryGetBaseName(StreetNameUs(""), "");
}

void TryHasSameBaseName(const StreetNameUs& street_name, const StreetNameUs& rhs) {
  if (!street_name.HasSameBaseName(rhs)) {
    throw std::runtime_error(
        street_name.value() + ": Incorrect HasSameBaseName");
  }
}

void TestHasSameBaseName() {
  TryHasSameBaseName(StreetNameUs("North Main Street"),
                     StreetNameUs("Main Street"));
  TryHasSameBaseName(StreetNameUs("East Chestnut Avenue"),
                     StreetNameUs("Chestnut Avenue"));
  TryHasSameBaseName(StreetNameUs("South Main Street"),
                     StreetNameUs("Main Street"));
  TryHasSameBaseName(StreetNameUs("West 26th Street"),
                     StreetNameUs("East 26th Street"));
  TryHasSameBaseName(StreetNameUs("I 695 West"), StreetNameUs("I 695 South"));
  TryHasSameBaseName(StreetNameUs("US 220 North"), StreetNameUs("US 220"));
  TryHasSameBaseName(StreetNameUs("US 22 East"), StreetNameUs("US 22"));
  TryHasSameBaseName(StreetNameUs("I 81 South"), StreetNameUs("I 81"));
  TryHasSameBaseName(StreetNameUs("PA 283 West"), StreetNameUs("PA 283"));
  TryHasSameBaseName(StreetNameUs("Constitution Avenue Northeast"),
                     StreetNameUs("Constitution Avenue"));
  TryHasSameBaseName(StreetNameUs("Constitution Avenue Northwest"),
                     StreetNameUs("Constitution Avenue"));
  TryHasSameBaseName(StreetNameUs("Constitution Avenue Northwest"),
                     StreetNameUs("Constitution Avenue Northeast"));
  TryHasSameBaseName(StreetNameUs("Independence Avenue Southeast"),
                     StreetNameUs("Independence Avenue"));
  TryHasSameBaseName(StreetNameUs("Independence Avenue Southwest"),
                     StreetNameUs("Independence Avenue"));
  TryHasSameBaseName(StreetNameUs("Independence Avenue Southwest"),
                     StreetNameUs("Independence Avenue Southeast"));
  TryHasSameBaseName(StreetNameUs("Main Street"), StreetNameUs("Main Street"));
  TryHasSameBaseName(StreetNameUs("Broadway"), StreetNameUs("Broadway"));
  TryHasSameBaseName(StreetNameUs(""), StreetNameUs(""));
}

}

int main() {
  test::suite suite("streetname_us");

  // Constructor
  suite.test(TEST_CASE(TestCtor));

  // Equals
  suite.test(TEST_CASE(TestEquals));

  // StartsWith
  suite.test(TEST_CASE(TestStartsWith));

  // EndsWith
  suite.test(TEST_CASE(TestEndsWith));

  // GetPreDir
  suite.test(TEST_CASE(TestGetPreDir));

  // GetPostDir
  suite.test(TEST_CASE(TestGetPostDir));

  // GetPostCardinalDir
  suite.test(TEST_CASE(TestGetPostCardinalDir));

  // GetBaseName
  suite.test(TEST_CASE(TestGetBaseName));

  // HasSameBaseName
  suite.test(TEST_CASE(TestHasSameBaseName));

  return suite.tear_down();
}
