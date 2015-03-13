#include "test.h"
#include "valhalla/baldr/streetname.h"

#include <vector>
#include <algorithm>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryCtor(const std::string& text) {
  StreetName street_name(text);

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
  StreetName lhs(text);
  StreetName rhs(text);

  if (!(lhs == rhs))
    throw std::runtime_error("Incorrect equals return");

}

void TestEquals() {
  TryEquals("Main Street");
  TryEquals("PA 743");
  TryEquals("US 220 Business");
  TryEquals("I 81 South");

}

void TryStartsWith(const StreetName& street_name, const std::string& prefix) {
  if (!street_name.StartsWith(prefix))
    throw std::runtime_error(street_name.value() + ": Incorrect StartsWith");
}

void TestStartsWith() {
  TryStartsWith(StreetName("I 81 South"), "I ");
  TryStartsWith(StreetName("North Main Street"), "North");
}

void TryEndsWith(const StreetName& street_name, const std::string& suffix) {
  if (!street_name.EndsWith(suffix))
    throw std::runtime_error(street_name.value() + ": Incorrect EndsWith");
}

void TestEndsWith() {
  TryEndsWith(StreetName("I 81 South"), "South");
  TryEndsWith(StreetName("Main Street"), "Street");
}

void TryGetPreDir(const StreetName& street_name, const std::string& pre_dir) {
  if (pre_dir != street_name.GetPreDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPreDir");
}

void TestGetPreDir() {
  TryGetPreDir(StreetName("North Main Street"), "North ");
  TryGetPreDir(StreetName("East Chestnut Avenue"), "East ");
  TryGetPreDir(StreetName("South Main Street"), "South ");
  TryGetPreDir(StreetName("West 26th Street"), "West ");
  TryGetPreDir(StreetName("Main Street"), "");
}

void TryGetPostDir(const StreetName& street_name, const std::string& post_dir) {
  if (post_dir != street_name.GetPostDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPostDir");
}

void TestGetPostDir() {
  TryGetPostDir(StreetName("US 220 North"), " North");
  TryGetPostDir(StreetName("US 22 East"), " East");
  TryGetPostDir(StreetName("I 81 South"), " South");
  TryGetPostDir(StreetName("PA 283 West"), " West");
  TryGetPostDir(StreetName("Constitution Avenue Northeast"), " Northeast");
  TryGetPostDir(StreetName("Constitution Avenue Northwest"), " Northwest");
  TryGetPostDir(StreetName("Independence Avenue Southeast"), " Southeast");
  TryGetPostDir(StreetName("Independence Avenue Southwest"), " Southwest");
  TryGetPostDir(StreetName("Main Street"), "");
}

void TryGetPostCardinalDir(const StreetName& street_name,
                           const std::string& post_dir) {
  if (post_dir != street_name.GetPostCardinalDir())
    throw std::runtime_error(
        street_name.value() + ": Incorrect GetPostCardinalDir");
}

void TestGetPostCardinalDir() {
  TryGetPostCardinalDir(StreetName("US 220 North"), " North");
  TryGetPostCardinalDir(StreetName("US 22 East"), " East");
  TryGetPostCardinalDir(StreetName("I 81 South"), " South");
  TryGetPostCardinalDir(StreetName("PA 283 West"), " West");
  TryGetPostCardinalDir(StreetName("Main Street"), "");
}

void TryGetBaseName(const StreetName& street_name,
                    const std::string& base_name) {
  if (base_name != street_name.GetBaseName())
    throw std::runtime_error(street_name.value() + ": Incorrect GetBaseName");
}

void TestGetBaseName() {
  TryGetBaseName(StreetName("North Main Street"), "Main Street");
  TryGetBaseName(StreetName("East Chestnut Avenue"), "Chestnut Avenue");
  TryGetBaseName(StreetName("South Main Street"), "Main Street");
  TryGetBaseName(StreetName("West 26th Street"), "26th Street");
  TryGetBaseName(StreetName("US 220 North"), "US 220");
  TryGetBaseName(StreetName("US 22 East"), "US 22");
  TryGetBaseName(StreetName("I 81 South"), "I 81");
  TryGetBaseName(StreetName("PA 283 West"), "PA 283");
  TryGetBaseName(StreetName("Constitution Avenue Northeast"),
                 "Constitution Avenue");
  TryGetBaseName(StreetName("Constitution Avenue Northwest"),
                 "Constitution Avenue");
  TryGetBaseName(StreetName("Independence Avenue Southeast"),
                 "Independence Avenue");
  TryGetBaseName(StreetName("Independence Avenue Southwest"),
                 "Independence Avenue");
  TryGetBaseName(StreetName("North South Street Northwest"), "South Street");
  TryGetBaseName(StreetName("East North Avenue Southwest"), "North Avenue");
  TryGetBaseName(StreetName("Main Street"), "Main Street");
  TryGetBaseName(StreetName("Broadway"), "Broadway");
  TryGetBaseName(StreetName(""), "");
}

void TryHasSameBaseName(const StreetName& street_name, const StreetName& rhs) {
  if (!street_name.HasSameBaseName(rhs)) {
    throw std::runtime_error(
        street_name.value() + ": Incorrect HasSameBaseName");
  }
}

void TestHasSameBaseName() {
  TryHasSameBaseName(StreetName("North Main Street"),
                     StreetName("Main Street"));
  TryHasSameBaseName(StreetName("East Chestnut Avenue"),
                     StreetName("Chestnut Avenue"));
  TryHasSameBaseName(StreetName("South Main Street"),
                     StreetName("Main Street"));
  TryHasSameBaseName(StreetName("West 26th Street"),
                     StreetName("East 26th Street"));
  TryHasSameBaseName(StreetName("I 695 West"), StreetName("I 695 South"));
  TryHasSameBaseName(StreetName("US 220 North"), StreetName("US 220"));
  TryHasSameBaseName(StreetName("US 22 East"), StreetName("US 22"));
  TryHasSameBaseName(StreetName("I 81 South"), StreetName("I 81"));
  TryHasSameBaseName(StreetName("PA 283 West"), StreetName("PA 283"));
  TryHasSameBaseName(StreetName("Constitution Avenue Northeast"),
                     StreetName("Constitution Avenue"));
  TryHasSameBaseName(StreetName("Constitution Avenue Northwest"),
                     StreetName("Constitution Avenue"));
  TryHasSameBaseName(StreetName("Constitution Avenue Northwest"),
                     StreetName("Constitution Avenue Northeast"));
  TryHasSameBaseName(StreetName("Independence Avenue Southeast"),
                     StreetName("Independence Avenue"));
  TryHasSameBaseName(StreetName("Independence Avenue Southwest"),
                     StreetName("Independence Avenue"));
  TryHasSameBaseName(StreetName("Independence Avenue Southwest"),
                     StreetName("Independence Avenue Southeast"));
  TryHasSameBaseName(StreetName("Main Street"), StreetName("Main Street"));
  TryHasSameBaseName(StreetName("Broadway"), StreetName("Broadway"));
  TryHasSameBaseName(StreetName(""), StreetName(""));
}

}

int main() {
  test::suite suite("streetname");

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
