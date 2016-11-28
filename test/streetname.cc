#include "test.h"
#include "baldr/streetname.h"

#include <vector>
#include <algorithm>
#include <utility>
#include <memory>

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
  TryEquals("Mittelstraße");

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
  TryGetPreDir(StreetName("North Main Street"), "");
  TryGetPreDir(StreetName("Main Street"), "");
}

void TryGetPostDir(const StreetName& street_name, const std::string& post_dir) {
  if (post_dir != street_name.GetPostDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPostDir");
}

void TestGetPostDir() {
  TryGetPostDir(StreetName("I 81 South"), "");
  TryGetPostDir(StreetName("Main Street"), "");
}

void TryGetPostCardinalDir(const StreetName& street_name,
                           const std::string& post_dir) {
  if (post_dir != street_name.GetPostCardinalDir())
    throw std::runtime_error(
        street_name.value() + ": Incorrect GetPostCardinalDir");
}

void TestGetPostCardinalDir() {
  TryGetPostCardinalDir(StreetName("US 220 North"), "");
  TryGetPostCardinalDir(StreetName("Main Street"), "");
}

void TryGetBaseName(const StreetName& street_name,
                    const std::string& base_name) {
  if (base_name != street_name.GetBaseName())
    throw std::runtime_error(street_name.value() + ": Incorrect GetBaseName");
}

void TestGetBaseName() {
  TryGetBaseName(StreetName("North Main Street"), "North Main Street");
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
                     StreetName("North Main Street"));
  TryHasSameBaseName(StreetName("I 81 South"), StreetName("I 81 South"));
  TryHasSameBaseName(StreetName("PA 283 West"), StreetName("PA 283 West"));
  TryHasSameBaseName(StreetName("Constitution Avenue Northeast"),
                     StreetName("Constitution Avenue Northeast"));
  TryHasSameBaseName(StreetName("Main Street"), StreetName("Main Street"));
  TryHasSameBaseName(StreetName("Broadway"), StreetName("Broadway"));
  TryHasSameBaseName(StreetName("Mittelstraße"), StreetName("Mittelstraße"));
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
