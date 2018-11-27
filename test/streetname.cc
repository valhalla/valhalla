#include "baldr/streetname.h"
#include "test.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryCtor(const std::string& text, const bool is_route_number) {
  StreetName street_name(text, is_route_number);

  if (text != street_name.value())
    throw std::runtime_error("Incorrect street name value");
  if (is_route_number != street_name.is_route_number())
    throw std::runtime_error("Incorrect street name is_route_number");
}

void TestCtor() {
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

  if (!(lhs == rhs))
    throw std::runtime_error("Incorrect equals return");
}

void TestEquals() {
  TryEquals("Main Street", false);
  TryEquals("PA 743", true);
  TryEquals("US 220 Business", true);
  TryEquals("I 81 South", true);
  TryEquals("Mittelstraße", false);
}

void TryStartsWith(const StreetName& street_name, const std::string& prefix) {
  if (!street_name.StartsWith(prefix))
    throw std::runtime_error(street_name.value() + ": Incorrect StartsWith");
}

void TestStartsWith() {
  TryStartsWith(StreetName("I 81 South", true), "I ");
  TryStartsWith(StreetName("North Main Street", false), "North");
}

void TryEndsWith(const StreetName& street_name, const std::string& suffix) {
  if (!street_name.EndsWith(suffix))
    throw std::runtime_error(street_name.value() + ": Incorrect EndsWith");
}

void TestEndsWith() {
  TryEndsWith(StreetName("I 81 South", true), "South");
  TryEndsWith(StreetName("Main Street", false), "Street");
}

void TryGetPreDir(const StreetName& street_name, const std::string& pre_dir) {
  if (pre_dir != street_name.GetPreDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPreDir");
}

void TestGetPreDir() {
  TryGetPreDir(StreetName("North Main Street", false), "");
  TryGetPreDir(StreetName("Main Street", false), "");
}

void TryGetPostDir(const StreetName& street_name, const std::string& post_dir) {
  if (post_dir != street_name.GetPostDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPostDir");
}

void TestGetPostDir() {
  TryGetPostDir(StreetName("I 81 South", true), "");
  TryGetPostDir(StreetName("Main Street", true), "");
}

void TryGetPostCardinalDir(const StreetName& street_name, const std::string& post_dir) {
  if (post_dir != street_name.GetPostCardinalDir())
    throw std::runtime_error(street_name.value() + ": Incorrect GetPostCardinalDir");
}

void TestGetPostCardinalDir() {
  TryGetPostCardinalDir(StreetName("US 220 North", true), "");
  TryGetPostCardinalDir(StreetName("Main Street", false), "");
}

void TryGetBaseName(const StreetName& street_name, const std::string& base_name) {
  if (base_name != street_name.GetBaseName())
    throw std::runtime_error(street_name.value() + ": Incorrect GetBaseName");
}

void TestGetBaseName() {
  TryGetBaseName(StreetName("North Main Street", false), "North Main Street");
  TryGetBaseName(StreetName("Main Street", false), "Main Street");
  TryGetBaseName(StreetName("Broadway", false), "Broadway");
  TryGetBaseName(StreetName("", false), "");
}

void TryHasSameBaseName(const StreetName& street_name, const StreetName& rhs) {
  if (!street_name.HasSameBaseName(rhs)) {
    throw std::runtime_error(street_name.value() + ": Incorrect HasSameBaseName");
  }
}

void TestHasSameBaseName() {
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
