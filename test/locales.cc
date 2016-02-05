#include "test.h"
#include "odin/util.h"

using namespace valhalla::odin;


namespace {

  void test_init_get() {
    const auto& init = get_locales("./conf/locales");
    if(init.size() < 1)
      throw std::runtime_error("Should be at least one parsable test json file");
    if(init.find("en-US") == init.cend())
      throw std::runtime_error("Should find 'en-US' locales file");

    const auto& get = get_locales();
    if(&init != &get)
      throw std::runtime_error("The first singleton should be the same as subsequent singletons");
  }

}

int main() {
  test::suite suite("locales");

  // initializing and getting
  suite.test(TEST_CASE(test_init_get));

  return suite.tear_down();
}
