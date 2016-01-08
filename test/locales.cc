#include "test.h"
#include "odin/util.h"

using namespace valhalla::odin;


namespace {

  void test_init_get() {
    const auto& init = get_locales("./test/data/locales");
    if(init.size() != 1)
      throw std::runtime_error("Should be only one parsable test json file");
    if(init.find("success") == init.cend())
      throw std::runtime_error("Only parsable test json file should be named 'success'");

    const auto& get = get_locales();
    if(&init != &get)
      throw std::runtime_error("The first singleton should be the same as subsequent singletons");
  }

}

int main() {
  test::suite suite("sign");

  // initializing and getting
  suite.test(TEST_CASE(test_init_get));

  return suite.tear_down();
}
