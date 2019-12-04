#include "test.h"
#include "config.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
using namespace std;

namespace test {

suite::suite(const string& suite_name) : failed(0) {
  cout << "=== Testing " << suite_name << " ===" << endl;
}

void suite::test(const string& test_name, test_function function) {
  cout << setw(32) << test_name << flush;
  try {
    // run the test
    function();
    // it didnt throw
    cout << "  [PASS]" << endl;
  } // it threw so log the issue
  catch (const exception& e) {
    cout << "  [FAIL: " << e.what() << "]" << endl;
    ++failed;
  } // it threw something that wasn't derived from std::exception?
  catch (...) {
    cerr << "  [FAIL: Unexpected error]" << endl;
    throw;
  }
}

int suite::tear_down() {
  cout << "=== Failed " << failed << " tests ===" << endl;
  if (failed > 0) {
    return EXIT_FAILURE;
  } else {
    return EXIT_SUCCESS;
  }
}

} // namespace test
