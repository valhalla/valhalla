// -*- mode: c++ -*-

#ifndef TEST_HPP
#define TEST_HPP

#include <string>
#include <stdexcept>

#define TEST_CASE(x) #x, x

namespace test{

//function pointer to run as a test
using test_function = void (*)(void);

struct suite {
 public:
  //initializes the test suite
  explicit suite(const std::string& test_suite_name);
  //run the test
  void test(const std::string& test_name, test_function function);
  //returns EXIT_FAILURE if any tests failed otherwise returns EXIT_SUCCESS
  int tear_down();
 private:
  //number of failed tests
  size_t failed;
};

template <typename value_t>
void assert_bool(value_t value, const std::string& message)
{
  if (!value) {
    throw std::runtime_error(message);
  }
}

template <typename exception_t, typename function_t>
void assert_throw(function_t func, const std::string& message)
{
  bool thrown = false;

  try {
    func();
  } catch (const exception_t& ex) {
    thrown = true;
  }

  if (!thrown) {
    throw std::runtime_error(message);
  }
}

}

#endif
