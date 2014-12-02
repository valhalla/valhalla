#ifndef TEST_HPP
#define TEST_HPP

#include <string>
#include <stdexcept>
#include <boost/function.hpp>

#define TEST_CASE(x) #x, &(x)

namespace test{

  struct suite {
    public:
      //initializes the test suite
      explicit suite(const std::string& test_suite_name);
      //run the test
      void test(const std::string& test_name, boost::function<void ()> test_function);
      //returns EXIT_FAILURE if any tests failed otherwise returns EXIT_SUCCESS
      int tear_down();
    private:
      //number of failed tests
      size_t failed;
  };



}

#endif
