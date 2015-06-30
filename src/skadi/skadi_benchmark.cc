#include <cstdlib>
#include <iostream>

#include "skadi/sample.h"

int main(int argc, char** argv) {

  if(argc < 2)
    throw std::runtime_error("No data source specified");

  valhalla::skadi::sample sample(argv[1]);
  std::cout << sample.get(5.329821, 60.385316) << std::endl;

  return EXIT_SUCCESS;
}
