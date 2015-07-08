#include <cstdlib>
#include <iostream>

#include "skadi/sample.h"

int main(int argc, char** argv) {

  if(argc < 2)
    throw std::runtime_error("No data source specified");

  valhalla::skadi::sample sample(argv[1]);
  std::cout << sample.get(std::make_pair(-76.579355, 40.451491)) << std::endl;
  std::cout << sample.get(std::make_pair(-76.473434, 40.597690)) << std::endl;

  return EXIT_SUCCESS;
}
