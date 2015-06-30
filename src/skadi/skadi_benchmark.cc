#include <cstdlib>
#include <iostream>

#include "skadi/sample.h"

int main(int argv, char** argvc) {

  valhalla::skadi::sample sample("/data/elevation/srtm/srtm.vrt");
  sample.get(5.329821, 60.385316);

  return EXIT_SUCCESS;
}
