#include "test.h"

#include <string>
#include <vector>
#include "valhalla/skadi/service.h"

using namespace std;
using namespace valhalla::midgard;
using namespace valhalla::skadi;

namespace {

void TryGetElevationFromLatLng(std::vector<PointLL&> shape, uint32_t expected_arraysize) {
  if (elevation(shape) != expected_arraysize) {
    throw std::runtime_error(
        std::string("No Elevation Data available."));
  }
}


void TryGetElevationFromEncodedPolyline(std::string encoded_shape, uint32_t expected_arraysize) {
  if (elevation(encoded_shape) != expected_arraysize) {
    throw std::runtime_error(
        std::string("No Elevation Data available."));
  }
}


void TestGetElevationFromLatLng() {
  TryGetElevationFromLatLon([{"lat":40.712431, "lon":-74.004916},
      {"lat":40.712275, "lon":-74.005259},
      {"lat":40.712122, "lon":-74.005694},
      {"lat":40.712027, "lon":-74.006091},
      {"lat":40.711989, "lon":-74.006251},
      {"lat":40.711802, "lon":-74.006961},
      {"lat":40.711779, "lon":-74.007052},
      {"lat":40.711707, "lon":-74.006961},
      {"lat":40.711726, "lon":-74.006884},
      {"lat":40.711901, "lon":-74.006213},
      {"lat":40.711939, "lon":-74.006068},
      {"lat":40.711916, "lon":-74.005877},
      {"lat":40.711974, "lon":-74.005595},
      {"lat":40.711989, "lon":-74.005419},
      {"lat":40.712000, "lon":-74.005305},
      {"lat":40.711985, "lon":-74.005320},
      {"lat":40.712069, "lon":-74.005354},
      {"lat":40.712153, "lon":-74.005392},
      {"lat":40.712214, "lon":-74.005549},
      {"lat":40.712458, "lon":-74.006342},
      {"lat":40.713633, "lon":-74.006518},
      {"lat":40.713923, "lon":-74.007250},
      {"lat":40.714800, "lon":-74.007830},
      {"lat":40.715609, "lon":-74.007677},
      {"lat":40.715716, "lon":-74.007651},
      {"lat":40.715929, "lon":-74.007651},
      {"lat":40.716044, "lon":-74.007677},
      {"lat":40.716204, "lon":-74.007712},
      {"lat":40.716364, "lon":-74.007849}],29);
}

void TestGetElevationFromEncodedPolyline() {
  TryGetElevationFromEncodedPolyline("gysalAlg|zpC~Clt@tDtx@hHfaBdKl{BrKbnApGro@tJrz@jBbQj@zVt@lTjFnnCrBz}BmFnoB]pHwCvm@eJxtATvXTnfAk@|^z@rGxGre@nTpnBhBbQvXduCrUr`Edd@naEja@~gAhk@nzBxf@byAfm@tuCvDtOvNzi@|jCvkKngAl`HlI|}@`N`{Adx@pjE??xB|J", 41);
}

int main(void) {
  test::suite suite("elevation");

  suite.test(TEST_CASE(TestGetElevationFromLatLng));
  suite.test(TEST_CASE(TestGetElevationFromEncodedPolyline));

  return suite.tear_down();
}
}
