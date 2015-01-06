#include "valhalla/midgard/util.h"
#include "valhalla/midgard/constants.h"
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/point2.h>


#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <utility>

namespace valhalla{
namespace midgard{

int GetTime(const float length, const float speed) {
  return (int)(length / (speed * kHourPerSec) + 0.5f);
}

// TODO - do we really need these?
/*float degrees_to_radians(const float d) {
  return d * kDegPerRad;
}
float radians_to_degrees(const float r) {
  return r * kRadPerDeg;
}*/

float rand01() {
  return (float)rand() / (float)RAND_MAX;
}

float FastInvSqrt(float x) {
   float xhalf = 0.5f * x;
   int i = *(int*)&x;            // get bits for floating value
   i = 0x5f3759df - (i>>1);      // give initial guess y0
   x = *(float*)&i;              // convert bits back to float
   return x*(1.5f - xhalf*x*x);  // newton step
   // x *= 1.5f - xhalf*x*x;     // repeating step increases accuracy
}

float sqr(const float a) {
  return a * a;
}

template<class container_t>
std::string encode(const container_t& points) {
  //a place to keep the output
  std::string output;
  //unless the shape is very course you should probably only need about
  //2 or 3 bytes per coordinate, and there are 2 coordinates per point
  output.reserve(points.size() * 6);

  //handy lambda to turn an integer into an encoded string
  auto serialize = [&output](int number) {
    //move the bits left 1 position and flip all the bits if it was a negative number
    number = number < 0 ? ~(number << 1) : (number << 1);
    //write 5 bit chunks of the number
    while (number >= 0x20) {
      int nextValue = (0x20 | (number & 0x1f)) + 63;
      output.push_back(static_cast<char>(nextValue));
      number >>= 5;
    }
    //write the last chunk
    number += 63;
    output.push_back(static_cast<char>(number));
  };

  //this is an offset encoding so we remember the last point we saw
  int last_lon = 0, last_lat = 0;
  //for each point
  for(const auto& p : points) {
    //shift the decimal point 5 places to the right and truncate
    int lon = static_cast<int>(floor(p.first * 1e5));
    int lat = static_cast<int>(floor(p.second * 1e5));
    //encode each coordinate, lat first for some reason
    serialize(lat - last_lat);
    serialize(lon - last_lon);
    //remember the last one we encountered
    last_lon = lon;
    last_lat = lat;
  }
  return output;
}

template<class container_t>
container_t decode(const std::string& encoded) {
  //a place to keep the output
  container_t output;
  //based on the length of the string we can make a guess at how many points are in it
  //as above we'll say each point uses 6 bytes, so we'll guess its a sixth the size
  output.reserve(encoded.size() * .16f);

  //what byte are we looking at
  size_t i = 0;

  //handy lambda to turn a few bytes of an encoded string into an integer
  auto deserialize = [&encoded, &i](const int previous) {
    //grab each 5 bits and mask it in where it belongs using the shift
    int byte, shift = 0, result = 0;
    do {
      //TODO: could use a check here for out of bounds
      //which could happen on improper polyline string data
      byte = static_cast<int>(encoded[i++]) - 63;
      result |= (byte & 0x1f) << shift;
      shift += 5;
    } while (byte >= 0x20);
    //undo the left shift from above or the bit flipping and add to previous since its an offset
    return previous + (result & 1 ? ~(result >> 1) : (result >> 1));
  };

  //make sure to go over all the characters
  int last_lon = 0, last_lat = 0;
  while (i < encoded.length()) {
    //decode the coordinates, lat first for some reason
    int lat = deserialize(last_lat);
    int lon = deserialize(last_lon);
    //shift the decimal point 5 places to the left
    output.emplace_back(static_cast<float>(lon) / 1e5, static_cast<float>(lat) / 1e5);
    //remember the last one we encountered
    last_lon = lon;
    last_lat = lat;
  }
  return output;
}

//explicit instantiations, we should probably just move the implementation to the header so that
//projects that depend on this library aren't limited in the instantiations made here
template std::string encode<std::vector<PointLL> >(const std::vector<PointLL>&);
template std::string encode<std::vector<Point2> >(const std::vector<Point2>&);
template std::string encode<std::vector<std::pair<float, float> > >(const std::vector<std::pair<float, float> >&);
template std::string encode<std::vector<std::pair<double, double> > >(const std::vector<std::pair<double, double> >&);
template std::vector<PointLL> decode<std::vector<PointLL> >(const std::string&);
template std::vector<Point2> decode<std::vector<Point2> >(const std::string&);
template std::vector<std::pair<float, float> > decode<std::vector<std::pair<float, float> > >(const std::string&);
template std::vector<std::pair<double, double> > decode<std::vector<std::pair<double, double> > >(const std::string&);

}
}
