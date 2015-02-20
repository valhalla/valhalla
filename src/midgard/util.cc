#include "valhalla/midgard/util.h"
#include "valhalla/midgard/constants.h"
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/point2.h>

#include <cstdint>
#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <algorithm>

namespace {
constexpr double POLYLINE_PRECISION = 1E6;
constexpr double INV_POLYLINE_PRECISION = 1.0 / POLYLINE_PRECISION;
}

namespace valhalla {
namespace midgard {

int GetTime(const float length, const float speed) {
  return (int)(length / (speed * kHourPerSec) + 0.5f);
}

uint32_t GetTurnDegree(const uint32_t from_heading, const uint32_t to_heading) {
  return (((to_heading - from_heading) + 360) % 360);
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
  int i = *(int*)&x;                 // get bits for floating value
  i = 0x5f3759df - (i >> 1);         // give initial guess y0
  x = *(float*)&i;                   // convert bits back to float
  return x * (1.5f - xhalf * x * x); // newton step
  // x *= 1.5f - xhalf*x*x;          // repeating step increases accuracy
}

template<class container_t>
std::string encode(const container_t& points) {
  //a place to keep the output
  std::string output;
  //unless the shape is very course you should probably only need about 3 bytes
  //per coord, which is 6 bytes with 2 coords, so we overshoot to 8 just in case
  output.reserve(points.size() * 8);

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
  for (const auto& p : points) {
    //shift the decimal point 5 places to the right and truncate
    int lon = static_cast<int>(floor(static_cast<double>(p.first) * POLYLINE_PRECISION));
    int lat = static_cast<int>(floor(static_cast<double>(p.second) * POLYLINE_PRECISION));
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
  //as above we'll say each point uses 6 bytes, so we overshoot to a quarter of the size
  output.reserve(encoded.size() * .25f);

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
    }while (byte >= 0x20);
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
    output.emplace_back(static_cast<typename container_t::value_type::first_type>(static_cast<double>(lon) * INV_POLYLINE_PRECISION),
                        static_cast<typename container_t::value_type::second_type>(static_cast<double>(lat) * INV_POLYLINE_PRECISION));
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
template std::string encode<std::vector<std::pair<float, float> > >(
    const std::vector<std::pair<float, float> >&);
template std::string encode<std::vector<std::pair<double, double> > >(
    const std::vector<std::pair<double, double> >&);
template std::vector<PointLL> decode<std::vector<PointLL> >(const std::string&);
template std::vector<Point2> decode<std::vector<Point2> >(const std::string&);
template std::vector<std::pair<float, float> > decode<
    std::vector<std::pair<float, float> > >(const std::string&);
template std::vector<std::pair<double, double> > decode<
    std::vector<std::pair<double, double> > >(const std::string&);

memory_status::memory_status(const std::unordered_set<std::string> interest){
  //grab the vm stats from the file
  std::ifstream file("/proc/self/status");
  std::string line;
  while(std::getline(file, line)){
    //did we find a memory metric
    if(line.find_first_of("Vm") == 0){
      //grab the name of it and see if we care about it
      std::string name = line.substr(0, line.find_first_of(':'));
      if(interest.size() > 0 && interest.find(name) == interest.end())
        continue;
      //try to get the number of bytes
      std::remove_if(line.begin(), line.end(), [](const char c) {return !std::isdigit(c);});
      if(line.size() == 0)
        continue;
      auto bytes = std::stod(line) * 1024.0;
      //get the units and scale
      std::pair<double, std::string> metric = std::make_pair(bytes, "b");
      for(auto unit : { "B", "KB", "MB", "GB" }){
        metric.second = unit;
        if (metric.first > 1024.0)
          metric.first /= 1024.0;
        else
          break;
      }
      metrics.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(metric));
    }
    line.clear();
  }
}

std::ostream& operator<<(std::ostream& stream, const memory_status& s){
  for(const auto& metric : s.metrics)
    stream << metric.first << ": " << metric.second.first << metric.second.second << std::endl;
  return stream;
}

}
}
