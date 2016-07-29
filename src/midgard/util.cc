#include "valhalla/midgard/util.h"
#include "valhalla/midgard/constants.h"
#include "valhalla/midgard/point2.h"
#include "valhalla/midgard/distanceapproximator.h"

#include <cstdint>
#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <list>
#include <sys/stat.h>

namespace {

constexpr double POLYLINE_PRECISION = 1E6;
constexpr double INV_POLYLINE_PRECISION = 1.0 / POLYLINE_PRECISION;
constexpr double RAD_PER_METER  = 1.0 / 6378160.187;
constexpr double RAD_PER_DEG = M_PI / 180.0;
constexpr double DEG_PER_RAD = 180.0 / M_PI;

}

namespace valhalla {
namespace midgard {

int GetTime(const float length, const float speed) {
  if (speed > 0.0f)
    return (int)(length / (speed * kHourPerSec) + 0.5f);
  return 0;
}

uint32_t GetTurnDegree(const uint32_t from_heading, const uint32_t to_heading) {
  return (((to_heading - from_heading) + 360) % 360);
}

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
void decode(const std::string& encoded, container_t& output) {
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
}

template<class container_t>
std::string encode7(const container_t& points) {
  //a place to keep the output
  std::string output;
  //unless the shape is very course you should probably only need about 3 bytes
  //per coord, which is 6 bytes with 2 coords, so we overshoot to 8 just in case
  output.reserve(points.size() * 8);

  //handy lambda to turn an integer into an encoded string
  auto serialize = [&output](int number) {
    //get the sign bit down on the least significant end to
    //make the most significant bits mostly zeros
    number = number < 0 ? ~(number << 1) : number << 1;
    //we take 7 bits of this at a time
    while (number > 0x7f) {
      //marking the most significant bit means there are more pieces to come
      int nextValue = (0x80 | (number & 0x7f));
      output.push_back(static_cast<char>(nextValue));
      number >>= 7;
    }
    //write the last chunk
    output.push_back(static_cast<char>(number & 0x7f));
  };

  //this is an offset encoding so we remember the last point we saw
  int last_lon = 0, last_lat = 0;
  //for each point
  for (const auto& p : points) {
    //shift the decimal point x places to the right and truncate
    int lon = static_cast<int>(floor(static_cast<double>(p.first) * 1e6));
    int lat = static_cast<int>(floor(static_cast<double>(p.second) * 1e6));
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
void decode7(const std::string& encoded, container_t& output) {
  //what byte are we looking at
  size_t i = 0;

  //handy lambda to turn a few bytes of an encoded string into an integer
  auto deserialize = [&encoded, &i](const int previous) {
    //TODO: a bogus polyline could cause reading from out of bounds
    int byte, shift = 0, result = 0;
    do {
      //take the least significant 7 bits shifted into place
      byte = static_cast<int>(encoded[i++]);
      result |= (byte & 0x7f) << shift;
      shift += 7;
      //if the most significant bit is set there is more to this number
    }while (byte & 0x80);
    //undo the left shift from above or the bit flipping and add to previous since its an offset
    return previous + ((result & 1 ? ~result : result) >> 1);
  };

  //make sure to go over all the characters
  int last_lon = 0, last_lat = 0;
  while (i < encoded.length()) {
    //decode the coordinates, lat first for some reason
    int lat = deserialize(last_lat);
    int lon = deserialize(last_lon);
    //shift the decimal point 5 places to the left
    output.emplace_back(static_cast<typename container_t::value_type::first_type>(static_cast<double>(lon) * 1e-6),
                        static_cast<typename container_t::value_type::second_type>(static_cast<double>(lat) * 1e-6));
    //remember the last one we encountered
    last_lon = lon;
    last_lat = lat;
  }
}

//specialize for list
template <>
std::list<PointLL> decode<std::list<PointLL> >(const std::string& encoded)
{ std::list<PointLL> l; decode(encoded, l); return l; }

template <>
std::list<std::pair<double,double> > decode<std::list<std::pair<double,double> > >(const std::string& encoded)
{ std::list<std::pair<double,double> > l; decode(encoded, l); return l; }

template <>
std::list<Point2> decode<std::list<Point2> >(const std::string& encoded)
{ std::list<Point2> l; decode(encoded, l); return l; }

template <>
std::list<std::pair<float,float> > decode<std::list<std::pair<float,float> > >(const std::string& encoded)
{ std::list<std::pair<float,float> > l; decode(encoded, l); return l; }

template <>
std::list<PointLL> decode7<std::list<PointLL> >(const std::string& encoded)
{ std::list<PointLL> l; decode7(encoded, l); return l; }

template <>
std::list<std::pair<double,double> > decode7<std::list<std::pair<double,double> > >(const std::string& encoded)
{ std::list<std::pair<double,double> > l; decode7(encoded, l); return l; }

template <>
std::list<Point2> decode7<std::list<Point2> >(const std::string& encoded)
{ std::list<Point2> l; decode7(encoded, l); return l; }

template <>
std::list<std::pair<float,float> > decode7<std::list<std::pair<float,float> > >(const std::string& encoded)
{ std::list<std::pair<float,float> > l; decode7(encoded, l); return l; }

//specialize for vector with a preallocation
//based on the length of the string we can make a guess at how many points are in it
//as above we'll say each point uses 6 bytes, so we overshoot to a quarter of the size
template <>
std::vector<PointLL> decode<std::vector<PointLL> >(const std::string& encoded)
{ std::vector<PointLL> v; v.reserve(encoded.size() * .25f); decode(encoded, v); return v; }

template <>
std::vector<std::pair<double,double> > decode<std::vector<std::pair<double,double> > >(const std::string& encoded)
{ std::vector<std::pair<double,double> > v; v.reserve(encoded.size() * .25f); decode(encoded, v); return v; }

template <>
std::vector<Point2> decode<std::vector<Point2> >(const std::string& encoded)
{ std::vector<Point2> v; v.reserve(encoded.size() * .25f); decode(encoded, v); return v; }

template <>
std::vector<std::pair<float,float> > decode<std::vector<std::pair<float,float> > >(const std::string& encoded)
{ std::vector<std::pair<float,float> > v; v.reserve(encoded.size() * .25f); decode(encoded, v); return v; }

template <>
std::vector<PointLL> decode7<std::vector<PointLL> >(const std::string& encoded)
{ std::vector<PointLL> v; v.reserve(encoded.size() * .25f); decode7(encoded, v); return v; }

template <>
std::vector<std::pair<double,double> > decode7<std::vector<std::pair<double,double> > >(const std::string& encoded)
{ std::vector<std::pair<double,double> > v; v.reserve(encoded.size() * .25f); decode7(encoded, v); return v; }

template <>
std::vector<Point2> decode7<std::vector<Point2> >(const std::string& encoded)
{ std::vector<Point2> v; v.reserve(encoded.size() * .25f); decode7(encoded, v); return v; }

template <>
std::vector<std::pair<float,float> > decode7<std::vector<std::pair<float,float> > >(const std::string& encoded)
{ std::vector<std::pair<float,float> > v; v.reserve(encoded.size() * .25f); decode7(encoded, v); return v; }

//explicit instantiations, we should probably just move the implementation to the header so that
//projects that depend on this library aren't limited in the instantiations made here
template std::string encode<std::vector<PointLL> >(const std::vector<PointLL>&);
template std::string encode<std::vector<std::pair<double,double> > >(const std::vector<std::pair<double,double> >&);
template std::string encode<std::vector<Point2> >(const std::vector<Point2>&);
template std::string encode<std::vector<std::pair<float,float> > >(const std::vector<std::pair<float,float> >&);

template std::string encode<std::list<PointLL> >(const std::list<PointLL>&);
template std::string encode<std::list<std::pair<double,double> > >(const std::list<std::pair<double,double> >&);
template std::string encode<std::list<Point2> >(const std::list<Point2>&);
template std::string encode<std::list<std::pair<float,float> > >(const std::list<std::pair<float,float> >&);

template std::string encode7<std::vector<PointLL> >(const std::vector<PointLL>&);
template std::string encode7<std::vector<std::pair<double,double> > >(const std::vector<std::pair<double,double> >&);
template std::string encode7<std::vector<Point2> >(const std::vector<Point2>&);
template std::string encode7<std::vector<std::pair<float,float> > >(const std::vector<std::pair<float,float> >&);

template std::string encode7<std::list<PointLL> >(const std::list<PointLL>&);
template std::string encode7<std::list<std::pair<double,double> > >(const std::list<std::pair<double,double> >&);
template std::string encode7<std::list<Point2> >(const std::list<Point2>&);
template std::string encode7<std::list<std::pair<float,float> > >(const std::list<std::pair<float,float> >&);

template std::vector<PointLL> decode<std::vector<PointLL> >(const std::string&);
template std::vector<std::pair<double,double> > decode<std::vector<std::pair<double,double> > >(const std::string&);
template std::vector<Point2> decode<std::vector<Point2> >(const std::string&);
template std::vector<std::pair<float,float> > decode<std::vector<std::pair<float,float> > >(const std::string&);

template std::list<PointLL> decode<std::list<PointLL> >(const std::string&);
template std::list<std::pair<double,double> > decode<std::list<std::pair<double,double> > >(const std::string&);
template std::list<Point2> decode<std::list<Point2> >(const std::string&);
template std::list<std::pair<float,float> > decode<std::list<std::pair<float,float> > >(const std::string&);

template std::vector<PointLL> decode7<std::vector<PointLL> >(const std::string&);
template std::vector<std::pair<double,double> > decode7<std::vector<std::pair<double,double> > >(const std::string&);
template std::vector<Point2> decode7<std::vector<Point2> >(const std::string&);
template std::vector<std::pair<float,float> > decode7<std::vector<std::pair<float,float> > >(const std::string&);

template std::list<PointLL> decode7<std::list<PointLL> >(const std::string&);
template std::list<std::pair<double,double> > decode7<std::list<std::pair<double,double> > >(const std::string&);
template std::list<Point2> decode7<std::list<Point2> >(const std::string&);
template std::list<std::pair<float,float> > decode7<std::list<std::pair<float,float> > >(const std::string&);

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

bool memory_status::supported() {
  struct stat s;
  return stat("/proc/self/status", &s) == 0;
}

std::ostream& operator<<(std::ostream& stream, const memory_status& s){
  for(const auto& metric : s.metrics)
    stream << metric.first << ": " << metric.second.first << metric.second.second << std::endl;
  return stream;
}

/* This method makes use of several computations explained and demonstrated at:
 * http://williams.best.vwh.net/avform.htm *
 * We humbly bow to you sir!
 */
template <class container_t>
container_t resample_spherical_polyline(const container_t& polyline, double resolution, bool preserve) {
  if(polyline.size() == 0)
    return {};

  //for each point
  container_t resampled = {polyline.front()};
  resolution *= RAD_PER_METER;
  double remaining = resolution;
  PointLL last = resampled.back();
  for(auto p = std::next(polyline.cbegin()); p != polyline.cend(); ++p) {
    //radians
    auto lon2 = p->first * -RAD_PER_DEG;
    auto lat2 = p->second * RAD_PER_DEG;
    //how much do we have left on this segment from where we are (in great arc radians)
    //double d = 2.0 * asin(sqrt(pow(sin((resampled.back().second * RAD_PER_DEG - lat2) / 2.0), 2.0) + cos(resampled.back().second * RAD_PER_DEG) * cos(lat2) *pow(sin((resampled.back().first * -RAD_PER_DEG - lon2) / 2.0), 2.0)));
    auto d = acos(
      sin(last.second * RAD_PER_DEG) * sin(lat2) +
      cos(last.second * RAD_PER_DEG) * cos(lat2) *
      cos(last.first * -RAD_PER_DEG - lon2)
    );
    //keep placing points while we can fit them
    while(d > remaining) {
      //some precomputed stuff
      auto lon1 = last.first * -RAD_PER_DEG;
      auto lat1 = last.second * RAD_PER_DEG;
      auto sd = sin(d);
      auto a = sin(d - remaining) / sd;
      auto acs1 = a * cos(lat1);
      auto b = sin(remaining) / sd;
      auto bcs2 = b * cos(lat2);
      //find the interpolated point along the arc
      auto x = acs1 * cos(lon1) + bcs2 * cos(lon2);
      auto y = acs1 * sin(lon1) + bcs2 * sin(lon2);
      auto z = a * sin(lat1) + b * sin(lat2);
      last.first = atan2(y, x) * -DEG_PER_RAD;
      last.second = atan2(z, sqrt(x * x + y * y)) * DEG_PER_RAD;
      resampled.push_back(last);
      //we just consumed a bit
      d -= remaining;
      //we need another bit
      remaining = resolution;
    }
    //we're going to the next point so consume whatever's left
    remaining -= d;
    last = *p;
    if(preserve)
      resampled.push_back(last);
  }

  //TODO: do we want to let them know remaining?

  //hand it back
  return resampled;
}

//explicit instantiations
template std::vector<PointLL> resample_spherical_polyline<std::vector<PointLL> >(const std::vector<PointLL>&, double, bool);
template std::vector<Point2> resample_spherical_polyline<std::vector<Point2> >(const std::vector<Point2>&, double, bool);
template std::list<PointLL> resample_spherical_polyline<std::list<PointLL> >(const std::list<PointLL>&, double, bool);
template std::list<Point2> resample_spherical_polyline<std::list<Point2> >(const std::list<Point2>&, double, bool);

//Return the intersection of two infinite lines if any
template <class coord_t>
bool intersect(const coord_t& u, const coord_t& v, const coord_t& a, const coord_t& b, coord_t& i) {
  auto uv_xd = u.first - v.first;
  auto uv_yd = u.second - v.second;
  auto ab_xd = a.first - b.first;
  auto ab_yd = a.second - b.second;
  auto d_cross = uv_xd*ab_yd - ab_xd*uv_yd;
  //parallel or very close to it
  if(std::abs(d_cross) < 1e-5)
    return false;
  auto uv_cross = u.first*v.second - u.second*v.first;
  auto ab_cross = a.first*b.second - a.second*b.first;
  i.first  = (uv_cross*ab_xd - uv_xd*ab_cross) / d_cross;
  i.second = (uv_cross*ab_yd - uv_yd*ab_cross) / d_cross;
  return true;
}
template bool intersect<PointLL>(const PointLL& u, const PointLL& v, const PointLL& a, const PointLL& b, PointLL& i);
template bool intersect<Point2>(const Point2& u, const Point2& v, const Point2& a, const Point2& b, Point2& i);

//Return the intercept of the line passing through uv with the horizontal line defined by y
template <class coord_t>
typename coord_t::first_type y_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type y) {
  if(std::abs(u.first - v.first) < 1e-5)
    return u.first;
  if(std::abs(u.second - u.second) < 1e-5)
    return NAN;
  auto m = (v.second - u.second) / (v.first - u.first);
  auto b = u.second - (u.first * m);
  return (y - b) / m;
}
template PointLL::first_type y_intercept<PointLL>(const PointLL& u, const PointLL& v, const PointLL::first_type y);
template Point2::first_type y_intercept<Point2>(const Point2& u, const Point2& v, const Point2::first_type y);

//Return the intercept of the line passing through uv with the vertical line defined by x
template <class coord_t>
typename coord_t::first_type x_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type x) {
  if(std::abs(u.second - v.second) < 1e-5)
    return u.second;
  if(std::abs(u.first - v.first) < 1e-5)
    return NAN;
  auto m = (v.second - u.second) / (v.first - u.first);
  auto b = u.second - (u.first * m);
  return x * m + b;
}
template PointLL::second_type x_intercept<PointLL>(const PointLL& u, const PointLL& v, const PointLL::second_type x);
template Point2::second_type x_intercept<Point2>(const Point2& u, const Point2& v, const Point2::second_type x);

template <class container_t>
float polygon_area(const container_t& polygon) {
  typename container_t::value_type::first_type area = polygon.back() == polygon.front() ? 0.f :
    (polygon.back().first + polygon.front().first)*(polygon.back().second + polygon.front().second);
  for(auto p1 = polygon.cbegin(), p2 = std::next(polygon.cbegin()); p2 != polygon.cend(); ++p1, ++p2)
    area += (p1->first + p2->first)*(p1->second + p2->second);
  return area*.5;
}

template PointLL::first_type polygon_area(const std::list<PointLL>&);
template PointLL::first_type polygon_area(const std::vector<PointLL>&);
template Point2::first_type polygon_area(const std::list<Point2>&);
template Point2::first_type polygon_area(const std::vector<Point2>&);

}
}
