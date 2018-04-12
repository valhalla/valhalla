#include "midgard/pointll.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include "midgard/distanceapproximator.h"
#include "midgard/vector2.h"
#include "midgard/logging.h"

#include <limits>
#include <cmath>
#include <list>

namespace {
constexpr double RAD_PER_DEG = valhalla::midgard::kPiDouble / 180.0;
constexpr double DEG_PER_RAD = 180.0 / valhalla::midgard::kPiDouble;
}

namespace valhalla {
namespace midgard {

constexpr float PointLL::INVALID;

// Checks for validity of the coordinates.
bool PointLL::IsValid() const {
  // TODO: is a range check appropriate?
  return first != INVALID && second != INVALID;
}

// Sets the coordinates to an invalid state
void PointLL::Invalidate() {
  first  = INVALID;
  second = INVALID;
}

// Approximates the distance squared between two lat,lng points.
float PointLL::DistanceSquared(const PointLL& ll2) const {
  DistanceApproximator approx(*this);
  return approx.DistanceSquared(ll2);
}

// Mid point along the geodesic between these points
PointLL PointLL::MidPoint(const PointLL& p) const {
  //radians
  const auto lon1 = first * -RAD_PER_DEG;
  const auto lat1 = second * RAD_PER_DEG;
  const auto lon2 = p.first * -RAD_PER_DEG;
  const auto lat2 = p.second * RAD_PER_DEG;
  //useful throughout
  const auto sl1 = sin(lat1);
  const auto sl2 = sin(lat2);
  const auto cl1 = cos(lat1);
  const auto cl2 = cos(lat2);
  //fairly accurate distance between points
  const auto d = acos(
    sl1 * sl2 +
    cl1 * cl2 *
    cos(lon1 - lon2)
  );
  //interpolation parameters
  const auto ab = sin(d * .5) / sin(d);
  const auto acs1 = ab * cl1;
  const auto bcs2 = ab * cl2;
  //find the interpolated point along the arc
  const auto x = acs1 * cos(lon1) + bcs2 * cos(lon2);
  const auto y = acs1 * sin(lon1) + bcs2 * sin(lon2);
  const auto z = ab * (sl1 + sl2);
  return PointLL(atan2(y, x) * -DEG_PER_RAD, atan2(z, sqrt(x * x + y * y)) * DEG_PER_RAD);
}

// Calculates the distance between two lat/lng's in meters. Uses spherical
// geometry (law of cosines).
float PointLL::Distance(const PointLL& ll2) const {
  // If points are the same, return 0
  if (*this == ll2)
    return 0.0f;

  // Delta longitude. Don't need to worry about crossing 180
  // since cos(x) = cos(-x)
  double deltalng = (ll2.lng() - lng()) * RAD_PER_DEG;
  double a = lat() * RAD_PER_DEG;
  double c = ll2.lat() * RAD_PER_DEG;

  // Find the angle subtended in radians (law of cosines)
  double cosb = (sin(a) * sin(c)) + (cos(a) * cos(c) * cos(deltalng));

  // Angle subtended * radius of earth (portion of the circumference).
  // Protect against cosb being outside -1 to 1 range.
  if (cosb >= 1.0)
    return 0.00001f;
  else if (cosb < -1.0)
    return kPi * kRadEarthMeters;
  else
    return (float)(acos(cosb) * kRadEarthMeters);
}

// Calculates the curvature using this position and 2 others. Found by
// computing the radius of the circle that circumscribes the 3 positions.
float PointLL::Curvature(const PointLL& ll1, const PointLL& ll2) const {
  // Find the 3 distances between positions
  float a = Distance(ll1);
  float b = ll1.Distance(ll2);
  float c = Distance(ll2);
  float s = (a + b + c) * 0.5f;
  float k = sqrtf(s * (s - a) * (s - b) * (s - c));
  return (std::isnan(k) || k == 0.0f) ? std::numeric_limits<float>::max() :
            ((a * b * c) / (4.0f * k));
}

// Calculates the heading or azimuth from the current lat,lng to the
// specified lat,lng. This uses Haversine method (spherical geometry).
float PointLL::Heading(const PointLL& ll2) const {
  // If points are the same, return 0
  if (*this == ll2)
    return 0.0f;

  // Convert to radians and compute heading
  float lat1 = lat() * kRadPerDeg;
  float lat2 = ll2.lat() * kRadPerDeg;
  float dlng = (ll2.lng() - lng()) * kRadPerDeg;
  float y = sinf(dlng) * cosf(lat2);
  float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlng);
  float bearing = atan2f(y, x) * kDegPerRad;
  return (bearing < 0.0f) ? bearing + 360.0f : bearing;
}

// Finds the closest point to the supplied polyline as well as the distance
// squared to that point and the index of the segment where the closest point
// lies.
std::tuple<PointLL, float, int> PointLL::ClosestPoint(
    const std::vector<PointLL>& pts, size_t begin_index, float dist_cutoff) const {
  PointLL closest {};
  int closest_segment = -1;
  float mindistsqr = std::numeric_limits<float>::max();
  size_t process_size = 0;

  if (begin_index < pts.size())
    process_size = (pts.size() - begin_index);

  // If there are no points to process we are done
  if (process_size == 0)
    return std::make_tuple(std::move(closest), std::move(mindistsqr),
                           std::move(closest_segment));

  // If there is one point to process we are done
  if (process_size == 1)
    return std::make_tuple(pts[begin_index], sqrt(DistanceSquared(pts[begin_index])), begin_index);

  // Longitude (x) is scaled by the cos of the latitude so that distances are
  // correct in lat,lon space
  float lon_scale = cosf(lat() * kRadPerDeg);

  DistanceApproximator approx(*this);
  PointLL point;
  for (size_t index = begin_index; index < pts.size() - 1; ++index) {
    // Get the current segment
    const PointLL& u = pts[index];
    const PointLL& v = pts[index + 1];

    // Project a onto b where b is the origin vector representing this segment
    // and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    auto bx = v.lng() - u.lng();
    auto by = v.lat() - u.lat();

    // Scale longitude when finding the projection. Avoid divided-by-zero
    // which gives a NaN scale, otherwise comparisons below will fail
    auto bx2 = bx * lon_scale;
    auto sq  = bx2 * bx2 + by * by;
    auto scale = sq > 0 ?  (((lng() - u.lng()) * lon_scale * bx2 +
                             (lat() - u.lat()) *by) / sq) : 0.f;

    // Projects along the ray before u
    if (scale <= 0.f) {
      point = { u.lng(), u.lat() };
    } // Projects along the ray after v
    else if (scale >= 1.f) {
      point = { v.lng(), v.lat() };
    } // Projects along the ray between u and v
    else {
      point = { u.lng() + bx * scale, u.lat() + by * scale };
    }

    // Check if this point is better
    const auto sq_distance = approx.DistanceSquared(point);
    if (sq_distance < mindistsqr) {
      closest_segment = static_cast<int>(index);
      mindistsqr = sq_distance;
      closest = std::move(point);
    }

    // Check if we should bail early because of looking at too much shape
    if(dist_cutoff != std::numeric_limits<float>::infinity() &&
        (dist_cutoff -= u.Distance(v)) < 0)
      break;
  }
  return std::make_tuple(std::move(closest), sqrt(mindistsqr),
                         closest_segment);
}

// Calculate the heading from the start index within a polyline of lat,lng
// points to a point at the specified distance from the start.
float PointLL::HeadingAlongPolyline(const std::vector<PointLL>& pts,
                                    const float dist, const uint32_t idx0,
                                    const uint32_t idx1) {
  // Check that at least 2 points exist
  int n = static_cast<int>(idx1) - static_cast<int>(idx0);
  if (n < 1) {
    LOG_ERROR("PointLL::HeadingAlongPolyline has < 2 vertices");
    return 0.0f;
  }

  // If more than 2 points, walk edges of the polyline until the length
  // is exceeded.
  if (n > 1) {
    double d = 0.0;
    double seglength = 0.0;
    auto pt0 = pts.begin() + idx0;
    auto pt1 = pt0 + 1;
    while (d < dist && pt1 <= pts.begin() + idx1) {
      seglength = pt0->Distance(*pt1);
      if (d + seglength > dist) {
        // Set the extrapolated point along the line.
        float pct = static_cast<float>((dist - d) / seglength);
        PointLL ll(pt0->lng() + ((pt1->lng() - pt0->lng()) * pct),
                   pt0->lat() + ((pt1->lat() - pt0->lat()) * pct));
        return pts[idx0].Heading(ll);
      } else {
        d += seglength;
        pt0++;
        pt1++;
      }
    }
  }

  // Only 2 points or the length of polyline is less than the specified
  // distance. Return heading from first to last point.
  return pts[idx0].Heading(pts[idx1]);
}

// Calculate the heading from a point at a specified distance from the end
// of a polyline of lat,lng points to the end point of the polyline.
float PointLL::HeadingAtEndOfPolyline(const std::vector<PointLL>& pts,
                                      const float dist, const uint32_t idx0,
                                      const uint32_t idx1) {
  // Check that at least 2 points exist
  int n = static_cast<int>(idx1) - static_cast<int>(idx0);
  if (n < 1) {
    LOG_ERROR("PointLL::HeadingAtEndOfPolyline has < 2 vertices");
    return 0.0f;
  }

  // If more than 2 points, walk edges of the polyline until the length
  // is exceeded.
  if (n > 1) {
    int i = n - 2;
    double d = 0.0;
    double seglength;
    auto pt1 = pts.begin() + idx1;
    auto pt0 = pt1 - 1;
    while (d < dist && pt0 >= pts.begin() + idx0) {
      seglength = pt0->Distance(*pt1);
      if (d + seglength > dist) {
        // Set the extrapolated point along the line.
        float pct = static_cast<float>((dist - d) / seglength);
        PointLL ll(pt1->lng() + ((pt0->lng() - pt1->lng()) * pct),
                   pt1->lat() + ((pt0->lat() - pt1->lat()) * pct));
        return ll.Heading(pts[idx1]);
      } else {
        d += seglength;
        pt1--;
        pt0--;
      }
    }
  }

  // Only 2 points or the length of polyline is less than the specified
  // distance. Return heading from first to last point.
  return pts[idx0].Heading(pts[idx1]);
}

// Tests whether this point is within a convex polygon. Iterate through the
// edges - to be inside the point must be to the same side of each edge.
template <class container_t>
bool PointLL::WithinPolygon(const container_t& poly) const {
  auto p1 = poly.front() == poly.back() ? poly.begin() : std::prev(poly.end());
  auto p2 = poly.front() == poly.back() ? std::next(p1) : poly.begin();
  //for each edge
  size_t winding_number = 0;
  for(; p2 != poly.end(); p1 = p2, ++p2) {
    //going upward
    if(p1->second <= second) {
      //crosses if its in between on the y and to the left
      winding_number += p2->second > second && IsLeft(*p1, *p2) > 0;
    }//going downward maybe
    else {
      //crosses if its in between or on and to the right
      winding_number -= p2->second <= second && IsLeft(*p1, *p2) < 0;
    }
  }

  // If it was a full ring we are done otherwise check the last segment
  return winding_number != 0;
}

bool PointLL::IsSpherical() { return true; }

PointLL PointLL::Project(const PointLL& u, const PointLL& v, float lon_scale) const {
  //we're done if this is a zero length segment
  if(u == v)
    return u;

  //project a onto b where b is the origin vector representing this segment
  //and a is the origin vector to the point we are projecting, (a.b/b.b)*b
  auto bx = v.first - u.first;
  auto by = v.second - u.second;

  // Scale longitude when finding the projection
  auto bx2 = bx * lon_scale;
  auto sq = bx2 * bx2 + by * by;
  auto scale = (first - u.first) * lon_scale * bx2 + (second - u.second) * by; //only need the numerator at first

  //projects along the ray before u
  if(scale <= 0.f)
    return u;
  //projects along the ray after v
  else if(scale >= sq)
    return v;
  //projects along the ray between u and v
  scale /= sq;
  return {u.first + bx * scale, u.second + by * scale};
}

// Explicit instantiations
template bool PointLL::WithinPolygon(const std::vector<PointLL>&) const;
template bool PointLL::WithinPolygon(const std::list<PointLL>&) const;

}
}
