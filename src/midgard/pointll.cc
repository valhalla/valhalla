#include "valhalla/midgard/pointll.h"
#include <valhalla/midgard/util.h>
#include "valhalla/midgard/distanceapproximator.h"
#include "valhalla/midgard/vector2.h"

#include <limits>

namespace {
const float INVALID = 0xBADBADBAD;
}

namespace valhalla {
namespace midgard {

PointLL::PointLL()
    : Point2(INVALID, INVALID) {
}

float PointLL::lat() const {
  return second;
}

float PointLL::lng() const {
  return first;
}

bool PointLL::IsValid() const {
  //is a range check appropriate?
  //return first >= -180 && y >= -90 && first < 180 && second < 90;
  return first != INVALID && second != INVALID;
}

void PointLL::Invalidate() {
  first = INVALID;
  second = INVALID;
}

// May want to use DistanceApproximator!
float PointLL::DistanceSquared(const PointLL& ll2) const {
  return sqr(Distance(ll2));
}

float PointLL::Distance(const PointLL& ll2) const {
  // If points are the same, return 0
  if (*this == ll2)
    return 0.0f;

  // Delta longitude. Don't need to worry about crossing 180
  // since cos(x) = cos(-x)
  double deltalng = (ll2.lng() - lng()) * kRadPerDeg;
  double a = lat() * kRadPerDeg;
  double c = ll2.lat() * kRadPerDeg;

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

float PointLL::Length(const std::vector<PointLL>& pts) {
  float length = 0.f;
  for (unsigned int i = 0; i < pts.size() - 1; ++i)
    length += pts[i].Distance(pts[i + 1]);
  return length;
}

float PointLL::Curvature(const PointLL& ll1, const PointLL& ll2) const {
  // Find the 3 distances between positions
  float a = Distance(ll1);
  float b = ll1.Distance(ll2);
  float c = Distance(ll2);
  float s = (a + b + c) * 0.5f;
  float k = sqrtf(s * (s - a) * (s - b) * (s - c));
  return ((a * b * c) / (4.0f * k));
}

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

std::tuple<PointLL, float, int> PointLL::ClosestPoint(const std::vector<PointLL>& pts) const {
  PointLL closest;
  int idx;
  float mindist = std::numeric_limits<float>::max();

  // If there are no points we are done
  if(pts.size() == 0)
    return std::make_tuple(std::move(closest), std::move(mindist), std::move(idx));
  // If there is one point we are done
  if(pts.size() == 1)
    return std::make_tuple(pts.front(), DistanceSquared(pts.front()), 0);

  DistanceApproximator approx(*this);

  // Iterate through the pts
  bool beyond_end = true;   // Need to test past the end point?
  Vector2 v1;       // Segment vector (v1)
  Vector2 v2;       // Vector from origin to target (v2)
  PointLL projpt;   // Projected point along v1
  float dot;        // Dot product of v1 and v2
  float comp;       // Component of v2 along v1
  float dist;       // Squared distance from target to closest point on line
  for (size_t index = 0; index < pts.size() - 1; ++index) {
    // Get the current segment
    const PointLL& p0 = pts[index];
    const PointLL& p1 = pts[index + 1];

    // Construct vector v1 - represents the segment.  Skip 0 length segments
    v1.Set(p0, p1);
    if (v1.x() == 0.0f && v1.y() == 0.0f)
      continue;

    // Vector v2 from the segment origin to the target point
    v2.Set(p0, *this);

    // Find the dot product of v1 and v2.  If less than 0 the segment
    // origin is the closest point.  Find the distance and continue
    // to the next segment.
    dot = v1.Dot(v2);
    if (dot <= 0.0f) {
      beyond_end = false;
      dist = approx.DistanceSquared(p0);
      if (dist < mindist) {
        mindist = dist;
        closest = p0;
        idx = index;
      }
      continue;
    }

    // Closest point is either beyond the end of the segment or at a point
    // along the segment. Find the component of v2 along v1
    comp = dot / v1.Dot(v1);

    // If component >= 1.0 the segment end is the closest point. A future
    // polyline segment will be closer.  If last segment we need to check
    // distance to the endpoint.  Set flag so this happens.
    if (comp >= 1.0f)
      beyond_end = true;
    else {
      // Closest point is along the segment.  The closest point is found
      // by adding the projection of v2 onto v1 to the origin point.
      // The squared distance from this point to the target is then found.
      // TODO - why not:  *p0 + v1 * comp;
      beyond_end = false;
      projpt = p0 + v1 * comp;
      dist = approx.DistanceSquared(projpt);
      if (dist < mindist) {
        mindist = dist;
        closest = projpt;
        idx = index;
      }
    }
  }

  // Test the end point if flag is set - it may be the closest point
  if (beyond_end) {
    dist = approx.DistanceSquared(pts[pts.size() - 1]);
    if (dist < mindist) {
      mindist = dist;
      closest = pts.back();
      idx = (int) (pts.size() - 2);
    }
  }
  return std::make_tuple(std::move(closest), std::move(mindist), std::move(idx));
}

// Calculate the heading from the start of a polyline of lat,lng points to a
// point at the specified distance from the start.
float PointLL::HeadingAlongPolyline(const std::vector<PointLL>& pts,
                                    const float dist) {
  int n = (int) pts.size();
  if (n < 2) {
    // TODO error!?
    return 0.0f;
  }
  if (n == 2) {
    return pts[0].Heading(pts[1]);
  }

  int i = 0;
  double d = 0.0;
  double seglength = 0.0;
  while (d < dist && i < n - 1) {
    seglength = pts[i].Distance(pts[i + 1]);
    if (d + seglength > dist) {
      // Set the extrapolated point along the line.
      float pct = (float) ((dist - d) / seglength);
      PointLL ll(pts[i].lng() + ((pts[i + 1].lng() - pts[i].lng()) * pct),
                 pts[i].lat() + ((pts[i + 1].lat() - pts[i].lat()) * pct));
      return pts[0].Heading(ll);
    } else {
      d += seglength;
      i++;
    }
  }

  // Length of polyline is less than the specified distance.
  // Return heading from first to last point.
  return pts[0].Heading(pts[n-1]);
}

// Calculate the heading from a point at a specified distance from the end
// of a polyline of lat,lng points to the end point of the polyline.
float PointLL::HeadingAtEndOfPolyline(const std::vector<PointLL>& pts,
                                      const float dist) {
  int n = (int) pts.size();
  if (n < 2) {
    // TODO error!?
    return 0.0f;
  }
  if (n == 2) {
    return pts[0].Heading(pts[1]);
  }

  float heading = 0.0f;
  int i = n - 2;
  double d = 0.0;
  double seglength;
  while (d < dist && i >= 0) {
    seglength = pts[i].Distance(pts[i + 1]);
    if (d + seglength > dist) {
      // Set the extrapolated point along the line.
      float pct = (float) ((dist - d) / seglength);
      PointLL ll(pts[i + 1].lng() + ((pts[i].lng() - pts[i + 1].lng()) * pct),
                 pts[i + 1].lat() + ((pts[i].lat() - pts[i + 1].lat()) * pct));
      return ll.Heading(pts[n - 1]);
    } else {
      d += seglength;
      i--;
    }
  }

  // Length of polyline is less than the specified distance.
  // Return heading from first to last point.
  return pts[0].Heading(pts[n-1]);
}

// Test whether this point is to the left of a segment from p1 to p2. Uses a
// 2-D cross product and tests the sign (> 0 indicates the point is to the
// left).
bool PointLL::IsLeft(const PointLL& p1, const PointLL& p2) const {
  return ((p2.x() - p1.x()) * (   y() - p1.y()) -
             (x() - p1.x()) * (p2.y() - p1.y()) >= 0.0f);
}

// Tests whether this point is within a convex polygon. Iterate through the
// edges - to be inside the point must be to the same side of each edge.
bool PointLL::WithinConvexPolygon(const std::vector<PointLL>& poly) const {
   // Get the side relative to the last edge
  bool left = IsLeft(poly.back(), poly.front());

  // Iterate through edges
  auto p1 = poly.begin();
  auto p2 = p1 + 1;
  for ( ; p2 < poly.end(); p1++, p2++) {
    if (IsLeft(*p1, *p2) != left) {
      return false;
    }
  }
  return true;
}

}
}
