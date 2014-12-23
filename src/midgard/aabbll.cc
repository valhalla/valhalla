#include "valhalla/midgard/aabbll.h"

namespace valhalla {
namespace midgard {

// Default constructor.
AABBLL::AABBLL()
    : AABB2(-180.0f, -90.0f, 180.0f, 90.0f) {
}

// Constructor with min,max lat,lngs
AABBLL::AABBLL(const float minlat, const float minlng, const float maxlat,
               const float maxlng)
    : AABB2(minlng, minlat, maxlng, maxlat) {
}

// Destructor
AABBLL::~AABBLL() {
}

// Get the minimum latitude
float AABBLL::minlat() const {
  return miny_;
}

// Get the minimum longitude
float AABBLL::minlng() const {
  return minx_;
}

// Get the maximum latitude
float AABBLL::maxlat() const {
  return maxy_;
}

// Get the maximum longitude
float AABBLL::maxlng() const {
  return maxx_;
}

// Get the upper left lat,lng point
PointLL AABBLL::GetUpperLeft() const {
  return PointLL(maxy_, minx_);
}

// Get the upper right lat,lng point
PointLL AABBLL::GetUpperRight() const {
  return PointLL(maxy_, maxx_);
}

// Get the lower left lat,lng point
PointLL AABBLL::GetLowerLeft() const {
  return PointLL(miny_, minx_);
}

// Get the lower right lat,lng point
PointLL AABBLL::GetLowerRight() const {
  return PointLL(miny_, maxx_);
}

}
}
