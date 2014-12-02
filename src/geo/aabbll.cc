#include "geo/aabbll.h"

namespace valhalla{
namespace geo{
  AABBLL::AABBLL() {
    minx_ = 180.0f;      // Longitude
    miny_ = -90.0f;      // Latitude
    maxx_ = 180.0f;      // Longitude
    maxy_ = 90.0f;       // Latitude
  }

  AABBLL::AABBLL(const float minlat, const float minlng,
         const float maxlat, const float maxlng) {
    minx_ = minlng;
    miny_ = minlat;
    maxx_ = maxlng;
    maxy_ = maxlat;
  }

  AABBLL::~AABBLL() { }

  float AABBLL::minlat() const {
    return miny_;
  }

  float AABBLL::minlng() const {
    return minx_;
  }

  float AABBLL::maxlat() const {
    return maxy_;
  }

  float AABBLL::maxlng() const {
    return maxx_;
  }

  PointLL AABBLL::GetUpperLeft() const {
    return PointLL(maxy_, minx_);
  }

  PointLL AABBLL::GetUpperRight() const {
    return PointLL(maxy_, maxx_);
  }

  PointLL AABBLL::GetLowerLeft() const {
    return PointLL(miny_, minx_);
  }

  PointLL AABBLL::GetLowerRight() const {
    return PointLL(miny_, maxx_);
  }
}
}
