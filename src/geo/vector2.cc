#include "geo/vector2.h"

namespace valhalla{
namespace geo{

Vector2 operator * (float s, const Vector2 &v) {
  return Vector2(v.x() * s, v.y() * s);
}

}
}
