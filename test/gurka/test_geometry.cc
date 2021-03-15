
#include "meili/geometry_helpers.h"
#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::midgard;

TEST(geometry, simple_projection) {
  PointLL a(13.262611, 38.159604000000002);
  PointLL b(13.262670, 38.159586999999995);
  PointLL gps_point{13.2626372, 38.159659599999998};

  float d0 = std::sqrt(a.DistanceSquared(b));
  float d1 = std::sqrt(b.DistanceSquared(a));

  PointLL pp(13.262611253749965, 38.159603926885602);

  float k0 = std::sqrt(a.DistanceSquared(pp));
  float k1 = std::sqrt(b.DistanceSquared(pp));

  float per0 = std::sqrt(a.DistanceSquared(pp)) / d0;
  float per1 = std::sqrt(b.DistanceSquared(pp)) / d0;

  float pert = per0 + per1;

  std::vector<PointLL> dir0 = {a, b};
  std::string enc_shape = midgard::encode7(dir0);
  midgard::Shape7Decoder<midgard::PointLL> shape0(enc_shape.c_str(), enc_shape.size());
  float sq_distance0 = -1.f;
  size_t segment0;
  float percentage_along0 = -1.f;
  PointLL projection_point0;
  std::tie(projection_point0, sq_distance0, segment0, percentage_along0) =
      valhalla::meili::helpers::Project(gps_point, shape0);

  std::vector<PointLL> dir1 = {b, a};
  enc_shape = midgard::encode7(dir1);
  midgard::Shape7Decoder<midgard::PointLL> shape1(enc_shape.c_str(), enc_shape.size());
  float sq_distance1 = -1.f;
  size_t segment1;
  float percentage_along1 = -1.f;
  PointLL projection_point1;
  std::tie(projection_point1, sq_distance1, segment1, percentage_along1) =
      valhalla::meili::helpers::Project(gps_point, shape1);

  int t = 4;
}

TEST(geometry, real_world_projection) {
  // This is the real world shape that was previously problematic.
  const char enc_shape[] = { -24, -109, -78, 36, -90, -4, -46, 12, 33, 118, 71, 16, 117, 47, 41, 91,
                             113, 44, 45, 112, -125, 1, 80, -67, 1, 54, -1, 1, 117, -31, 2, -43, 1,
                            -33, 1, 21, -93, 2, 15, -25, 1, 47, -97, 1, 86, -123, 1, -64, 1, 51, -110,
                            2, 48, -90, 1, 29, -84, 1, 67, -94, 2, 84, -10, 1, -86, 1, 82, -106, 2,
                            70, -32, 1, 64, -96, 3, -20, 1, -126, 2, 112, -56, 3, 26, -104, 3, 15,
                            -30, 3, 101, -48, 3, -87, 1, -28, 3, -107, 1, -62, 1, 113, 118, -81, 1,
                            110, 27, 56, -106, 1, 71, -78, 1, -67, 1, -122, 2, -7, 1, -40, 1, -81, 1,
                            -86, 1, 17, 108, 26, -84, 1, -100, 1, 54, -62, 1, -111, 2, -120, 3, -127,
                            2, -16, 1, 127, -62, 1, -43, 1, -32, 1, -67, 2, 38, -49, 1, -100, 1, 64,
                            14, -74, 1, 105, -100, 2, -125, 1, -44, 3, -81, 1, -126, 3, -93, 1, -64,
                            1, -67, 2, -82, 3, -85, 1, -36, 1, -75, 1, 48, -67, 1, -128, 1, -111, 1,
                            -72, 2, -59, 2, -8, 3, -113, 3, -40, 3, -85, 2, -126, 2, -43, 2, -68, 2,
                            -17, 1, -46, 1, -35, 2, -4, 2, 42, -106, 1, -96, 1, 48, -82, 7, -38, 4,
                            -102, 2, -90, 1, -74, 3, -36, 1, -16, 1, 47, -38, 2, -105, 2, -70, 1, -37,
                            1, -34, 2, -85, 2, -44, 1, 117, -106, 2, 91, -42, 2, -68, 1, -18, 2, -126,
                            1, -62, 7, -62, 2, -4, 2, -106, 1, -2, 3, -118, 1, -78, 1, -118, 1 };

  // These are the exact gps points that exposed the issue. (Technically, only the
  // first point exposed the issue - but I'm keeping this real-world.) Both project to the
  // very end of the above shape. After projection, both should have a "percentage_along"
  // of exactly 0.0.
  const PointLL gps_points[] = {{13.2626372, 38.159659599999998},
                                {13.262609100000001, 38.159699600000003}};

  // first pass - shape in give order
  for (const auto gps_point : gps_points) {
    float sq_distance = -1.f;
    size_t segment;
    float percentage_along = -1.f;
    PointLL projection_point;

    // meili::helpers::Project() consumes the incoming shape so we have to
    // create it with each iteration.
    midgard::Shape7Decoder<midgard::PointLL> shape(enc_shape,
                                                   sizeof(enc_shape) / sizeof(*enc_shape));

    std::tie(projection_point, sq_distance, segment, percentage_along) =
        valhalla::meili::helpers::Project(gps_point, shape);

    // Here's what we're testing: that both points return percentage_along
    // of exactly zero. Previously the first point was returning a percentage along
    // of ~1e-9 which is lost in the computation 1.0-1e-9 because floats only have
    // ~7.2 decimal digits of precision.
    //ASSERT_EQ(percentage_along, 0.0);

    // might as well assert the other reasonable aspect, that both points project to
    // the same segment.
    ASSERT_EQ(segment, 0);
  }

  //------------------------------------------
  // second pass - shape in reverse order
  midgard::Shape7Decoder<midgard::PointLL> rev_shape(enc_shape,
                                                     sizeof(enc_shape) / sizeof(*enc_shape));

  std::vector<PointLL> rev_shape_points;
  while (!rev_shape.empty())
    rev_shape_points.emplace_back(rev_shape.pop());

  std::reverse(rev_shape_points.begin(), rev_shape_points.end());

  std::string rev_enc_shape = midgard::encode7(rev_shape_points);

  for (const auto gps_point : gps_points) {
    float sq_distance = -1.f;
    size_t segment;
    float percentage_along = -1.f;
    PointLL projection_point;

    // meili::helpers::Project() consumes the incoming shape so we have to
    // create it with each iteration.
    midgard::Shape7Decoder<midgard::PointLL> shape(rev_enc_shape.c_str(), rev_enc_shape.size());

    std::tie(projection_point, sq_distance, segment, percentage_along) =
        valhalla::meili::helpers::Project(gps_point, shape);

    // closest point:
    // (13.262611253749965, 38.159603926885602)

    // Here's what we're testing: that both points return percentage_along
    // of exactly zero. Previously the first point was returning a percentage along
    // of ~1e-9 which is lost in the computation 1.0-1e-9 because floats only have
    // ~7.2 decimal digits of precision.
    //ASSERT_EQ(percentage_along, 0.0);

    // might as well assert the other reasonable aspect, that both points project to
    // the same segment.
    ASSERT_EQ(segment, 80);
  }
}
