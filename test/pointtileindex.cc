#include <vector>

#include "midgard/point_tile_index.h"

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

//=========================================================================
TEST(PointTileIndex, Basic) {
  std::vector<PointLL> points = {{50, 50}, {50.5, 50}};
  std::unordered_set<size_t> near_pts;

  //--------------------------------------------------------
  {
    PointTileIndex index(0.1, points);

    near_pts = index.get_points_near(points[0]);
    EXPECT_EQ(near_pts.size(), 1);
    EXPECT_EQ(*near_pts.begin(), 0);

    near_pts = index.get_points_near(points[1]);
    EXPECT_EQ(near_pts.size(), 1);
    EXPECT_EQ(*near_pts.begin(), 1);
  }

  //--------------------------------------------------------
  {
    PointTileIndex index(0.5, points);

    std::unordered_set<size_t> exp_pts = {0, 1};

    near_pts = index.get_points_near(points[0]);
    EXPECT_EQ(near_pts.size(), 2);
    EXPECT_EQ(near_pts, exp_pts);

    near_pts = index.get_points_near(points[1]);
    EXPECT_EQ(near_pts.size(), 2);
    EXPECT_EQ(near_pts, exp_pts);
  }
}

//=========================================================================
TEST(PointTileIndex, Intermediate) {
  double min_lon = -0.001;
  double max_lon = 0.001;
  double min_lat = min_lon;
  double max_lat = max_lon;
  int grid_dim = 19; // any odd number will do
  double width = (max_lat - min_lat) / (grid_dim - 1);
  PointLL origin_pt{0, 0};
  size_t origin_idx = (grid_dim * grid_dim) / 2; // idx for origin point
  double lat, lon;
  std::unordered_set<size_t> near_pts;

  // a grid of points
  std::vector<PointLL> points;
  for (int x = 0; x < grid_dim; x++) {
    lon = min_lon + x * width;
    for (int y = 0; y < grid_dim; y++) {
      lat = min_lat + y * width;
      points.emplace_back(PointLL{lon, lat});
    }
  }

  //--------------------------------------------------------
  // Tile space using 'width'. Ask for near points around the origin,
  // then reduce those near points to those exactly within 'width'.
  //--------------------------------------------------------
  {
    // index space using the 2d distance between each point. Keep in mind
    // that doesn't mean that space will be divided into that exact width;
    // space will (almost always) be divided into a width that is a bit
    // larger than the width given.
    PointTileIndex index(width, points);

    near_pts = index.get_points_near(origin_pt);

    // remember the index returns points that are close, certainly
    // some will be outside the width we specified when we tiled space.
    // as such, now reduce the near_pts to those that are exactly within
    // the desired width.
    constexpr double eps = 1e-10;
    std::unordered_set<size_t> exact_near_pts;
    for (size_t idx : near_pts) {
      const PointLL& near_pt = index.points[idx];
      double dx = origin_pt.x() - near_pt.x(), dy = origin_pt.y() - near_pt.y();
      double pure_pythagorean_dist = std::sqrt(dx * dx + dy * dy);
      if (pure_pythagorean_dist < width + eps) {
        exact_near_pts.insert(idx);
      }
    }

    // The expected points form a "plus" sign at the origin.
    EXPECT_EQ(exact_near_pts.size(), 5);

    // turns out we can compute the index of each points around the origin
    // based on the grid size
    size_t tidx = origin_idx + 1;        // idx for point above origin
    size_t bidx = origin_idx - 1;        // idx for point below origin
    size_t lidx = origin_idx - grid_dim; // idx for point left of origin
    size_t ridx = origin_idx + grid_dim; // idx for point right of origin
    std::unordered_set<size_t> exp_pts{lidx, bidx, origin_idx, tidx, ridx};
    EXPECT_EQ(exact_near_pts, exp_pts);
  }

  //--------------------------------------------------------
  // Tile space using width/10.0. Add a cluster of points around the
  // origin. Ask for near points around the origin, see that the
  // origin point and the cluster is returned.
  //--------------------------------------------------------
  {
    std::vector<PointLL> points_enhanced(points);
    size_t circle_pt_idx = points_enhanced.size();
    std::unordered_set<size_t> circle_pt_indicies;

    // add a bunch of points tightly circled around the origin
    int num_circle_pts = 100;
    for (int i = 0; i < num_circle_pts; i++) {
      double radians = 2 * M_PI * ((double)i / (double)num_circle_pts);
      double x = std::cos(radians), y = std::sin(radians);
      constexpr double len = 1e-7;
      points_enhanced.emplace_back(PointLL{len * x, len * y});
      circle_pt_indicies.insert(circle_pt_idx++);
    }

    // a tighter tiling width this time
    PointTileIndex index(width / 10.0, points_enhanced);

    // with the tighter tiling, asking for points around the origin
    // should only pick up the origin point and the circle points
    // we added.
    near_pts = index.get_points_near(origin_pt);
    EXPECT_EQ(near_pts.size(), num_circle_pts + 1);

    circle_pt_indicies.insert(origin_idx);
    EXPECT_EQ(near_pts, circle_pt_indicies);
  }

  //--------------------------------------------------------
  // Tile space using width/3. Use get_points_near_segment() to query
  // for the points along each of the four sides.
  //--------------------------------------------------------
  {
    PointTileIndex index(width / 3.0, points);

    PointLL sw{min_lon, min_lat};
    PointLL se{max_lon, min_lat};
    PointLL nw{min_lon, max_lat};
    PointLL ne{max_lon, max_lat};

    //------- left ---------------------------------
    std::unordered_set<size_t> exp_left_indices;
    for (int i = 0; i < grid_dim; i++) {
      size_t idx = i;
      exp_left_indices.insert(idx);
    }

    LineSegment2<PointLL> left_side{sw, nw};
    near_pts = index.get_points_near_segment(left_side);

    EXPECT_EQ(near_pts.size(), exp_left_indices.size());
    EXPECT_EQ(near_pts, exp_left_indices);

    //------- bottom ---------------------------------
    std::unordered_set<size_t> exp_bottom_indices;
    for (int i = 0; i < grid_dim; i++) {
      size_t idx = grid_dim * i;
      exp_bottom_indices.insert(idx);
    }

    LineSegment2<PointLL> bottom_side{sw, se};
    near_pts = index.get_points_near_segment(bottom_side);

    EXPECT_EQ(near_pts.size(), exp_bottom_indices.size());
    EXPECT_EQ(near_pts, exp_bottom_indices);

    //------- right ---------------------------------
    std::unordered_set<size_t> exp_right_indices;
    for (int i = 0; i < grid_dim; i++) {
      size_t idx = grid_dim * (grid_dim - 1) + i;
      exp_right_indices.insert(idx);
    }

    LineSegment2<PointLL> right_side{se, ne};
    near_pts = index.get_points_near_segment(right_side);

    EXPECT_EQ(near_pts.size(), exp_right_indices.size());
    EXPECT_EQ(near_pts, exp_right_indices);

    //------- top ---------------------------------
    std::unordered_set<size_t> exp_top_indices;
    for (int i = 0; i < grid_dim; i++) {
      size_t idx = grid_dim * (i + 1) - 1;
      exp_top_indices.insert(idx);
    }

    LineSegment2<PointLL> top_side{nw, ne};
    near_pts = index.get_points_near_segment(top_side);

    EXPECT_EQ(near_pts.size(), exp_top_indices.size());
    EXPECT_EQ(near_pts, exp_top_indices);
  }
}