
#include "baldr/json.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "tyr/serializers.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>
#include "midgard/point_tile_index.h"

using namespace valhalla::baldr::json;

namespace {
using rgba_t = std::tuple<float, float, float>;
}

namespace valhalla {
namespace tyr {



template <typename coord_t>
int self_intersects(const std::vector<coord_t>& points) {
  int num_self_intersections = 0;
  for (size_t i = 1; i < points.size() - 2; i++) {
    const coord_t& ia(points[i - 1]);
    const coord_t& ib(points[i]);
    for (size_t j = i + 2; j < points.size() - 1; j++) {
      const coord_t& ja(points[j - 1]);
      const coord_t& jb(points[j]);
      LineSegment2<coord_t> segmenti(ia, ib);
      LineSegment2<coord_t> segmentj(ja, jb);
      coord_t intersection_point;
      if (segmenti.Intersect(segmentj, intersection_point)) {
        num_self_intersections++;
        printf("Orig Intersection:\n");
        printf("Line: (%.7f, %.7f), (%.7f, %.7f)\n", ia.lat(), ia.lng(), ib.lat(),
               ib.lng()); printf("Line: (%.7f, %.7f), (%.7f, %.7f)\n", ja.lat(), ja.lng(),
                                 jb.lat(), jb.lng());
      }
    }
  }

  return num_self_intersections;
}

template<class container_t>
int self_intersects(const container_t& points) {

  struct segment {
    size_t ai, bi;
    bool operator==(const segment& s) const {
      return (ai == s.ai) && (bi == s.bi);
    }
  };

  struct segment_hash_functor {
    size_t operator()(const segment& s) const {
      return 2*s.ai + s.bi;
    }
  };

  int num_self_intersections = 0;

  if (points.size() < 4)
    return num_self_intersections;

//  AABB2<PointLL> box(PointLL{6.220698, 51.226051}, PointLL{6.234257, 51.233920});
//
//  bool all_contained = true;

  double total_seg_len_m = 0.0;
  std::vector<PointLL> vpoints;
  vpoints.reserve(points.size());
  auto prev_pt_iter = points.begin();
  vpoints.emplace_back(*prev_pt_iter);
  auto curr_pt_iter = prev_pt_iter++;
  for (; curr_pt_iter != points.end(); curr_pt_iter++) {
    const PointLL& pta = *prev_pt_iter;
    const PointLL& ptb = *curr_pt_iter;
//    all_contained = all_contained && box.Contains(pta) && box.Contains(ptb);
    double seg_len_m = pta.Distance(ptb);
    total_seg_len_m += seg_len_m;
    vpoints.emplace_back(*curr_pt_iter);
    prev_pt_iter = curr_pt_iter;
  }

//  if (all_contained)
//    int a = 4;

  // this distance works well for polylines from isochrones... but I wonder
  // if it can be derived (based on average polyline seg length above or ?).
  constexpr double max_seg_len_m = 50.0;

  // As far as I can tell this doesn't preserve the original points. This algo needs
  // the original polyline points preseved. However, resampling does demonstrate a
  // huge speedup. Also, I think the resampling routine should perform the distance
  // computation between polyline points, e.g., the total length shouldn't have to
  // be passed in.
  std::vector<PointLL> resampled_pts = resample_polyline(vpoints, total_seg_len_m, max_seg_len_m);

  // This has the option to preserve points but doesn't really make sense for this
  // routine which works strictly in a flattened earth manner.
  //std::vector<PointLL> resampled_pts = resample_spherical_polyline(vpoints, max_seg_len_m, true);

  const PointLL& pt = resampled_pts[0];
  double metersPerDegreeLon = std::fabs(DistanceApproximator<PointLL>::MetersPerLngDegree(pt.lng()));
  double deg_width = max_seg_len_m / metersPerDegreeLon;

  PointTileIndex point_tile_index(deg_width / 2.0, resampled_pts);

  PointLL intersection_point;
  std::unordered_set<size_t> near_pts;
  std::unordered_set<segment, segment_hash_functor> global_segments_tested;
  for (size_t i = 1; i < point_tile_index.points.size(); i++) {
    const PointLL& ia(point_tile_index.points[i - 1]);
    const PointLL& ib(point_tile_index.points[i]);
    auto segmenti = LineSegment2<PointLL>{ia, ib};

    near_pts = point_tile_index.get_points_near_segment(segmenti);

    global_segments_tested.insert(segment{i, i-1});
    global_segments_tested.insert(segment{i-1, i});

    std::unordered_set<segment, segment_hash_functor> local_segments_tested;

    for (size_t near_pt_idx : near_pts) {

      // We have point near our segment. From this point we can form
      // two segments: one using it's previous point, and one using it's
      // next. In both cases we want to make sure we're not connected to
      // the (i, i-1) segment. We are only interested in segments that
      // intersect along their length; not just ones that are connected.
      const auto& near_pt = point_tile_index.points[near_pt_idx];

      // avoid stepping off the front. Grab the previous point to
      // the current one. make sure we're not connected to the segment
      // (i, i-1). Form a segment if so and conduct the intersection test.
      if (near_pt_idx > 0) {
        size_t prev_pt_idx = near_pt_idx - 1;

        // not interested in connected segments
        if ((prev_pt_idx == i) || (prev_pt_idx == i-1))
          continue;
        if ((near_pt_idx == i) || (prev_pt_idx == i-1))
          continue;

        // boundary case where the polygon connects to itself
        if ((i-1 == 0) && (near_pt_idx == point_tile_index.points.size()-1))
          continue;
        if ((i == point_tile_index.points.size()-1) && (prev_pt_idx == 0))
          continue;

        // avoid retesting the same segment
        if (local_segments_tested.find(segment{prev_pt_idx, near_pt_idx}) != local_segments_tested.end())
          continue;
        // avoid retesting the same segment
        if (global_segments_tested.find(segment{prev_pt_idx, near_pt_idx}) != global_segments_tested.end())
          continue;

        // mark segment as tested
        local_segments_tested.insert(segment{prev_pt_idx, near_pt_idx});
        local_segments_tested.insert(segment{near_pt_idx, prev_pt_idx});

        // actually do the intersection test
        const auto& prev_pt = point_tile_index.points[prev_pt_idx];
        auto segmentj = LineSegment2<PointLL>{prev_pt, near_pt};
        if (segmenti.Intersect(segmentj, intersection_point)) {
          printf("idx: %zu, a: %.6f, %.6f\n", i-1, ia.lat(), ia.lng());
          printf("idx: %zu, b: %.6f, %.6f\n", i, ib.lat(), ib.lng());
          printf("idx: %zu, c: %.6f, %.6f\n", near_pt_idx, near_pt.lat(), near_pt.lng());
          printf("idx: %zu, d: %.6f, %.6f\n", prev_pt_idx, prev_pt.lat(), prev_pt.lng());
          printf("i: %.6f, %.6f\n", intersection_point.lat(), intersection_point.lng());
          num_self_intersections++;
        }
      }

      // avoid reaching beyond the end. Grab the next point from
      // the current one. make sure we're not connected to the segment
      // (i, i-1). Form a segment if so and conduct the intersection test.
      if (near_pt_idx < point_tile_index.points.size()-1) {
        size_t next_pt_idx = near_pt_idx + 1;

        // not interested in connected segments
        if ((next_pt_idx == i) || (next_pt_idx == i-1))
          continue;
        if ((near_pt_idx == i) || (next_pt_idx == i-1))
          continue;

        // boundary case where the polygon connects to itself
        if ((i-1 == 0) && (next_pt_idx == point_tile_index.points.size()-1))
          continue;
        if ((i == point_tile_index.points.size()-1) && (near_pt_idx == 0))
          continue;

        // avoid retesting the same segment
        if (local_segments_tested.find(segment{near_pt_idx, next_pt_idx}) != local_segments_tested.end())
          continue;
        // avoid retesting the same segment
        if (global_segments_tested.find(segment{near_pt_idx, next_pt_idx}) != global_segments_tested.end())
          continue;

        // mark segment as tested
        local_segments_tested.insert(segment{near_pt_idx, next_pt_idx});
        local_segments_tested.insert(segment{next_pt_idx, near_pt_idx});

        // actually do the intersection test
        const auto& next_pt = point_tile_index.points[next_pt_idx];
        auto segmentj = LineSegment2<PointLL>{near_pt, next_pt};
        if (segmenti.Intersect(segmentj, intersection_point)) {
          printf("idx: %zu, a: %.6f, %.6f\n", i-1, ia.lat(), ia.lng());
          printf("idx: %zu, b: %.6f, %.6f\n", i, ib.lat(), ib.lng());
          printf("idx: %zu, c: %.6f, %.6f\n", near_pt_idx, near_pt.lat(), near_pt.lng());
          printf("idx: %zu, d: %.6f, %.6f\n", next_pt_idx, next_pt.lat(), next_pt.lng());
          printf("i: %.6f, %.6f\n", intersection_point.lat(), intersection_point.lng());
          num_self_intersections++;
        }
      }
    }
  }

  return num_self_intersections;
}


bool check_for_intersections(std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                             midgard::GriddedData<2>::contours_t& contours) {

  clock_t start_time = clock();

  int total_self_intersections = 0;
  int cnt = 0;

  int total_new_self_int_time = 0;
  int total_orig_self_int_time = 0;

  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& feature_collection = contours[contour_index];
    for (const auto& feature : feature_collection) {
      for (const auto& contour : feature) {

        auto start_new_time = std::chrono::high_resolution_clock::now();
//        int num_self_intersections_new = self_intersects<std::list<GeoPoint<double>>>(contour);
        auto end_new_time = std::chrono::high_resolution_clock::now();
        total_new_self_int_time = total_new_self_int_time + std::chrono::duration_cast<std::chrono::milliseconds>(end_new_time - start_new_time).count();

        auto start_orig_time = std::chrono::high_resolution_clock::now();
        std::vector<GeoPoint<double>> points;
        for (const auto& coord : contour) {
          points.emplace_back(coord);
        }
        int num_self_intersections_orig = self_intersects(points);
        auto end_orig_time = std::chrono::high_resolution_clock::now();
        total_orig_self_int_time = total_orig_self_int_time + std::chrono::duration_cast<std::chrono::milliseconds>(end_orig_time - start_orig_time).count();

//        if (num_self_intersections_new != num_self_intersections_orig)
//          printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

        total_self_intersections += num_self_intersections_orig;
      }
    }
  }

  printf("Orig self-intersection check time: %d\n", total_orig_self_int_time);
  printf(" New self-intersection check time: %d\n", total_new_self_int_time);

  printf("Number self-intersections: %d\n", total_self_intersections);

  return total_self_intersections > 0;
}



std::string serializeIsochrones(const Api& request,
                                std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                                midgard::GriddedData<2>::contours_t& contours,
                                bool polygons,
                                bool show_locations) {

  bool inter1 = check_for_intersections(intervals, contours);


  // for each contour interval
  int i = 0;
  auto features = array({});
  assert(intervals.size() == contours.size());
  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& feature_collection = contours[contour_index];

    // color was supplied
    std::stringstream hex;
    if (!std::get<3>(interval).empty()) {
      hex << "#" << std::get<3>(interval);
    } // or we computed it..
    else {
      auto h = i * (150.f / intervals.size());
      auto c = .5f;
      auto x = c * (1 - std::abs(std::fmod(h / 60.f, 2.f) - 1));
      auto m = .25f;
      rgba_t color = h < 60 ? rgba_t{m + c, m + x, m}
                            : (h < 120 ? rgba_t{m + x, m + c, m} : rgba_t{m, m + c, m + x});
      hex << "#" << std::hex << static_cast<int>(std::get<0>(color) * 255 + .5f) << std::hex
          << static_cast<int>(std::get<1>(color) * 255 + .5f) << std::hex
          << static_cast<int>(std::get<2>(color) * 255 + .5f);
    }
    ++i;

    // for each feature on that interval
    for (const auto& feature : feature_collection) {
      // for each contour in that feature
      auto geom = array({});
      for (const auto& contour : feature) {
        // make some geometry
        auto coords = array({});
        for (const auto& coord : contour) {
          coords->push_back(array({fp_t{coord.first, 6}, fp_t{coord.second, 6}}));
        }
        // its either a ring
        if (polygons) {
          geom->emplace_back(coords);
          // or a single line, if someone has more than one contour per feature they messed up
        } else {
          geom = coords;
        }
      }
      // add a feature
      features->emplace_back(map({
          {"type", std::string("Feature")},
          {"geometry", map({
                           {"type", std::string(polygons ? "Polygon" : "LineString")},
                           {"coordinates", geom},
                       })},
          {"properties", map({
                             {"metric", std::get<2>(interval)},
                             {"contour", static_cast<uint64_t>(std::get<1>(interval))},
                             {"color", hex.str()},            // lines
                             {"fill", hex.str()},             // geojson.io polys
                             {"fillColor", hex.str()},        // leaflet polys
                             {"opacity", fp_t{.33f, 2}},      // lines
                             {"fill-opacity", fp_t{.33f, 2}}, // geojson.io polys
                             {"fillOpacity", fp_t{.33f, 2}},  // leaflet polys
                         })},
      }));
    }
  }
  // Add input and snapped locations to the geojson
  if (show_locations) {
    int idx = 0;
    for (const auto& location : request.options().locations()) {
      // first add all snapped points as MultiPoint feature per origin point
      auto snapped_points_array = array({});
      std::unordered_set<midgard::PointLL> snapped_points;
      for (const auto& path_edge : location.path_edges()) {
        const midgard::PointLL& snapped_current =
            midgard::PointLL(path_edge.ll().lng(), path_edge.ll().lat());
        // remove duplicates of path_edges in case the snapped object is a node
        if (snapped_points.insert(snapped_current).second) {
          snapped_points_array->push_back(
              array({fp_t{snapped_current.lng(), 6}, fp_t{snapped_current.lat(), 6}}));
        }
      };
      features->emplace_back(map(
          {{"type", std::string("Feature")},
           {"properties",
            map({{"type", std::string("snapped")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry",
            map({{"type", std::string("MultiPoint")}, {"coordinates", snapped_points_array}})}}));

      // then each user input point as separate Point feature
      const valhalla::LatLng& input_latlng = location.ll();
      const auto input_array = array({fp_t{input_latlng.lng(), 6}, fp_t{input_latlng.lat(), 6}});
      features->emplace_back(map(
          {{"type", std::string("Feature")},
           {"properties",
            map({{"type", std::string("input")}, {"location_index", static_cast<uint64_t>(idx)}})},
           {"geometry", map({{"type", std::string("Point")}, {"coordinates", input_array}})}}));
      idx++;
    }
  }

  // make the collection
  auto feature_collection = map({
      {"type", std::string("FeatureCollection")},
      {"features", features},
  });

  if (request.options().has_id()) {
    feature_collection->emplace("id", request.options().id());
  }

  std::stringstream ss;
  ss << *feature_collection;

  return ss.str();
}
} // namespace tyr
} // namespace valhalla
