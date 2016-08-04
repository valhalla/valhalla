// -*- mode: c++ -*-
#ifndef MMP_GEOMETRY_HELPERS_H_
#define MMP_GEOMETRY_HELPERS_H_

#include <vector>
#include <algorithm>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>


namespace {

inline float
normalize(float num, float den)
{ return 0.f == den? 0.f : std::min(std::max(num / den, 0.f), 1.f); }

}


namespace valhalla{
namespace meili {
namespace helpers {


template <typename coord_t>
inline coord_t
LocateAlong(const coord_t& start, const coord_t& end, float offset)
{
  return {start.x() + (end.x() - start.x()) * offset,
          start.y() + (end.y() - start.y()) * offset};
}


template <typename iterator_t>
float LineStringLength(const iterator_t& begin, const iterator_t& end)
{
  if (begin == end) {
    return 0.f;
  }

  float length = 0.f;
  for (auto vertex = std::next(begin); vertex != end; vertex++) {
    length += std::prev(vertex)->Distance(*vertex);
  }

  return length;
}


template <typename iterator_t>
std::vector<typename iterator_t::value_type>
ClipLineString(const iterator_t& begin, const iterator_t& end,
               float source, float target)
{
  if (target < source || target < 0.f || 1.f < source || begin == end) {
    return {};
  }

  source = std::min(std::max(source, 0.f), 1.f);
  target = std::min(std::max(target, 0.f), 1.f);

  std::vector<typename iterator_t::value_type> clip;

  float total_length = LineStringLength(begin, end),
  prev_vertex_length = 0.f,
       source_length = total_length * source,
       target_length = total_length * target;

  // An state indicating if the position of current vertex is larger
  // than source and smaller than target
  bool open = false;

  // Iterate segments
  iterator_t prev_vertex = begin;
  for (auto vertex = std::next(begin); vertex != end; vertex++) {
    const auto segment_length = prev_vertex->Distance(*vertex),
                vertex_length = prev_vertex_length + segment_length;

    // Open if source is located at current segment
    if (!open && source_length < vertex_length) {
      const auto offset = normalize(source_length - prev_vertex_length, segment_length);
      clip.push_back(LocateAlong(*prev_vertex, *vertex, offset));
      open = true;
    }

    // Open -> Close if target is located at current segment
    if (open && target_length < vertex_length) {
      const auto offset = normalize(target_length - prev_vertex_length, segment_length);
      clip.push_back(LocateAlong(*prev_vertex, *vertex, offset));
      open = false;
      break;
    }

    // Add the end vertex of current segment if it is in open state
    if (open) {
      clip.push_back(*vertex);
    }

    prev_vertex = vertex;
    prev_vertex_length = vertex_length;
  }
  // assert(clip.size() != 1) because when opening state, the source
  // vertex was inserted and then followed by either target vertex
  // inserted or current vertex inserted

  if (clip.empty()) {
    // assert(1.f == source && 1.f == target)
    clip.push_back(*prev_vertex);
    clip.push_back(*prev_vertex);
  }

  // So here we have assert(1 < clip.size())

  return clip;
}


inline uint8_t
get_turn_degree180(uint16_t left, uint16_t right)
{
  if (!(left < 360 && right < 360)) {
    throw std::invalid_argument("expect angles to be within [0, 360)");
  }
  const auto turn = std::abs(left - right);
  return 180 < turn? 360 - turn : turn;
}


inline float
TranslateLatitudeInMeters(float lat, float meters)
{
  const float offset = meters / midgard::kMetersPerDegreeLat;
  return lat + offset;
}


inline float
TranslateLatitudeInMeters(const midgard::PointLL& lnglat, float meters)
{ return TranslateLatitudeInMeters(lnglat.lat(), meters); }


inline float
TranslateLongitudeInMeters(const midgard::PointLL& lnglat, float meters)
{
  const float offset = meters / midgard::DistanceApproximator::MetersPerLngDegree(lnglat.lat());
  return lnglat.lng() + offset;
}


inline midgard::AABB2<midgard::PointLL>
ExpandMeters(const midgard::AABB2<midgard::PointLL>& bbox, float meters)
{
  if (meters < 0.f) {
    throw std::invalid_argument("expect non-negative meters");
  }

  midgard::PointLL minpt(TranslateLongitudeInMeters(bbox.minpt(), -meters),
                         TranslateLatitudeInMeters(bbox.minpt(), -meters));
  midgard::PointLL maxpt(TranslateLongitudeInMeters(bbox.maxpt(), meters),
                         TranslateLatitudeInMeters(bbox.maxpt(), meters));
  return {minpt, maxpt};
}


inline midgard::AABB2<midgard::PointLL>
ExpandMeters(const midgard::PointLL& pt, float meters)
{
  if (meters < 0.f) {
    throw std::invalid_argument("expect non-negative meters");
  }

  midgard::PointLL minpt(TranslateLongitudeInMeters(pt, -meters),
                         TranslateLatitudeInMeters(pt, -meters));
  midgard::PointLL maxpt(TranslateLongitudeInMeters(pt, meters),
                         TranslateLatitudeInMeters(pt, meters));
  return {minpt, maxpt};
}


// snapped point, sqaured distance, segment index, offset
template <typename coord_t>
std::tuple<coord_t, float, typename std::vector<coord_t>::size_type, float>
Project(const coord_t& p,
        const typename std::vector<coord_t>& shape,
        const midgard::DistanceApproximator& approximator,
        float snap_distance = 0.f)
{
  if (shape.empty()) {
    throw std::invalid_argument("got empty shape");
  }

  coord_t closest_point(shape.front());
  float closest_distance = approximator.DistanceSquared(closest_point);
  decltype(shape.size()) closest_segment = 0;
  float closest_partial_length = 0.f;
  float total_length = 0.f;

  //for each segment
  for(decltype(shape.size()) i = 0; i < shape.size() - 1; ++i) {
    //project a onto b where b is the origin vector representing this segment
    //and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    const auto& u = shape[i];
    const auto& v = shape[i + 1];
    auto bx = v.first - u.first;
    auto by = v.second - u.second;
    auto sq = bx*bx + by*by;
    const auto scale = sq > 0? (((p.first - u.first)*bx + (p.second - u.second)*by) / sq) : 0.f;
    //projects along the ray before u
    if (scale <= 0.f) {
      bx = u.first;
      by = u.second;
    }//projects along the ray after v
    else if (scale >= 1.f) {
      bx = v.first;
      by = v.second;
    }//projects along the ray between u and v
    else {
      bx = bx*scale + u.first;
      by = by*scale + u.second;
    }
    //check if this point is better
    coord_t point(bx, by);
    const auto distance = approximator.DistanceSquared(point);
    if (distance < closest_distance) {
      closest_point = std::move(point);
      closest_distance = distance;
      closest_segment = i;
      closest_partial_length = total_length;
    }

    //total edge length
    total_length += u.Distance(v);
  }

  // Offset is a float between 0 and 1 representing the location of
  // the closest point on LineString to the given Point, as a fraction
  // of total 2d line length.
  closest_partial_length += shape[closest_segment].Distance(closest_point);
  float offset = total_length > 0.f? static_cast<float>(closest_partial_length / total_length) : 0.f;
  offset = std::max(0.f, std::min(offset, 1.f));

  // Snapp to vertexes if it's close
  if (total_length * offset <= snap_distance) {
    closest_point = shape.front();
    closest_distance = approximator.DistanceSquared(closest_point);
    closest_segment = 0;
    offset = 0.f;
  } else if (total_length * (1.f - offset) <= snap_distance) {
    closest_point = shape.back();
    closest_distance = approximator.DistanceSquared(closest_point);
    closest_segment = shape.size() - 1;
    offset = 1.f;
  }

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment, offset);
}

}
}
}
#endif // MMP_GEOMETRY_HELPERS_H_
