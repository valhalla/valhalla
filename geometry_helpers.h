// -*- mode: c++ -*-

#ifndef MM_GEOMETRY_HELPERS_H_
#define MM_GEOMETRY_HELPERS_H_

#include <algorithm>
#include <cassert>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>


namespace {

inline float
normalize(float num, float den)
{
  assert (num >= 0.f && den >= 0.f);
  return den > 0.f? std::min(std::max(num / den, 0.f), 1.f) : 0.f;
}

}


namespace mm {
namespace helpers {

using namespace valhalla;


template <typename coord_t>
inline coord_t
LocateAlong(const coord_t& start, const coord_t& end, float offset)
{
  return {start.x() + (end.x() - start.x()) * offset,
        start.y() + (end.y() - start.y()) * offset};
}


template <typename iterator_t>
float LineStringLength(const iterator_t& begin,
                       const iterator_t& end)
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
  bool open = false;

  float total_length = LineStringLength(begin, end),
  prev_vertex_length = 0.f,
       source_length = total_length * source,
       target_length = total_length * target;

  iterator_t prev_vertex = begin;
  for (auto vertex = std::next(begin); vertex != end; vertex++) {
    auto segment_length = prev_vertex->Distance(*vertex),
          vertex_length = prev_vertex_length + segment_length;

    if (!open && source_length < vertex_length) {
      auto offset = normalize(source_length - prev_vertex_length, segment_length);
      assert(0.f <= offset && offset <= 1.f);
      clip.push_back(LocateAlong(*prev_vertex, *vertex, offset));
      open = true;
    }

    if (open && target_length < vertex_length) {
      auto offset = normalize(target_length - prev_vertex_length, segment_length);
      assert(0.f <= offset && offset <= 1.f);
      clip.push_back(LocateAlong(*prev_vertex, *vertex, offset));
      open = false;
      break;
    }

    if (open) {
      clip.push_back(*vertex);
    }

    prev_vertex = vertex;
    prev_vertex_length = vertex_length;
  }

  if (clip.empty()) {
    assert(1.f == source && 1.f == target);
    clip.push_back(*prev_vertex);
    clip.push_back(*prev_vertex);
  }

  assert(clip.size() != 1);
  return clip;
}


inline float
TranslateLatitudeInMeters(float lat, float meters)
{
  float offset = meters / midgard::kMetersPerDegreeLat;
  return lat + offset;
}


inline float
TranslateLatitudeInMeters(const midgard::PointLL& lnglat, float meters)
{ return TranslateLatitudeInMeters(lnglat.lat(), meters); }


inline float
TranslateLongitudeInMeters(const midgard::PointLL& lnglat, float meters)
{
  float offset = meters / midgard::DistanceApproximator::MetersPerLngDegree(lnglat.lat());
  return lnglat.lng() + offset;
}


inline midgard::AABB2<midgard::PointLL>
ExpandMeters(const midgard::AABB2<midgard::PointLL>& bbox, float meters)
{
  if (meters < 0.f) {
    throw std::runtime_error("Expect non-negative meters");
  }

  PointLL minpt(TranslateLongitudeInMeters(bbox.minpt(), -meters),
                TranslateLatitudeInMeters(bbox.minpt(), -meters));
  PointLL maxpt(TranslateLongitudeInMeters(bbox.maxpt(), meters),
                TranslateLatitudeInMeters(bbox.maxpt(), meters));
  return {minpt, maxpt};
}


inline midgard::AABB2<midgard::PointLL>
ExpandMeters(const midgard::PointLL& pt, float meters)
{
  if (meters < 0.f) {
    throw std::runtime_error("Expect non-negative meters");
  }

  PointLL minpt(TranslateLongitudeInMeters(pt, -meters),
                TranslateLatitudeInMeters(pt, -meters));
  PointLL maxpt(TranslateLongitudeInMeters(pt, meters),
                TranslateLatitudeInMeters(pt, meters));
  return {minpt, maxpt};
}


}
}


#endif // MM_GEOMETRY_HELPERS_H_
