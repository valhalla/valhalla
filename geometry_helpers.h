// -*- mode: c++ -*-

#include <algorithm>
#include <cassert>


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
    assert(1.f <= source && 1.f <= target);
    if (source == 1.f) {
      clip.push_back(*prev_vertex);
      clip.push_back(*prev_vertex);
    }
  }

  assert(clip.size() != 1);
  return clip;
}


}
}
