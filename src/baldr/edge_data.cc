#include "baldr/edge_data.h"
#include <vector>

namespace valhalla {
namespace baldr {

  float EdgeData::type() const {
    return type_;
  }

  void EdgeData::set_type(const float type) {
    type_ = type;
  }

  float EdgeData::distance() const {
    return distance_;
  }

  void EdgeData::set_distance(const float distance) {
    distance_ = distance;
  }

  float EdgeData::percentage() const {
    return percentage_;
  }

  void EdgeData::set_percentage(const float percentage) {
    percentage_ = percentage;
  }

  int EdgeData::start_index() const {
    return start_index_;
  }

  void EdgeData::set_start_index(const int  start_index) {
    start_index_ = start_index;
  }

  int EdgeData::end_index() const {
    return end_index_;
  }

  void EdgeData::set_end_index(const int end_index) {
    end_index_ = end_index;
  }

  bool EdgeData::operator <(const EdgeData &edge_data) const{
    return type() < edge_data.type();
  }

  void EdgeData::operator +=(const EdgeData &edge_data) {
    set_distance(distance() + edge_data.distance());
    set_percentage(percentage() + edge_data.percentage());
  }

  void EdgeData::operator +=(const float new_distance) {
    set_distance(distance() + new_distance);
  }
} // namespace baldr
} // namespace valhalla