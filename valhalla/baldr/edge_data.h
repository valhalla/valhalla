#ifndef VALHALLA_BALDR_EDGE_DATA_H_
#define VALHALLA_BALDR_EDGE_DATA_H_

namespace valhalla {
namespace baldr {

/** 
 * Contains the data for a certain edge
 */
class EdgeData {
public:
  float type() const;

  void set_type(const float type);

  float distance() const;

  void set_distance(const float distance);

  float percentage() const;

  void set_percentage(const float percentage);

  int start_index() const;

  void set_start_index(const int start_index);

  int end_index() const;

  void set_end_index(const int end_index);

  bool operator <(const EdgeData& edge_data) const;

  void operator +=(const EdgeData& edge_data);
  
protected:
  float type_;
  float distance_;
  float percentage_;
  int start_index_;
  int end_index_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_EDGE_DATA_H_
