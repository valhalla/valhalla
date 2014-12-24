#ifndef VALHALLA_BALDR_EDGEINFO_H_
#define VALHALLA_BALDR_EDGEINFO_H_

#include <vector>
#include <string>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/exitsign.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
 * @author  David W. Nesbitt
 */
class EdgeInfo {
 public:
  /**
   * Constructor
   */
  EdgeInfo();

  /**
   * Get the reference node (start) of the edge.
   * @return  Returns the GraphId of the reference node of the edge.
   */
  const GraphId& nodea() const;

  /**
   * Get the end node of the edge.
   * @return  Returns the GraphId of the end node of the edge.
   */
  const GraphId& nodeb() const;

  // Returns the name index list offset
  const uint32_t street_name_offset_list_offset() const;

  // Returns the name count
  const uint32_t name_count() const;

  // Returns the shape count
  const uint32_t shape_count() const;

  // Returns the exit sign count
  const uint32_t exit_sign_count() const;

  // TODO - implement later
  // Returns the name index at the specified index.
  const size_t GetStreetNameOffset(uint8_t index) const;

//  // Returns the shape point at the specified index.
//  const PointLL* GetShapePoint(uint8_t index) const;

  /**
   * Get the shape of the edge.
   * @return  Returns the the list of lat,lng points describing the
   * *        shape of the edge.
   */
  const std::vector<PointLL>& shape() const;

  // Operator EqualTo based nodea and nodeb.
  bool operator ==(const EdgeInfo& rhs) const;

 protected:
  // Computes and returns the offset to the shape points based on the name offsets.
  const uint32_t GetShapeOffset() const;

  // Computes and returns the offset to the exit signs based on shape and name offsets.
  const uint32_t GetExitSignsOffset() const;

  // GraphIds of the 2 end nodes
  GraphId nodea_;
  GraphId nodeb_;

  union PackedItem {
    struct Fields {
      uint32_t street_name_offset_list_offset :8;
      uint32_t name_count                     :4;
      uint32_t shape_count                    :11;
      uint32_t exit_sign_count                :4;
      uint32_t spare                          :5;
    } fields;
    uint32_t value;
  };
  PackedItem item_;

 private:
  // List of roadname indexes
  size_t* street_name_offset_list_;

  // Lat,lng shape of the edge
  PointLL* shape_;

  // List of exit signs (type and index)
  ExitSign* exit_signs_;

};

}
}

#endif  // VALHALLA_BALDR_EDGEINFO_H_
