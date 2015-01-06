#ifndef VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_

#include <vector>
#include <string>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/exitsign.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
 */
class EdgeInfoBuilder : public baldr::EdgeInfo {
 public:
  /**
   * Constructor
   */
  EdgeInfoBuilder();

  /**
   * Copy Constructor
   */
  EdgeInfoBuilder(const EdgeInfoBuilder& other);

  /**
   * Copy Assignment
   */
  EdgeInfoBuilder& operator=(const EdgeInfoBuilder& rhs);

  /**
   * Destrcutor
   */
  virtual ~EdgeInfoBuilder();

  /**
   * Set the reference node (start) of the edge.
   * @param  nodea  the GraphId of the reference node of the edge.
   */
  void set_nodea(const baldr::GraphId& nodea);

  /**
   * Set the end node of the edge.
   * @param  nodeb  the GraphId of the end node of the edge.
   */
  void set_nodeb(const baldr::GraphId& nodeb);

  /**
   * Set the indexes to names used by this edge
   * @param  nameindexes  a list of name indexes.
   */
  void set_street_name_offset_list(const std::vector<size_t>& street_name_offset_list);

  /**
   * Set the shape of the edge.
   * @param  shape  the the list of lat,lng points describing the
   * *        shape of the edge.
   */
  void set_shape(const std::vector<PointLL>& shape);

  // Returns the name index at the specified index.
  const size_t GetStreetNameOffset(uint8_t index) const;

  // Returns the shape point at the specified index.
  // TODO: replace with vector once it works
  const PointLL GetShapePoint(uint16_t index) const;

  // Returns the size in bytes of this object.
  std::size_t SizeOf() const;

  void SerializeToOstream(std::ostream& out) const;

 private:

  // List of roadname indexes
  std::vector<size_t> street_name_offset_list_;

  // Lat,lng shape of the edge
  std::vector<PointLL> shape_;

  // List of exit signs (type and index)
  std::vector<ExitSign> exit_signs_;

};

}
}

#endif  // VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
