#ifndef VALHALLA_MJOLNIR_ADMININFOBUILDER_H_
#define VALHALLA_MJOLNIR_ADMININFOBUILDER_H_

#include <vector>
#include <string>
#include <iostream>

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Admin information not required in shortest path algorithm.
  */
class AdminInfoBuilder {
 public:
  /**
   * Set the indexes to names used by this admin
   * @param  ame_offset_list  a list of name indexes.
   */
  void set_name_offset_list(const std::vector<uint32_t>& name_offset_list);

  // Returns the size in bytes of this object.
  std::size_t SizeOf() const;

 protected:

  // List of name indexes
  std::vector<uint32_t> name_offset_list_;

  friend std::ostream& operator<<(std::ostream& os, const AdminInfoBuilder& id);

};

}
}

#endif  // VALHALLA_MJOLNIR_ADMININFOBUILDER_H_
