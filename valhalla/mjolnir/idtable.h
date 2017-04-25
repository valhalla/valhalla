#ifndef VALHALLA_MJOLNIR_IDTABLE_H
#define VALHALLA_MJOLNIR_IDTABLE_H

#include <cstdint>
#include <vector>
#include <algorithm>

namespace valhalla {
namespace mjolnir {

/**
 * A method for marking OSM Ids that are used by ways/nodes/relations.
 * Uses a vector where 1 bit is used for each possible Id.
 * So for a maximum OSM Id of 4 billion this uses 500MB memory
 */
class IdTable {
 public:
   /**
    * Constructor
    * @param   maxosmid   Maximum OSM Id to support.
    */
  IdTable(const uint64_t maxosmid);

  /**
   * Destructor
   */
  ~IdTable();

  /**
   * Sets the OSM Id as used.
   * @param   osmid   OSM Id of the way/node/relation.
   */
  void set(const uint64_t id);

  /**
   * Test if the OSM Id is used / set in the bitmarker.
   * @param  id  OSM Id
   * @return  Returns true if the OSM Id is used. False if not.
   */
  const bool IsUsed(const uint64_t id) const;

 private:
  const uint64_t maxosmid_;
  std::vector<uint64_t> bitmarkers_;
};
}
}

#endif  // VALHALLA_MJOLNIR_IDTABLE_H
