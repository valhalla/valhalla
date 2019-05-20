#ifndef VALHALLA_MJOLNIR_IDTABLE_H
#define VALHALLA_MJOLNIR_IDTABLE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <midgard/logging.h>
#include <vector>

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
  IdTable(const uint64_t maxosmid) {
    // Create a vector to mark bits. Initialize to 0.
    bitmarkers_.resize((maxosmid / 64) + 1, 0);
    // Set this to the actual maximum as dictated by the storage
    maxosmid_ = bitmarkers_.size() * 64 - 1;
  }

  /**
   * Destructor
   */
  ~IdTable() {
  }

  /**
   * Sets the OSM Id as used.
   * @param   osmid   OSM Id of the way/node/relation.
   */
  inline void set(const uint64_t id) {
    auto idx = maybe_resize(id);
    bitmarkers_[idx] |= static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64));
  }

  /**
   * Test if the OSM Id is used / set in the bitmarker.
   * @param  id  OSM Id
   * @return  Returns true if the OSM Id is used. False if not.
   */
  inline const bool get(const uint64_t id) {
    return id > maxosmid_ ? false
                          : bitmarkers_[id / 64] &
                                (static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64)));
  }

  /**
   * Gets the maximum current id that could be found in the set
   * @return maximum id
   */
  inline uint64_t max() const {
    return maxosmid_;
  }

private:
  /**
   * Resizes the internal storage and adjusts maxosmid_ if needed
   * the idea is that we dont need to actually do this very often
   *
   * @param the id which needs to fit in storage
   * @returns the index holding the bit
   */
  inline uint64_t maybe_resize(const uint64_t id) {
    // we dont double it because that could be huge, so we stay conservative
    uint64_t idx = id / 64;
    if (id > maxosmid_) {
      LOG_WARN("Max osmid exceeded bitset, resizing to fit id: " + std::to_string(id));
      bitmarkers_.resize(std::ceil(idx * 1.01 + 1), 0);
      maxosmid_ = bitmarkers_.size() * 64 - 1;
    }
    return idx;
  }

  uint64_t maxosmid_;
  std::vector<uint64_t> bitmarkers_;
};
} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_IDTABLE_H
