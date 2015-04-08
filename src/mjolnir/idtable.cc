#include "mjolnir/idtable.h"
#include <stdexcept>

namespace valhalla {
namespace mjolnir {

// Constructor to create table of OSM Node IDs being used
IdTable::IdTable(const uint64_t maxosmid): maxosmid_(maxosmid) {
  // Create a vector to mark bits. Initialize to 0.
  bitmarkers_.resize((maxosmid / 64) + 1, 0);
}

// Destructor for NodeId table
IdTable::~IdTable() {
}

// Set an OSM Id within the node table
void IdTable::set(const uint64_t id) {
  // Test if the max is exceeded
  if (id > maxosmid_) {
    throw std::runtime_error("NodeIDTable - OSM Id exceeds max specified");
  }
  bitmarkers_[id / 64] |= static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64));
}

// Check if an OSM Id is used (in the Node table)
const bool IdTable::IsUsed(const uint64_t id) const {
  return bitmarkers_[id / 64] & (static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64)));
}

}
}
