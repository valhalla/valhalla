#include "mjolnir/uniquenames.h"

#include "midgard/logging.h"

namespace valhalla {
namespace mjolnir {

// Constructor
UniqueNames::UniqueNames() {
  // Insert dummy so index 0 is never used
  index("");
}

// Get an index given a name. Add the name if it is not in the current list
// of unique names
uint32_t UniqueNames::index(const std::string& name) {
  // Find the name in the map. If it is there return the index.
  uint32_t index = 0;
  auto it = names_.find(name);
  if (it != names_.end()) {
     index = it->second;
  }
  else {
     // Not in the map, add index and update
     it = names_.insert(it, NamesMap::value_type(name, 0));
     indexes_.push_back(it);
     index = indexes_.size() - 1;
     it->second = index;
  }
  return index;
}

// Get the name given the index
const std::string& UniqueNames::name(const uint32_t index) const {
  if (index < (uint32_t)indexes_.size())
    return indexes_[index]->first;

  // Return the empty string in the index 0 location
  return indexes_[0]->first;
}

// Clear the unique names list
void UniqueNames::Clear() {
  names_.clear();
  indexes_.clear();
}

// Get the number of unique names. Since a blank name is added as the first
// unique name we return the size of the map - 1.
size_t UniqueNames::Size() const {
  return names_.size() - 1;
}

/**
 * Log information about the number of unique names, size of the vector, etc.
 */
void UniqueNames::Log() const {
  LOG_DEBUG("Number of names: " + std::to_string(Size()));
  LOG_DEBUG("Number of indexes: " + std::to_string(indexes_.size()));
}

}
}
