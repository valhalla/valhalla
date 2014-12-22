#include "mjolnir/uniquenames.h"

namespace valhalla {
namespace mjolnir {

typedef std::map<std::string, uint32_t> nameindextype;

// Constructor
UniqueNames::UniqueNames() {
  // Insert dummy so index 0 is never used
  index("");
}

// Get an index given a name. Add the name if it is not in the current list
// of unique names
uint32_t UniqueNames::index(const std::string& name) {
  // Get the iterator into the names map. If it points to a pair whose key
  // is equivalent, then return the index stored in the map entry
  uint32_t index = 0;
  auto it = names_.lower_bound(name);
  if (it != names_.end() && !(names_.key_comp()(name, it->first))) {
     index = it->second;
  }
  else {
     // Not in the map, add index and update
     it = names_.insert(it, nameindextype::value_type(name, 0));
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

}
}
