#ifndef VALHALLA_MJOLNIR_UNIQUENAMES_H
#define VALHALLA_MJOLNIR_UNIQUENAMES_H

#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace mjolnir {

using NamesMap = std::unordered_map<std::string, uint32_t>;

/**
 * Class to hold a list of unique names and indexes to them.
 */
class UniqueNames {
public:
  /**
   * Constructor.
   */
  UniqueNames() {
    // Insert dummy so index 0 is never used
    index("");
  }

  /**
   * Get an index for the specified name. If the name is not already used
   * it is added to the name map.
   * @param  name  Name.
   * @return  Returns an index into the unique list of names.
   */
  uint32_t index(const std::string& name) {
    // Find the name in the map. If it is there return the index.
    auto it = names_.find(name);
    if (it != names_.end()) {
      return it->second;
    } else {
      // Not in the map, add index and update
      it = names_.insert(it, NamesMap::value_type(name, 0));
      indexes_.push_back(it);
      uint32_t index = indexes_.size() - 1;
      it->second = index;
      return index;
    }
  }

  /**
   * Get a name given an index. Returns an empty string if the index is out of range.
   * @param  index  Index into the unique name list.
   * @return  Returns the name
   */
  const std::string& name(const uint32_t index) const {
    return (index < (uint32_t)indexes_.size()) ? indexes_[index]->first : indexes_[0]->first;
  }

  /**
   * Clear the names and indexes.
   */
  void Clear() {
    names_.clear();
    indexes_.clear();
  }

  /**
   * Get the size - number of names. Since a blank name is added as the first unique name this
   * returns the size of the map - 1.
   * @return  Returns the number of unique names.
   */
  size_t Size() const {
    return names_.size() - 1;
  }

protected:
  // Map of names to indexes
  NamesMap names_;

  // List of entries into the map
  using nameiter = NamesMap::iterator;
  std::vector<nameiter> indexes_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_UNIQUENAMES_H
