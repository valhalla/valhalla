#ifndef VALHALLA_MJOLNIR_UNIQUENAMES_H
#define VALHALLA_MJOLNIR_UNIQUENAMES_H

#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>

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
  UniqueNames();

  /**
   * Get an index for the specified name. If the name is not already used
   * it is added to the name map.
   * @param  name  Name.
   * @return  Returns an index into the unique list of names.
   */
  uint32_t index(const std::string& name);

  /**
   * Get a name given an index.
   * @param  index  Index into the unique name list.
   * @return  Returns the name
   */
  const std::string& name(const uint32_t index) const;

  /**
   * Clear the names and indexes.
   */
  void Clear();

  /**
   * Get the size - number of names.
   * @return  Returns the number of unique names.
   */
  size_t Size() const;

  /**
   * Log information about the number of unique names, size of the vector, etc.
   */
  void Log() const;

 protected:
  // Map of names to indexes
  NamesMap names_;

  // List of entries into the map
  using nameiter = NamesMap::iterator;
  std::vector<nameiter> indexes_;
};

}
}

#endif  // VALHALLA_MJOLNIR_UNIQUENAMES_H
