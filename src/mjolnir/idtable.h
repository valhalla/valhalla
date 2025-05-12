#ifndef VALHALLA_MJOLNIR_IDTABLE_H
#define VALHALLA_MJOLNIR_IDTABLE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>

#include <robin_hood.h>

#include <midgard/logging.h>

namespace valhalla {
namespace mjolnir {

class UnorderedIdTable final {
public:
  /**
   * Constructor
   * @param   size_hint   Hint about the total number of ids.
   */
  UnorderedIdTable(const uint64_t size_hint) {
    bitmarkers_.reserve((size_hint / 64) + 1);
  }

  /**
   * Sets the OSM Id as used.
   * @param   osmid   OSM Id of the way/node/relation.
   */
  inline void set(const uint64_t id) {
    uint64_t idx = id / 64;
    bitmarkers_[idx] |= static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64));
  }

  /**
   * Test if the OSM Id is used / set in the bitmarker.
   * @param  id  OSM Id
   * @return  Returns true if the OSM Id is used. False if not.
   */

  inline bool get(const uint64_t id) const {
    uint64_t idx = id / 64;
    auto found = bitmarkers_.find(idx);
    return found != bitmarkers_.cend() &&
           found->second & (static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64)));
  }

  /**
   * Serializes the table to file
   * @param file_name  the file to which we should serialize the table
   * @return true if the table could be serialized
   */
  bool serialize(const std::string& file_name) {
    std::ofstream file(file_name, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
      return false;
    }
    // TODO: do more than one entry at a time, buffer 10k of them into a vector and write that
    // Write key/value pairs
    for (const auto& i : bitmarkers_) {
      // key
      file.write(reinterpret_cast<const char*>(&i.first), sizeof(uint64_t));
      // value
      file.write(reinterpret_cast<const char*>(&i.second), sizeof(uint64_t));
    }
    file.close();
    return true;
  }

  /**
   * Deserializes the table from file
   * @param file_name  the file from which to deserialize the table
   * @return true if it was successfully deserialized
   */
  bool deserialize(const std::string& file_name) {
    std::ifstream file(file_name, std::ios::in | std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
      return false;
    }
    uint64_t entries = static_cast<uint64_t>(file.tellg()) / 2;
    file.seekg(0, std::ios::beg);

    // TODO: do more than one entry at a time, buffer 10k of them into a vector and read that
    // Read the count and then the via ids
    bitmarkers_.reserve(entries);
    uint64_t key, value;
    while (file && entries--) {
      file.read(reinterpret_cast<char*>(&key), sizeof(key));
      file.read(reinterpret_cast<char*>(&value), sizeof(value));
      bitmarkers_[key] = value;
    }
    file.close();
    return true;
  }

  /**
   * For unit tests only
   * @param other
   * @return
   */
  bool operator==(const UnorderedIdTable& other) const {
    return bitmarkers_ == other.bitmarkers_;
  }

private:
  robin_hood::unordered_map<uint64_t, uint64_t> bitmarkers_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_IDTABLE_H
