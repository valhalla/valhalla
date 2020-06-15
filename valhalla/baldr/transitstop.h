#ifndef VALHALLA_BALDR_TRANSITSTOP_H_
#define VALHALLA_BALDR_TRANSITSTOP_H_

#include <cstdint>
#include <stdexcept>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Information held for each transit stop. This is information not required
 * during path generation. Such information is held within NodeInfo (lat,lng,
 * type, etc.).
 */
class TransitStop {
public:
  // Constructor with arguments
  TransitStop(const uint32_t one_stop_offset,
              const uint32_t name_offset,
              const bool generated,
              const uint32_t traversability)
      : generated_(generated), traversability_(traversability), spare_(0) {
    if (one_stop_offset > kMaxNameOffset) {
      throw std::runtime_error("TransitStop: Exceeded maximum name offset");
    }
    one_stop_offset_ = one_stop_offset;

    if (name_offset > kMaxNameOffset) {
      throw std::runtime_error("TransitStop: Exceeded maximum name offset");
    }
    name_offset_ = name_offset;
  }

  /**
   * Get the TransitLand one stop Id offset for the stop.
   * @return  Returns the TransitLand one stop Id offset.
   */
  uint32_t one_stop_offset() const {
    return one_stop_offset_;
  }

  /**
   * Get the text/name offset for the stop name.
   * @return  Returns the name offset in the text/name list.
   */
  uint32_t name_offset() const {
    return name_offset_;
  }

  /**
   * Get the generated flag that indicates if
   * the stop has been generated or exists in
   * real world
   * @return  Returns the generated flag.
   */
  bool generated() const {
    return generated_;
  }

  /**
   * Get the traversability indicates if
   * the egress can be entered, exited, or both
   * in the real world.
   * @return  Returns the traversability.
   */
  Traversability traversability() const {
    return static_cast<Traversability>(traversability_);
  }

protected:
  uint64_t one_stop_offset_ : 24; // TransitLand one stop Id offset.
  uint64_t name_offset_ : 24;     // Stop name offset in the text/name list.
  uint64_t generated_ : 1;
  uint64_t traversability_ : 2;
  uint64_t spare_ : 13;
  // size of tests
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TRANSITSTOP_H_
