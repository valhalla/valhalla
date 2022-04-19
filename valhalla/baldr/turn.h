#ifndef VALHALLA_BALDR_TURN_H_
#define VALHALLA_BALDR_TURN_H_

#include <cstdint>
#include <stdint.h>
#include <string>

namespace valhalla {
namespace baldr {

/**
 * Defines the turn type based on turn degrees.
 */
class Turn {
public:
  enum class Type : uint8_t {
    kStraight = 0,
    kSlightRight = 1,
    kRight = 2,
    kSharpRight = 3,
    kReverse = 4,
    kSharpLeft = 5,
    kLeft = 6,
    kSlightLeft = 7
  };

  Turn() = delete;

  /**
   * Returns the turn type based on the specified turn degree.
   * For example, if 90 is supplied for the turn_degree,
   * then Turn::Type::kRight is returned.
   *
   * @param  turn_degree  the specified turn degree that is used to determine
   *                      the returned type. Expected range is 0 to 359.
   * @return the turn type based on the specified turn degree.
   */
  static Turn::Type GetType(uint32_t turn_degree);

  /**
   * Returns the turn type string.
   *
   * @param  turn_type  the specified turn type.
   * @return the turn type string based on the specified turn type.
   */
  static std::string GetTypeString(Turn::Type turn_type);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TURN_H_
