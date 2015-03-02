#ifndef VALHALLA_BALDR_TURN_H_
#define VALHALLA_BALDR_TURN_H_

#include <stdint.h>

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

  /**
   * Returns the turn type based on the specified turn degree.
   * For example, if 90 is supplied for the turn_degree,
   * then Turn::Type::kRight is returned.
   *
   * @param  turn_degree  the specified turn degree that is used to determine
   *                      the returned type.
   * @return the turn type based on the specified turn degree.
   */
  static Turn::Type GetType(uint32_t turn_degree);

 protected:
  Turn();
};

}
}

#endif  // VALHALLA_BALDR_TURN_H_
