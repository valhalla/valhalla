#ifndef VALHALLA_BALDR_TURN_H_
#define VALHALLA_BALDR_TURN_H_

#include <stdint.h>

#include <valhalla/midgard/util.h>

using namespace valhalla::midgard;

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
   * @return the turn type based on the specified turn degree.
   */
  static Turn::Type GetType(uint32_t turn_degree);

 protected:
  Turn();
};

}
}

#endif  // VALHALLA_BALDR_TURN_H_
