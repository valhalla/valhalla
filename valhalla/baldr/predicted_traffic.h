#ifndef VALHALLA_BALDR_PREDICTEDTRAFFIC_H_
#define VALHALLA_BALDR_PREDICTEDTRAFFIC_H_

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Structure to store the predicted traffic information for an edge.
 */
class PredictedTraffic {
 public:
  /**
   * Constructor with arguments.
   * @param  constrained_speed  Constrained speed
   * @param  compressed_offset  Offset to compressed data
   */
  PredictedTraffic(const float constrained_speed, const uint32_t compressed_offset);

  /**
   * Get the constrained speed
   * @return  Returns the constrained speed
   */
  float constrained_speed() const {
    return constrained_speed_;
  }

  /**
   * Set the constrained speed.
   * @param  constrained_speed  Constrained speed
   */
  void set_constrained_speed(const float constrained_speed);

  /**
   * Offset to the compressed traffic data. The offset is from the start
   * of the compressed traffic data within a tile.
   * @return  Returns offset from the start of the edge info within a tile.
   */
  uint64_t compressed_offset() const {
    return compressed_offset_;
  }

  /**
   * Set the offset to the compressed traffic data. The offset is from the start
   * of the compressed traffic data within a tile.
   * @param  compressed_offset  Offset from the start of the edge info within a tile.
   */
  void set_compressed_offset(const uint32_t compressed_offset);

 protected:
  uint32_t compressed_offset_   : 24; // Offset to compressed data.
  uint32_t constrained_speed_   : 8;  // Speed (kph)
};

}
}

#endif  // VALHALLA_BALDR_PREDICTEDTRAFFIC_H_
