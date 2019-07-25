#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include <cstdint>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/mjolnir/osmway.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Derived class to build a directed edge given OSM way and other properties.
 */
class DirectedEdgeExtBuilder : public baldr::DirectedEdgeExt {
public:
  /**
   * Constructor with arguments.
   * @param  morning_speed         Morning speed in kph.
   * @param  general_speed         General speed in kph.
   * @param  evening_speed         Evening speed in kph.
   */
  DirectedEdgeExtBuilder(const uint32_t morning_speed,
                         const uint32_t general_speed,
                         const uint32_t evening_speed);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
