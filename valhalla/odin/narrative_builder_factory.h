#ifndef VALHALLA_ODIN_NARRATIVE_BUILDER_FACTORY_H_
#define VALHALLA_ODIN_NARRATIVE_BUILDER_FACTORY_H_

#include <string>
#include <memory>

#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/odin/narrativebuilder.h>
#include <valhalla/odin/enhancedtrippath.h>

namespace valhalla {
namespace odin {

/**
 * A factory class that creates a specific NarrativeBuilder pointer
 * based on the specified language tag.
 */
class NarrativeBuilderFactory {
 public:
  NarrativeBuilderFactory() = delete;

  /**
   * Returns a specific NarrativeBuilder pointer based on the specified
   * language tag.
   *
   * @param  directions_options  The directions options such as: distance units
   *                             and the language of the narration.
   * @param  trip_path  The nodes, edges, and attributes of the route path.
   * @return NarrativeBuilder unique pointer.
   */
  static std::unique_ptr<NarrativeBuilder> Create(
      const DirectionsOptions& directions_options,
      const EnhancedTripPath* trip_path);

};

}
}

#endif  // VALHALLA_ODIN_NARRATIVE_BUILDER_FACTORY_H_
