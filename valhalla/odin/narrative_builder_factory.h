#ifndef VALHALLA_ODIN_NARRATIVE_BUILDER_FACTORY_H_
#define VALHALLA_ODIN_NARRATIVE_BUILDER_FACTORY_H_

#include <memory>
#include <string>

#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/odin/narrativebuilder.h>
#include <valhalla/proto/options.pb.h>

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
   * @param  options  The directions options such as: distance units
   *                             and the language of the narration.
   * @param  trip_path  The nodes, edges, and attributes of the route path.
   * @return NarrativeBuilder unique pointer.
   */
  static std::unique_ptr<NarrativeBuilder> Create(const Options& options,
                                                  const EnhancedTripLeg* trip_path);
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_NARRATIVE_BUILDER_FACTORY_H_
