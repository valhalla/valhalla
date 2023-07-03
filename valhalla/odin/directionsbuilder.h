#ifndef VALHALLA_ODIN_DIRECTIONSBUILDER_H_
#define VALHALLA_ODIN_DIRECTIONSBUILDER_H_

#include <list>

#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/odin/maneuver.h>
#include <valhalla/odin/markup_formatter.h>
#include <valhalla/proto/api.pb.h>

namespace valhalla {
namespace odin {

/**
 * Builds the trip directions based on the specified directions options
 * and trip path.
 */
class DirectionsBuilder {
public:
  /**
   * Returns the trip directions based on the specified directions options
   * and trip path. This method calls ManeuversBuilder::Build and
   * NarrativeBuilder::Build to form the maneuver list. This method
   * calls PopulateDirectionsLeg to transform the maneuver list into the
   * trip directions.
   *
   * @param api   the protobuf object containing the request, the path and a place
   *              to store the resulting directions
   */
  static void Build(Api& api, const MarkupFormatter& markup_formatter);

protected:
  /**
   * Update the heading of ~0 length edges.
   *
   * @param etp The enhanced trip path contains the edges to process.
   */
  static void UpdateHeading(EnhancedTripLeg* etp);

  /**
   * Returns the trip directions based on the specified directions options,
   * trip path, and maneuver list.
   * @param options The directions options such as: units and
   *                           language.
   * @param etp The enhanced trip path - list of nodes, edges, attributes and shape.
   * @param maneuvers the maneuver list that contains the information required
   *                  to populate the trip directions.
   * @returns the trip directions.
   */
  static void PopulateDirectionsLeg(const Options& options,
                                    EnhancedTripLeg* etp,
                                    std::list<Maneuver>& maneuvers,
                                    DirectionsLeg& trip_directions);
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_DIRECTIONSBUILDER_H_
