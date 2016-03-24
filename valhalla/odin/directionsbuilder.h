#ifndef VALHALLA_ODIN_DIRECTIONSBUILDER_H_
#define VALHALLA_ODIN_DIRECTIONSBUILDER_H_

#include <list>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/odin/maneuver.h>
#include <valhalla/odin/enhancedtrippath.h>

namespace valhalla {
namespace odin {

/**
 * Builds the trip directions based on the specified directions options
 * and trip path.
 */
class DirectionsBuilder {
 public:
  DirectionsBuilder();

  /**
   * Returns the trip directions based on the specified directions options
   * and trip path. This method calls ManeuversBuilder::Build and
   * NarrativeBuilder::Build to form the maneuver list. This method
   * calls PopulateTripDirections to transform the maneuver list into the
   * trip directions.
   *
   * @param directions_options The directions options such as: units and
   *                           language.
   * @param trip_path The trip path - list of nodes, edges, attributes and shape.
   */
  TripDirections Build(const DirectionsOptions& directions_options,
                       TripPath& trip_path);

 protected:

  /**
   * Update the heading of ~0 length edges.
   *
   * @param etp The enhanced trip path contains the edges to process.
   */
  void UpdateHeading(EnhancedTripPath* etp);

  /**
   * Returns the trip directions based on the specified directions options,
   * trip path, and maneuver list.
   * @param directions_options The directions options such as: units and
   *                           language.
   * @param etp The enhanced trip path - list of nodes, edges, attributes and shape.
   * @param maneuvers the maneuver list that contains the information required
   *                  to populate the trip directions.
   * @returns the trip directions.
   */
  TripDirections PopulateTripDirections(
      const DirectionsOptions& directions_options, EnhancedTripPath* etp,
      std::list<Maneuver>& maneuvers);

  /**
   * Return the trip directions vehicle type for the specified trip path vehicle
   * type.
   *
   * @param vehicle_type trip path vehicle type.
   */
  TripDirections_VehicleType TranslateVehicleType(
      TripPath_VehicleType vehicle_type);

  /**
   * Return the trip directions pedestrian type for the specified trip path pedestrian
   * type.
   *
   * @param pedestrian_type trip path pedestrian type.
   */
  TripDirections_PedestrianType TranslatePedestrianType(
      TripPath_PedestrianType pedestrian_type);

  /**
   * Return the trip directions bicycle type for the specified trip path bicycle
   * type.
   *
   * @param bicycle_type trip path bicycle type.
   */
  TripDirections_BicycleType TranslateBicycleType(
      TripPath_BicycleType bicycle_type);

  /**
   * Return the trip directions transit type for the specified trip path transit
   * type.
   *
   * @param transit_type trip path transit type.
   */
  TripDirections_TransitType TranslateTransitType(
      TripPath_TransitType transit_type);

  /**
   * Return the trip directions travel mode for the specified trip path travel
   * mode.
   *
   * @param transit_type trip path transit type.
   */
  TripDirections_TravelMode TranslateTravelMode(
      TripPath_TravelMode travel_mode);

};

}
}

#endif  // VALHALLA_ODIN_DIRECTIONSBUILDER_H_

