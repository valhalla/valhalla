#ifndef VALHALLA_ODIN_DIRECTIONSBUILDER_H_
#define VALHALLA_ODIN_DIRECTIONSBUILDER_H_

#include <list>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/odin/maneuver.h>

namespace valhalla {
namespace odin {

class DirectionsBuilder {
 public:
  DirectionsBuilder();

  TripDirections Build(const DirectionsOptions& directions_options,
                       TripPath& trip_path);

 protected:
  TripDirections PopulateTripDirections(
      const DirectionsOptions& directions_options, TripPath& trip_path,
      std::list<Maneuver>& maneuvers);

};

}
}

#endif  // VALHALLA_ODIN_DIRECTIONSBUILDER_H_
