#ifndef VALHALLA_ODIN_DIRECTIONSBUILDER_H_
#define VALHALLA_ODIN_DIRECTIONSBUILDER_H_

#include <list>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/odin/maneuver.h>

namespace valhalla {
namespace odin {

class DirectionsBuilder {
 public:
  DirectionsBuilder();

  TripDirections Build(TripPath& trip_path);

 protected:
  TripDirections PopulateTripDirections(TripPath& trip_path,
                                        std::list<Maneuver>& maneuvers);

};

}
}

#endif  // VALHALLA_ODIN_DIRECTIONSBUILDER_H_
