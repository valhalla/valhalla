#ifndef VALHALLA_ODIN_DIRECTIONSBUILDER_H_
#define VALHALLA_ODIN_DIRECTIONSBUILDER_H_

#include <vector>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>

namespace valhalla{
namespace odin{

class DirectionsBuilder {
 public:
  DirectionsBuilder();

  TripDirections BuildSimple(TripPath& trip_path);

  TripDirections Build(TripPath& trip_path);

 protected:


};

}
}

#endif  // VALHALLA_ODIN_DIRECTIONSBUILDER_H_
