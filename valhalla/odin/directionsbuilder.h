#ifndef VALHALLA_ODIN_DIRECTIONSBUILDER_H_
#define VALHALLA_ODIN_DIRECTIONSBUILDER_H_

#include <vector>

#include <valhalla/proto/trippath.pb.h>

namespace valhalla{
namespace odin{

class DirectionsBuilder {
 public:
  DirectionsBuilder();

  void BuildSimple(const TripPath& trip_path);

  void Build(const TripPath& trip_path);

 protected:


};

}
}

#endif  // VALHALLA_ODIN_DIRECTIONSBUILDER_H_
