#ifndef VALHALLA_ODIN_MANEUVERSBUILDER_H_
#define VALHALLA_ODIN_MANEUVERSBUILDER_H_

#include <vector>

#include <valhalla/proto/trippath.pb.h>

#include "odin/enhancedtrippath.h"

namespace valhalla{
namespace odin{

class ManeuversBuilder {
 public:
  ManeuversBuilder(EnhancedTripPath* trip_path);

  void Build();

 protected:
  EnhancedTripPath* trip_path_;

};

}
}

#endif  // VALHALLA_ODIN_MANEUVERSBUILDER_H_
