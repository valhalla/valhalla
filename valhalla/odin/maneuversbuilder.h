#ifndef VALHALLA_ODIN_MANEUVERSBUILDER_H_
#define VALHALLA_ODIN_MANEUVERSBUILDER_H_

#include <list>

#include <valhalla/proto/trippath.pb.h>

#include "odin/enhancedtrippath.h"
#include "odin/maneuver.h"

namespace valhalla {
namespace odin {

class ManeuversBuilder {
 public:
  ManeuversBuilder(EnhancedTripPath* trip_path);

  std::list<Maneuver> Build();

 protected:
  std::list<Maneuver> Produce();

  void Combine(std::list<Maneuver>& maneuvers);

  void CreateDestinationManeuver(Maneuver& maneuver);

  void CreateStartManeuver(Maneuver& maneuver);

  void InitializeManeuver(Maneuver& maneuver, int nodeIndex);

  void UpdateManeuver(Maneuver& maneuver, int nodeIndex);

  void FinalizeManeuver(Maneuver& maneuver, int nodeIndex);

  void SetManeuverType(Maneuver& maneuver, int nodeIndex);

  bool CanManeuverIncludePrevEdge(Maneuver& maneuver, int nodeIndex);

  EnhancedTripPath* trip_path_;

};

}
}

#endif  // VALHALLA_ODIN_MANEUVERSBUILDER_H_
