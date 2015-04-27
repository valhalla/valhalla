#ifndef VALHALLA_ODIN_MANEUVERSBUILDER_H_
#define VALHALLA_ODIN_MANEUVERSBUILDER_H_

#include <list>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/directions_options.pb.h>

#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/odin/maneuver.h>

namespace valhalla {
namespace odin {

class ManeuversBuilder {
 public:
  ManeuversBuilder(const DirectionsOptions& directions_options,
                   EnhancedTripPath* trip_path);

  std::list<Maneuver> Build();

 protected:
  std::list<Maneuver> Produce();

  void Combine(std::list<Maneuver>& maneuvers);

  std::list<Maneuver>::iterator CombineInternalManeuver(
      std::list<Maneuver>& maneuvers, std::list<Maneuver>::iterator prev_man,
      std::list<Maneuver>::iterator curr_man,
      std::list<Maneuver>::iterator next_man, bool start_man);

  std::list<Maneuver>::iterator CombineTurnChannelManeuver(
      std::list<Maneuver>& maneuvers, std::list<Maneuver>::iterator prev_man,
      std::list<Maneuver>::iterator curr_man,
      std::list<Maneuver>::iterator next_man, bool start_man);

  std::list<Maneuver>::iterator CombineSameNameStraightManeuver(
      std::list<Maneuver>& maneuvers, std::list<Maneuver>::iterator curr_man,
      std::list<Maneuver>::iterator next_man);

  void CountAndSortExitSigns(std::list<Maneuver>& maneuvers);

  void CreateDestinationManeuver(Maneuver& maneuver);

  void CreateStartManeuver(Maneuver& maneuver);

  void InitializeManeuver(Maneuver& maneuver, int node_index);

  void UpdateManeuver(Maneuver& maneuver, int node_index);

  void FinalizeManeuver(Maneuver& maneuver, int node_index);

  void SetManeuverType(Maneuver& maneuver);

  void SetSimpleDirectionalManeuverType(Maneuver& maneuver);

  TripDirections_Maneuver_CardinalDirection DetermineCardinalDirection(
      uint32_t heading);

  bool CanManeuverIncludePrevEdge(Maneuver& maneuver, int node_index);

  void DetermineRelativeDirection(Maneuver& maneuver);

  static Maneuver::RelativeDirection DetermineRelativeDirection(
      uint32_t turn_degree);

  bool IsRightSideOfStreetDriving() const;

  bool UsableInternalIntersectionName(Maneuver& maneuver, int node_index) const;

  void UpdateInternalTurnCount(Maneuver& maneuver, int node_index) const;

  const DirectionsOptions& directions_options_;
  EnhancedTripPath* trip_path_;

};

}
}

#endif  // VALHALLA_ODIN_MANEUVERSBUILDER_H_
