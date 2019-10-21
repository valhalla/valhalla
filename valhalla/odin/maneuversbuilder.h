#ifndef VALHALLA_ODIN_MANEUVERSBUILDER_H_
#define VALHALLA_ODIN_MANEUVERSBUILDER_H_

#include <cstdint>
#include <list>

#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/odin/maneuver.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

namespace valhalla {
namespace odin {

/**
 * Builds the maneuver list based on the specified directions options and
 * enhanced trip path.
 */
class ManeuversBuilder {
public:
  /**
   * Constructor that assigns the specified directions options and trip path.
   *
   * @param options The directions options such as: units and
   *                           language.
   * @param trip_path The trip path - list of nodes, edges, attributes and shape.
   */
  ManeuversBuilder(const Options& options, EnhancedTripLeg* trip_path);

  std::list<Maneuver> Build();

protected:
  std::list<Maneuver> Produce();

  void Combine(std::list<Maneuver>& maneuvers);

  std::list<Maneuver>::iterator
  CollapseTransitConnectionStartManeuver(std::list<Maneuver>& maneuvers,
                                         std::list<Maneuver>::iterator curr_man,
                                         std::list<Maneuver>::iterator next_man);

  std::list<Maneuver>::iterator
  CollapseTransitConnectionDestinationManeuver(std::list<Maneuver>& maneuvers,
                                               std::list<Maneuver>::iterator curr_man,
                                               std::list<Maneuver>::iterator next_man);

  std::list<Maneuver>::iterator CombineInternalManeuver(std::list<Maneuver>& maneuvers,
                                                        std::list<Maneuver>::iterator prev_man,
                                                        std::list<Maneuver>::iterator curr_man,
                                                        std::list<Maneuver>::iterator next_man,
                                                        bool start_man);

  std::list<Maneuver>::iterator CombineTurnChannelManeuver(std::list<Maneuver>& maneuvers,
                                                           std::list<Maneuver>::iterator prev_man,
                                                           std::list<Maneuver>::iterator curr_man,
                                                           std::list<Maneuver>::iterator next_man,
                                                           bool start_man);

  std::list<Maneuver>::iterator CombineManeuvers(std::list<Maneuver>& maneuvers,
                                                 std::list<Maneuver>::iterator curr_man,
                                                 std::list<Maneuver>::iterator next_man);

  void CountAndSortExitSigns(std::list<Maneuver>& maneuvers);

  void ConfirmManeuverTypeAssignment(std::list<Maneuver>& maneuvers);

  void CreateDestinationManeuver(Maneuver& maneuver);

  void CreateStartManeuver(Maneuver& maneuver);

  void InitializeManeuver(Maneuver& maneuver, int node_index);

  void UpdateManeuver(Maneuver& maneuver, int node_index);

  void FinalizeManeuver(Maneuver& maneuver, int node_index);

  void SetManeuverType(Maneuver& maneuver, bool none_type_allowed = true);

  void SetSimpleDirectionalManeuverType(Maneuver& maneuver,
                                        EnhancedTripLeg_Edge* prev_edge,
                                        EnhancedTripLeg_Edge* curr_edge);

  DirectionsLeg_Maneuver_CardinalDirection DetermineCardinalDirection(uint32_t heading);

  bool CanManeuverIncludePrevEdge(Maneuver& maneuver, int node_index);

  bool IncludeUnnamedPrevEdge(int node_index,
                              EnhancedTripLeg_Edge* prev_edge,
                              EnhancedTripLeg_Edge* curr_edge) const;

  Maneuver::RelativeDirection
  DetermineMergeToRelativeDirection(EnhancedTripLeg_Node* node,
                                    EnhancedTripLeg_Edge* prev_edge) const;

  bool IsMergeManeuverType(Maneuver& maneuver,
                           EnhancedTripLeg_Edge* prev_edge,
                           EnhancedTripLeg_Edge* curr_edge) const;

  bool IsFork(int node_index, EnhancedTripLeg_Edge* prev_edge, EnhancedTripLeg_Edge* curr_edge) const;

  bool IsPedestrianFork(int node_index,
                        EnhancedTripLeg_Edge* prev_edge,
                        EnhancedTripLeg_Edge* curr_edge) const;

  bool IsTee(int node_index, EnhancedTripLeg_Edge* prev_edge, EnhancedTripLeg_Edge* curr_edge) const;

  bool IsLeftPencilPointUturn(int node_index,
                              EnhancedTripLeg_Edge* prev_edge,
                              EnhancedTripLeg_Edge* curr_edge) const;

  bool IsRightPencilPointUturn(int node_index,
                               EnhancedTripLeg_Edge* prev_edge,
                               EnhancedTripLeg_Edge* curr_edge) const;

  bool IsIntersectingForwardEdge(int node_index,
                                 EnhancedTripLeg_Edge* prev_edge,
                                 EnhancedTripLeg_Edge* curr_edge) const;

  void DetermineRelativeDirection(Maneuver& maneuver);

  static Maneuver::RelativeDirection DetermineRelativeDirection(uint32_t turn_degree);

  bool UsableInternalIntersectionName(Maneuver& maneuver, int node_index) const;

  void UpdateInternalTurnCount(Maneuver& maneuver, int node_index) const;

  /**
   * Returns the speed based on the specified travel mode.
   *
   * @param travel_mode The current specified travel mode.
   * @param edge_speed The speed of the current edge - used for driving mode.
   *
   * @return the speed based on the specified travel mode.
   */
  float GetSpeed(TripLeg_TravelMode travel_mode, float edge_speed) const;

  /**
   * Returns true if the current turn channel maneuver is able to be combined
   * with the next maneuver, false otherwise.
   *
   * @param curr_man Current maneuver
   * @param next_man Next maneuver
   *
   * @return true if the current turn channel maneuver is able to be combined
   * with the next maneuver, false otherwise.
   */
  bool IsTurnChannelManeuverCombinable(std::list<Maneuver>::iterator prev_man,
                                       std::list<Maneuver>::iterator curr_man,
                                       std::list<Maneuver>::iterator next_man,
                                       bool start_man) const;

  /**
   * Returns true if the current and next ramp maneuvers are able to be combined,
   * false otherwise.
   *
   * @param curr_man Current maneuver
   * @param next_man Next maneuver
   *
   * @return true if the current and next ramp maneuvers are able to be combined,
   * false otherwise.
   */
  bool AreRampManeuversCombinable(std::list<Maneuver>::iterator curr_man,
                                  std::list<Maneuver>::iterator next_man) const;

  /**
   * Returns true if roundabouts are processable based on the specified travel mode.
   *
   * @param travel_mode The current specified travel mode.
   *
   * @return true if roundabouts are processable based on the specified travel mode.
   */
  bool AreRoundaboutsProcessable(const TripLeg_TravelMode travel_mode) const;

  /**
   * Review each roundabout and if appropriate - set the roundabout name and roundabout exit name.
   * The roundabout name shall be a non-route number street name that does not exist on the incoming
   * and outgoing steps.
   *
   * @param maneuvers The list of maneuvers to process.
   */
  void ProcessRoundaboutNames(std::list<Maneuver>& maneuvers);

  /**
   * Iterate through the maneuvers and set the 'to stay on' attribute as needed.
   *
   * @param maneuvers The list of maneuvers to process.
   */
  void SetToStayOnAttribute(std::list<Maneuver>& maneuvers);

  /**
   * Enhance a signless interchange maneuver by adding the subsequent street name
   * as a branch name.
   *
   * @param maneuvers The list of maneuvers to process.
   */
  void EnhanceSignlessInterchnages(std::list<Maneuver>& maneuvers);

  /**
   * Returns the expected turn lane direction based on the specified maneuver and the available turn
   * lanes at the intersection.
   *
   * @param maneuver The maneuver at the intersection.
   */
  uint16_t GetExpectedTurnLaneDirection(Maneuver& maneuver) const;

  /**
   * Process the turn lanes at the maneuver point as well as within the maneuver.
   * Activate the turn lane that matches the path traversal.
   *
   * @param maneuvers The list of maneuvers to process.
   */
  void ProcessTurnLanes(std::list<Maneuver>& maneuvers);

  const Options& options_;
  EnhancedTripLeg* trip_path_;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_MANEUVERSBUILDER_H_
