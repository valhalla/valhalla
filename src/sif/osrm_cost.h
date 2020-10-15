#ifndef VALHALLA_SIF_OSRM_COST_H_
#define VALHALLA_SIF_OSRM_COST_H_

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

namespace {

constexpr double kTurnPenalty = 7.5;
constexpr double kUTurnPenalty = 20;
constexpr double kTurnBias = 1.075;
constexpr double kTurnBiasInv = 1.0 / kTurnBias;
constexpr double kTrafficLightPenalty = 2;

inline uint32_t CalculateTurnDegree(const baldr::DirectedEdge* edge,
                                    const baldr::NodeInfo* node,
                                    const uint32_t idx_pred_opp) {
  auto in_heading = node->heading(idx_pred_opp);
  in_heading = ((in_heading + 180) % 360);
  auto out_heading = node->heading(edge->localedgeidx());
  uint32_t turn_degree = GetTurnDegree(in_heading, out_heading);
  return turn_degree;
}

} // namespace

float OSRMTurnCost(const baldr::DirectedEdge* edge,
                   const baldr::NodeInfo* node,
                   const uint32_t idx_pred_opp) {

  double turn_duration = node->traffic_signal() ? kTrafficLightPenalty : 0;
  uint32_t turn_degree = CalculateTurnDegree(edge, node, idx_pred_opp);

  // const auto is_uturn = guidance::getTurnDirection(turn_angle) ==
  // guidance::DirectionModifier::UTurn; BUT for OSRM utorn is 0 angle only, not some range
  const bool is_u_turn = Turn::GetType(turn_degree) == Turn::Type::kReverse;
  const uint32_t number_of_roads = node->local_edge_count();

  // if turn.number_of_roads > 2 or turn.source_mode ~= turn.target_mode or turn.is_u_turn then
  if (number_of_roads > 2 || is_u_turn) {
    int32_t osrm_angle = turn_degree > 180 ? static_cast<int32_t>(turn_degree) - 360 : turn_degree;
    double turn_bias = !node->drive_on_right() ? kTurnBiasInv : kTurnBias;
    double turn_bias_inv = !node->drive_on_right() ? kTurnBias : kTurnBiasInv;

    // if turn.angle >= 0 then
    //  turn.duration = turn.duration + turn_penalty / (1 + math.exp( -((13 / turn_bias) *
    //  turn.angle/180 - 6.5*turn_bias)))
    // else
    //  turn.duration = turn.duration + turn_penalty / (1 + math.exp( -((13 * turn_bias) *
    //  -turn.angle/180 - 6.5/turn_bias)))
    // end
    if (osrm_angle >= 0)
      turn_duration +=
          kTurnPenalty / (1 + std::exp(-((13 * turn_bias_inv) * osrm_angle / 180 - 6.5 * turn_bias)));
    else
      turn_duration += kTurnPenalty /
                       (1 + std::exp(-((13 * turn_bias) * -osrm_angle / 180 - 6.5 * turn_bias_inv)));

    turn_duration += is_u_turn ? kUTurnPenalty : 0;
  }
  return turn_duration;
}

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_OSRM_COST_H_
