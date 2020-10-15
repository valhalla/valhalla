#pragma once

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include <fastexp.h>

namespace {

constexpr double kTurnPenalty = 7.5;
constexpr double kUTurnPenalty = 20;
constexpr double kTurnBias = 1.075;
constexpr double kTurnBiasInv = 1.0 / kTurnBias;
constexpr double kTrafficLightPenalty = 2;

inline uint32_t CalculateTurnDegree(const valhalla::baldr::DirectedEdge* edge,
                                    const valhalla::baldr::NodeInfo* node,
                                    const uint32_t idx_pred_opp) {
  auto in_heading = node->heading(idx_pred_opp);
  in_heading = ((in_heading + 180) % 360);
  auto out_heading = node->heading(edge->localedgeidx());
  uint32_t turn_degree = valhalla::midgard::GetTurnDegree(in_heading, out_heading);
  return turn_degree;
}

// This is a port of:
// https://github.com/Project-OSRM/osrm-backend/blob/f5ebe8bc3b51831b7b19e73d4879ebbad0161a19/profiles/car.lua#L455
inline float OSRMTurnCost(const valhalla::baldr::DirectedEdge* edge,
                          const valhalla::baldr::NodeInfo* node,
                          const uint32_t idx_pred_opp) {

  double turn_duration = node->traffic_signal() ? kTrafficLightPenalty : 0;
  uint32_t turn_degree = CalculateTurnDegree(edge, node, idx_pred_opp);

  // const auto is_uturn = guidance::getTurnDirection(turn_angle) ==
  // guidance::DirectionModifier::UTurn; BUT for OSRM uturn is 0 angle only, not a range
  const bool is_u_turn =
      valhalla::baldr::Turn::GetType(turn_degree) == valhalla::baldr::Turn::Type::kReverse;
  const uint32_t number_of_roads = node->local_edge_count();

  if (number_of_roads > 2 || is_u_turn) {
    int32_t osrm_angle = turn_degree > 180 ? static_cast<int32_t>(turn_degree) - 360 : turn_degree;
    double turn_bias = !node->drive_on_right() ? kTurnBiasInv : kTurnBias;
    double turn_bias_inv = !node->drive_on_right() ? kTurnBias : kTurnBiasInv;

    if (osrm_angle >= 0)
      turn_duration += kTurnPenalty /
                       (1 + fasterexp(-((13 * turn_bias_inv) * osrm_angle / 180 - 6.5 * turn_bias)));
    else
      turn_duration += kTurnPenalty /
                       (1 + fasterexp(-((13 * turn_bias) * -osrm_angle / 180 - 6.5 * turn_bias_inv)));

    turn_duration += is_u_turn ? kUTurnPenalty : 0;
  }
  return turn_duration;
}

} // namespace
