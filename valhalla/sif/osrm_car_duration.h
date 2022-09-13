#pragma once

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"

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

// we create a lookup tables since the range is well known and the computation is relatively expensive
std::array<double, 360> lookup_table(bool right) {
  std::array<double, 360> turn_durations;
  for (int angle = 0; angle < 360; ++angle) {
    // make the angle symmetric about 0
    int symmetric = angle > 180 ? static_cast<int32_t>(angle) - 360 : angle;
    // calculate a left turn, note the turnary cares what side of the road you drive on
    if (symmetric >= 0)
      turn_durations[angle] =
          kTurnPenalty / (1 + std::exp(-((13 * (right ? kTurnBiasInv : kTurnBias)) * symmetric / 180 -
                                         6.5 * (right ? kTurnBias : kTurnBiasInv))));
    // calculate a right turn, note the turnary cares what side of the road you drive on
    else
      turn_durations[angle] =
          kTurnPenalty /
          (1 + std::exp(-((13 * (right ? kTurnBias : kTurnBiasInv)) * -symmetric / 180 -
                          6.5 * (right ? kTurnBiasInv : kTurnBias))));
  }

  return turn_durations;
}

// This is a port of:
// https://github.com/Project-OSRM/osrm-backend/blob/f5ebe8bc3b51831b7b19e73d4879ebbad0161a19/profiles/car.lua#L455
inline float OSRMCarTurnDuration(const valhalla::baldr::DirectedEdge* edge,
                                 const valhalla::baldr::NodeInfo* node,
                                 const uint32_t idx_pred_opp) {

  // look up tables for turn penalties based on angle
  static const auto left_hand_lookup(lookup_table(false));
  static const auto right_hand_lookup(lookup_table(true));

  // start with the traffic light
  double turn_duration = node->traffic_signal() ? kTrafficLightPenalty : 0;

  // const auto is_uturn = guidance::getTurnDirection(turn_angle) ==
  // guidance::DirectionModifier::UTurn; BUT for OSRM uturn is 0 angle only, not a range
  uint32_t turn_degree = CalculateTurnDegree(edge, node, idx_pred_opp);
  const bool is_u_turn =
      valhalla::baldr::Turn::GetType(turn_degree) == valhalla::baldr::Turn::Type::kReverse;

  // if its not a "false node" or its a uturn
  const uint32_t number_of_roads = node->local_edge_count();
  if (number_of_roads > 2 || is_u_turn) {
    // get the duration due to the turn angle and whether it has the right of way
    turn_duration +=
        node->drive_on_right() ? right_hand_lookup[turn_degree] : left_hand_lookup[turn_degree];

    // add a penalty for uturns
    turn_duration += is_u_turn ? kUTurnPenalty : 0;
  }

  return turn_duration;
}

} // namespace
