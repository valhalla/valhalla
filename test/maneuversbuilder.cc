#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "midgard/logging.h"
#include "midgard/util.h"

#include "odin/maneuver.h"
#include "odin/maneuversbuilder.h"

#include "proto/options.pb.h"

#include "test.h"

using namespace std;
using namespace valhalla;
using namespace valhalla::odin;
using namespace valhalla::midgard;

namespace {

constexpr size_t TEXT = 0;
constexpr size_t IS_ROUTE_NUMBER = 1;
constexpr size_t CONSECUTIVE_COUNT = 2;

// Sub class to test protected methods
class ManeuversBuilderTest : public ManeuversBuilder {
public:
  ManeuversBuilderTest(const Options& options = Options()) : ManeuversBuilder(options, nullptr) {
  }

  ManeuversBuilderTest(const Options& options, EnhancedTripLeg* etp)
      : ManeuversBuilder(options, etp) {
  }

  void Combine(std::list<Maneuver>& maneuvers) {
    ManeuversBuilder::Combine(maneuvers);
  }

  void ProcessRoundabouts(std::list<Maneuver>& maneuvers) {
    ManeuversBuilder::ProcessRoundabouts(maneuvers);
  }

  void CountAndSortSigns(std::list<Maneuver>& maneuvers) {
    ManeuversBuilder::CountAndSortSigns(maneuvers);
  }

  void SetSimpleDirectionalManeuverType(Maneuver& maneuver) {
    ManeuversBuilder::SetSimpleDirectionalManeuverType(maneuver, nullptr, nullptr);
  }

  DirectionsLeg_Maneuver_CardinalDirection DetermineCardinalDirection(uint32_t heading) {
    return ManeuversBuilder::DetermineCardinalDirection(heading);
  }

  void DetermineRelativeDirection(Maneuver& maneuver) {
    return ManeuversBuilder::DetermineRelativeDirection(maneuver);
  }

  static Maneuver::RelativeDirection DetermineRelativeDirection(uint32_t turn_degree) {
    return ManeuversBuilder::DetermineRelativeDirection(turn_degree);
  }

  bool IsIntersectingForwardEdge(int node_index,
                                 EnhancedTripLeg_Edge* prev_edge,
                                 EnhancedTripLeg_Edge* curr_edge) {
    return ManeuversBuilder::IsIntersectingForwardEdge(node_index, prev_edge, curr_edge);
  }

  EnhancedTripLeg* trip_path() {
    return trip_path_;
  }
};

void TrySetSimpleDirectionalManeuverType(uint32_t turn_degree, DirectionsLeg_Maneuver_Type expected) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;

  // node:0
  node = path.add_node();

  // node:1
  node = path.add_node();
  node->mutable_edge()->set_drive_on_left(false);

  // node:2 dummy last node
  node = path.add_node();

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);
  Maneuver maneuver;
  maneuver.set_begin_node_index(1);
  maneuver.set_turn_degree(turn_degree);
  mbTest.SetSimpleDirectionalManeuverType(maneuver);
  EXPECT_EQ(maneuver.type(), expected)
      << "Incorrect maneuver type for turn degree=" + std::to_string(turn_degree);
}

TEST(Maneuversbuilder, TestSetSimpleDirectionalManeuverType) {
  // Continue lower bound
  TrySetSimpleDirectionalManeuverType(350, DirectionsLeg_Maneuver_Type_kContinue);
  // Continue middle
  TrySetSimpleDirectionalManeuverType(0, DirectionsLeg_Maneuver_Type_kContinue);
  // Continue upper bound
  TrySetSimpleDirectionalManeuverType(10, DirectionsLeg_Maneuver_Type_kContinue);

  // Slight right lower bound
  TrySetSimpleDirectionalManeuverType(11, DirectionsLeg_Maneuver_Type_kSlightRight);
  // Slight right middle
  TrySetSimpleDirectionalManeuverType(28, DirectionsLeg_Maneuver_Type_kSlightRight);
  // Slight right upper bound
  TrySetSimpleDirectionalManeuverType(44, DirectionsLeg_Maneuver_Type_kSlightRight);

  // Right lower bound
  TrySetSimpleDirectionalManeuverType(45, DirectionsLeg_Maneuver_Type_kRight);
  // Right middle
  TrySetSimpleDirectionalManeuverType(90, DirectionsLeg_Maneuver_Type_kRight);
  // Right upper bound
  TrySetSimpleDirectionalManeuverType(135, DirectionsLeg_Maneuver_Type_kRight);

  // Sharp right lower bound
  TrySetSimpleDirectionalManeuverType(136, DirectionsLeg_Maneuver_Type_kSharpRight);
  // Sharp right middle
  TrySetSimpleDirectionalManeuverType(148, DirectionsLeg_Maneuver_Type_kSharpRight);
  // Sharp right upper bound
  TrySetSimpleDirectionalManeuverType(159, DirectionsLeg_Maneuver_Type_kSharpRight);

  // Right side of street driving
  // Reverse lower bound
  TrySetSimpleDirectionalManeuverType(160, DirectionsLeg_Maneuver_Type_kUturnRight);
  // Reverse middle
  TrySetSimpleDirectionalManeuverType(179, DirectionsLeg_Maneuver_Type_kUturnRight);
  // Reverse middle
  TrySetSimpleDirectionalManeuverType(180, DirectionsLeg_Maneuver_Type_kUturnLeft);
  // Reverse upper bound
  TrySetSimpleDirectionalManeuverType(200, DirectionsLeg_Maneuver_Type_kUturnLeft);

  // Sharp left lower bound
  TrySetSimpleDirectionalManeuverType(201, DirectionsLeg_Maneuver_Type_kSharpLeft);
  // Sharp left middle
  TrySetSimpleDirectionalManeuverType(213, DirectionsLeg_Maneuver_Type_kSharpLeft);
  // Sharp left upper bound
  TrySetSimpleDirectionalManeuverType(224, DirectionsLeg_Maneuver_Type_kSharpLeft);

  // Left lower bound
  TrySetSimpleDirectionalManeuverType(225, DirectionsLeg_Maneuver_Type_kLeft);
  // Left middle
  TrySetSimpleDirectionalManeuverType(270, DirectionsLeg_Maneuver_Type_kLeft);
  // Left upper bound
  TrySetSimpleDirectionalManeuverType(315, DirectionsLeg_Maneuver_Type_kLeft);

  // Slight left lower bound
  TrySetSimpleDirectionalManeuverType(316, DirectionsLeg_Maneuver_Type_kSlightLeft);
  // Slight left middle
  TrySetSimpleDirectionalManeuverType(333, DirectionsLeg_Maneuver_Type_kSlightLeft);
  // Slight left upper bound
  TrySetSimpleDirectionalManeuverType(349, DirectionsLeg_Maneuver_Type_kSlightLeft);
}

void TryDetermineCardinalDirection(uint32_t heading,
                                   DirectionsLeg_Maneuver_CardinalDirection expected) {
  ManeuversBuilderTest mbTest;
  EXPECT_EQ(mbTest.DetermineCardinalDirection(heading), expected) << "Incorrect cardinal direction";
}

TEST(Maneuversbuilder, TestDetermineCardinalDirection) {
  // North lower bound
  TryDetermineCardinalDirection(337, DirectionsLeg_Maneuver_CardinalDirection_kNorth);
  // North middle
  TryDetermineCardinalDirection(0, DirectionsLeg_Maneuver_CardinalDirection_kNorth);
  // North upper bound
  TryDetermineCardinalDirection(23, DirectionsLeg_Maneuver_CardinalDirection_kNorth);

  // Northeast lower bound
  TryDetermineCardinalDirection(24, DirectionsLeg_Maneuver_CardinalDirection_kNorthEast);
  // Northeast middle
  TryDetermineCardinalDirection(45, DirectionsLeg_Maneuver_CardinalDirection_kNorthEast);
  // Northeast upper bound
  TryDetermineCardinalDirection(66, DirectionsLeg_Maneuver_CardinalDirection_kNorthEast);

  // East lower bound
  TryDetermineCardinalDirection(67, DirectionsLeg_Maneuver_CardinalDirection_kEast);
  // East middle
  TryDetermineCardinalDirection(90, DirectionsLeg_Maneuver_CardinalDirection_kEast);
  // East upper bound
  TryDetermineCardinalDirection(113, DirectionsLeg_Maneuver_CardinalDirection_kEast);

  // Southeast lower bound
  TryDetermineCardinalDirection(114, DirectionsLeg_Maneuver_CardinalDirection_kSouthEast);
  // Southeast middle
  TryDetermineCardinalDirection(135, DirectionsLeg_Maneuver_CardinalDirection_kSouthEast);
  // Southeast upper bound
  TryDetermineCardinalDirection(156, DirectionsLeg_Maneuver_CardinalDirection_kSouthEast);

  // South lower bound
  TryDetermineCardinalDirection(157, DirectionsLeg_Maneuver_CardinalDirection_kSouth);
  // South middle
  TryDetermineCardinalDirection(180, DirectionsLeg_Maneuver_CardinalDirection_kSouth);
  // South upper bound
  TryDetermineCardinalDirection(203, DirectionsLeg_Maneuver_CardinalDirection_kSouth);

  // Southwest lower bound
  TryDetermineCardinalDirection(204, DirectionsLeg_Maneuver_CardinalDirection_kSouthWest);
  // Southwest middle
  TryDetermineCardinalDirection(225, DirectionsLeg_Maneuver_CardinalDirection_kSouthWest);
  // Southwest upper bound
  TryDetermineCardinalDirection(246, DirectionsLeg_Maneuver_CardinalDirection_kSouthWest);

  // West lower bound
  TryDetermineCardinalDirection(247, DirectionsLeg_Maneuver_CardinalDirection_kWest);
  // West middle
  TryDetermineCardinalDirection(270, DirectionsLeg_Maneuver_CardinalDirection_kWest);
  // West upper bound
  TryDetermineCardinalDirection(293, DirectionsLeg_Maneuver_CardinalDirection_kWest);

  // Northwest lower bound
  TryDetermineCardinalDirection(294, DirectionsLeg_Maneuver_CardinalDirection_kNorthWest);
  // Northwest middle
  TryDetermineCardinalDirection(315, DirectionsLeg_Maneuver_CardinalDirection_kNorthWest);
  // Northwest upper bound
  TryDetermineCardinalDirection(336, DirectionsLeg_Maneuver_CardinalDirection_kNorthWest);
}

void TryDetermineRelativeDirection_Maneuver(uint32_t prev_heading,
                                            uint32_t curr_heading,
                                            const vector<uint32_t>& intersecting_headings,
                                            Maneuver::RelativeDirection expected) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;

  // node:0
  node = path.add_node();
  node->mutable_edge()->set_end_heading(prev_heading);

  // node:1
  node = path.add_node();
  node->mutable_edge()->set_begin_heading(curr_heading);
  for (auto intersecting_heading : intersecting_headings) {
    TripLeg_IntersectingEdge* xedge = node->add_intersecting_edge();
    xedge->set_begin_heading(intersecting_heading);
    xedge->set_driveability(TripLeg_Traversability_kBoth);
  }

  // node:2 dummy last node
  node = path.add_node();

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);
  Maneuver maneuver;
  maneuver.set_begin_node_index(1);
  maneuver.set_turn_degree(valhalla::midgard::GetTurnDegree(prev_heading, curr_heading));
  mbTest.DetermineRelativeDirection(maneuver);
  EXPECT_EQ(maneuver.begin_relative_direction(), expected)
      << std::string("Incorrect relative direction: ") +
             std::to_string(static_cast<int>(maneuver.begin_relative_direction())) +
             " | expected: " + std::to_string(static_cast<int>(expected));
}

TEST(Maneuversbuilder, TestDetermineRelativeDirection_Maneuver) {
  // Path straight, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(0, 5, {355}, Maneuver::RelativeDirection::kKeepRight);

  // Path straight, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(0, 355, {5}, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight right, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(0, 11, {0}, Maneuver::RelativeDirection::kKeepRight);

  // Path slight right, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(90, 105, {85}, Maneuver::RelativeDirection::kKeepRight);

  // Path slight left, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(0, 345, {355}, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight left, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(270, 255, {275}, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight left, intersecting right and left - thus keep straight
  TryDetermineRelativeDirection_Maneuver(80, 60, {157, 337},
                                         Maneuver::RelativeDirection::kKeepStraight);

  // Path sharp right, intersecting right and left - thus right
  TryDetermineRelativeDirection_Maneuver(180, 339, {355, 270, 180, 90, 10},
                                         Maneuver::RelativeDirection::kRight);

  // Path sharp left, intersecting right and left - thus left
  TryDetermineRelativeDirection_Maneuver(180, 21, {90, 180, 270, 352, 355, 5},
                                         Maneuver::RelativeDirection::kLeft);

  // Path reverse right, intersecting right and left - thus reverse
  TryDetermineRelativeDirection_Maneuver(180, 352, {355, 270, 180, 90, 10},
                                         Maneuver::RelativeDirection::KReverse);

  // Path reverse left, intersecting right and left - thus reverse
  TryDetermineRelativeDirection_Maneuver(180, 15, {355, 270, 180, 90, 10},
                                         Maneuver::RelativeDirection::KReverse);
}

void TryDetermineRelativeDirection(uint32_t turn_degree, Maneuver::RelativeDirection expected) {
  EXPECT_EQ(ManeuversBuilderTest::DetermineRelativeDirection(turn_degree), expected)
      << "Incorrect relative direction";
}

TEST(Maneuversbuilder, TestDetermineRelativeDirection) {
  // kKeepStraight lower bound
  TryDetermineRelativeDirection(330, Maneuver::RelativeDirection::kKeepStraight);
  // kKeepStraight middle
  TryDetermineRelativeDirection(0, Maneuver::RelativeDirection::kKeepStraight);
  // kKeepStraight upper bound
  TryDetermineRelativeDirection(30, Maneuver::RelativeDirection::kKeepStraight);

  // kRight lower bound
  TryDetermineRelativeDirection(31, Maneuver::RelativeDirection::kRight);
  // kRight middle
  TryDetermineRelativeDirection(90, Maneuver::RelativeDirection::kRight);
  // kRight upper bound
  TryDetermineRelativeDirection(159, Maneuver::RelativeDirection::kRight);

  // KReverse lower bound
  TryDetermineRelativeDirection(160, Maneuver::RelativeDirection::KReverse);
  // KReverse middle
  TryDetermineRelativeDirection(180, Maneuver::RelativeDirection::KReverse);
  // KReverse upper bound
  TryDetermineRelativeDirection(200, Maneuver::RelativeDirection::KReverse);

  // kLeft lower bound
  TryDetermineRelativeDirection(201, Maneuver::RelativeDirection::kLeft);
  // kLeft middle
  TryDetermineRelativeDirection(270, Maneuver::RelativeDirection::kLeft);
  // kLeft upper bound
  TryDetermineRelativeDirection(329, Maneuver::RelativeDirection::kLeft);
}

void TryCombine(ManeuversBuilderTest& mbTest,
                std::list<Maneuver>& maneuvers,
                std::list<Maneuver>& expected_maneuvers) {
  mbTest.Combine(maneuvers);

  EXPECT_EQ(maneuvers.size(), expected_maneuvers.size());

  for (auto man = maneuvers.begin(), expected_man = expected_maneuvers.begin();
       man != maneuvers.end(); ++man, ++expected_man) {
    EXPECT_EQ(man->type(), expected_man->type());
    EXPECT_NEAR(man->length(), expected_man->length(), .00001);
    EXPECT_EQ(man->time(), expected_man->time()) << "Incorrect maneuver time";
  }
}

// Deprecated
void PopulateEdge(TripLeg_Edge* edge,
                  const std::vector<std::pair<std::string, bool>>& names,
                  float length,
                  float speed,
                  valhalla::RoadClass road_class,
                  ::google::protobuf::uint32 begin_heading,
                  ::google::protobuf::uint32 end_heading,
                  ::google::protobuf::uint32 begin_shape_index,
                  ::google::protobuf::uint32 end_shape_index,
                  TripLeg_Traversability traversability,
                  bool ramp,
                  bool turn_channel,
                  bool ferry,
                  bool rail_ferry,
                  bool toll,
                  bool unpaved,
                  bool tunnel,
                  bool bridge,
                  bool roundabout,
                  bool internal_intersection,
                  const std::vector<std::pair<std::string, bool>>& exit_numbers,
                  const std::vector<std::pair<std::string, bool>>& exit_onto_streets,
                  const std::vector<std::pair<std::string, bool>>& exit_toward_locations,
                  const std::vector<std::pair<std::string, bool>>& exit_names,
                  TravelMode travel_mode = TravelMode::kDrive) {
  for (const auto& name : names) {
    auto* edge_name = edge->add_name();
    edge_name->set_value(name.first);
    edge_name->set_is_route_number(name.second);
  }
  edge->set_length_km(length);
  edge->set_speed(speed);
  edge->set_road_class(road_class);
  edge->set_begin_heading(begin_heading);
  edge->set_end_heading(end_heading);
  edge->set_begin_shape_index(begin_shape_index);
  edge->set_end_shape_index(end_shape_index);
  edge->set_traversability(traversability);
  if (ramp) {
    edge->set_use(TripLeg_Use::TripLeg_Use_kRampUse);
  } else if (turn_channel) {
    edge->set_use(TripLeg_Use::TripLeg_Use_kTurnChannelUse);
  } else if (ferry) {
    edge->set_use(TripLeg_Use::TripLeg_Use_kFerryUse);
  } else if (rail_ferry) {
    edge->set_use(TripLeg_Use::TripLeg_Use_kRailFerryUse);
  }
  edge->set_toll(toll);
  edge->set_unpaved(unpaved);
  edge->set_tunnel(tunnel);
  edge->set_bridge(bridge);
  edge->set_roundabout(roundabout);
  edge->set_internal_intersection(internal_intersection);
  valhalla::TripSign* sign = edge->mutable_sign();
  for (const auto& exit_number : exit_numbers) {
    auto* edge_exit_number = sign->add_exit_numbers();
    edge_exit_number->set_text(exit_number.first);
    edge_exit_number->set_is_route_number(exit_number.second);
  }
  for (const auto& exit_onto_street : exit_onto_streets) {
    auto* edge_exit_onto_street = sign->add_exit_onto_streets();
    edge_exit_onto_street->set_text(exit_onto_street.first);
    edge_exit_onto_street->set_is_route_number(exit_onto_street.second);
  }
  for (const auto& exit_toward_location : exit_toward_locations) {
    auto* edge_exit_toward_location = sign->add_exit_toward_locations();
    edge_exit_toward_location->set_text(exit_toward_location.first);
    edge_exit_toward_location->set_is_route_number(exit_toward_location.second);
  }
  for (const auto& exit_name : exit_names) {
    auto* edge_exit_name = sign->add_exit_names();
    edge_exit_name->set_text(exit_name.first);
    edge_exit_name->set_is_route_number(exit_name.second);
  }
  edge->set_travel_mode(travel_mode);
}

void PopulateIntersectingEdge(TripLeg_IntersectingEdge* xedge,
                              ::google::protobuf::uint32 begin_heading,
                              bool prev_name_consistency = false,
                              bool curr_name_consistency = false,
                              TripLeg_Traversability driveability = TripLeg_Traversability_kBoth,
                              TripLeg_Traversability cyclability = TripLeg_Traversability_kBoth,
                              TripLeg_Traversability walkability = TripLeg_Traversability_kBoth) {
  xedge->set_begin_heading(begin_heading);
  xedge->set_driveability(driveability);
  xedge->set_prev_name_consistency(prev_name_consistency);
  xedge->set_curr_name_consistency(curr_name_consistency);
}

void PopulateManeuver(Maneuver& maneuver,
                      DirectionsLeg_Maneuver_Type type,
                      const std::vector<std::pair<std::string, bool>>& street_names,
                      const std::vector<std::pair<std::string, bool>>& begin_street_names,
                      const std::vector<std::pair<std::string, bool>>& cross_street_names,
                      const std::string& instruction,
                      float distance,
                      uint32_t time,
                      uint32_t turn_degree,
                      Maneuver::RelativeDirection begin_relative_direction,
                      DirectionsLeg_Maneuver_CardinalDirection begin_cardinal_direction,
                      uint32_t begin_heading,
                      uint32_t end_heading,
                      uint32_t begin_node_index,
                      uint32_t end_node_index,
                      uint32_t begin_shape_index,
                      uint32_t end_shape_index,
                      bool ramp,
                      bool turn_channel,
                      bool ferry,
                      bool rail_ferry,
                      bool roundabout,
                      bool portions_toll,
                      bool portions_unpaved,
                      bool portions_highway,
                      bool internal_intersection,
                      const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_numbers,
                      const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_branches,
                      const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_towards,
                      const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_names,
                      uint32_t internal_right_turn_count = 0,
                      uint32_t internal_left_turn_count = 0,
                      uint32_t roundabout_exit_count = 0,
                      bool fork = false,
                      bool begin_intersecting_edge_name_consistency = false,
                      bool intersecting_forward_edge = false,
                      const std::string& verbal_succinct_transition_instruction = "",
                      const std::string& verbal_transition_alert_instruction = "",
                      const std::string& verbal_pre_transition_instruction = "",
                      const std::string& verbal_post_transition_instruction = "",
                      bool tee = false,
                      bool unnamed_walkway = false,
                      bool unnamed_cycleway = false,
                      bool unnamed_mountain_bike_trail = false,
                      float basic_time = 0.0f,
                      bool imminent_verbal_multi_cue = false) {

  maneuver.set_type(type);

  // street_names
  maneuver.set_street_names(street_names);

  // begin_street_names
  maneuver.set_begin_street_names(begin_street_names);

  // cross_street_names
  maneuver.set_cross_street_names(cross_street_names);

  maneuver.set_instruction(instruction);
  maneuver.set_length(distance);
  maneuver.set_time(time);
  maneuver.set_turn_degree(turn_degree);
  maneuver.set_begin_relative_direction(begin_relative_direction);
  maneuver.set_begin_cardinal_direction(begin_cardinal_direction);
  maneuver.set_begin_heading(begin_heading);
  maneuver.set_end_heading(end_heading);
  maneuver.set_begin_node_index(begin_node_index);
  maneuver.set_end_node_index(end_node_index);
  maneuver.set_begin_shape_index(begin_shape_index);
  maneuver.set_end_shape_index(end_shape_index);
  maneuver.set_ramp(ramp);
  maneuver.set_turn_channel(turn_channel);
  maneuver.set_ferry(ferry);
  maneuver.set_rail_ferry(rail_ferry);
  maneuver.set_roundabout(roundabout);
  maneuver.set_portions_toll(portions_toll);
  maneuver.set_portions_unpaved(portions_unpaved);
  maneuver.set_portions_highway(portions_highway);
  maneuver.set_internal_intersection(internal_intersection);

  // exit_numbers
  auto* exit_number_list = maneuver.mutable_signs()->mutable_exit_number_list();
  for (auto& sign_items : exit_numbers) {
    exit_number_list->emplace_back(std::get<TEXT>(sign_items), std::get<IS_ROUTE_NUMBER>(sign_items));
    exit_number_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(sign_items));
  }

  // exit_branches,
  auto* exit_branch_list = maneuver.mutable_signs()->mutable_exit_branch_list();
  for (auto& sign_items : exit_branches) {
    exit_branch_list->emplace_back(std::get<TEXT>(sign_items), std::get<IS_ROUTE_NUMBER>(sign_items));
    exit_branch_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(sign_items));
  }

  //  exit_towards,
  auto* exit_toward_list = maneuver.mutable_signs()->mutable_exit_toward_list();
  for (auto& sign_items : exit_towards) {
    exit_toward_list->emplace_back(std::get<TEXT>(sign_items), std::get<IS_ROUTE_NUMBER>(sign_items));
    exit_toward_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(sign_items));
  }

  //  exit_names
  auto* exit_name_list = maneuver.mutable_signs()->mutable_exit_name_list();
  for (auto& sign_items : exit_names) {
    exit_name_list->emplace_back(std::get<TEXT>(sign_items), std::get<IS_ROUTE_NUMBER>(sign_items));
    exit_name_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(sign_items));
  }

  maneuver.set_internal_right_turn_count(internal_right_turn_count);
  maneuver.set_internal_left_turn_count(internal_left_turn_count);
  maneuver.set_roundabout_exit_count(roundabout_exit_count);
  maneuver.set_fork(fork);
  maneuver.set_begin_intersecting_edge_name_consistency(begin_intersecting_edge_name_consistency);
  maneuver.set_intersecting_forward_edge(intersecting_forward_edge);
  maneuver.set_verbal_succinct_transition_instruction(verbal_succinct_transition_instruction);
  maneuver.set_verbal_transition_alert_instruction(verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(verbal_post_transition_instruction);
  maneuver.set_tee(tee);
  if (unnamed_walkway) {
    maneuver.set_trail_type(TrailType::kUnnamedWalkway);
  }
  if (unnamed_cycleway) {
    maneuver.set_trail_type(TrailType::kUnnamedCycleway);
  }
  if (unnamed_mountain_bike_trail) {
    maneuver.set_trail_type(TrailType::kUnnamedMtbTrail);
  }
  maneuver.set_basic_time(basic_time);
  maneuver.set_imminent_verbal_multi_cue(imminent_verbal_multi_cue);
}

TEST(Maneuversbuilder, TestLeftInternalStraightCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743", 1}, {"PA 341 Truck", 1}}, 0.033835, 60.000000,
               valhalla::RoadClass::kSecondary, 158, 180, 0, 3, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 South", 1}}, 0.181000, 60.000000,
               valhalla::RoadClass::kSecondary, 187, 192, 3, 8, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:2
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 South", 1}}, 0.079000, 60.000000,
               valhalla::RoadClass::kSecondary, 196, 196, 8, 10, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:3
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 South", 1}}, 0.160000, 60.000000,
               valhalla::RoadClass::kSecondary, 198, 198, 10, 13, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:4 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {}, 0.013000, 50.000000, valhalla::RoadClass::kSecondary, 118, 118, 13, 14,
               TripLeg_Traversability_kForward, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:5
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {}, 0.073000, 50.000000, valhalla::RoadClass::kSecondary, 127, 127, 14, 15,
               TripLeg_Traversability_kForward, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, {},
               {{"PA 283 East", 1}}, {{"Lancaster", 0}}, {});

  // node:6
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {}, 0.432000, 50.000000, valhalla::RoadClass::kSecondary, 127, 130, 15, 20,
               TripLeg_Traversability_kForward, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:7
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"PA 283 East", 1}}, 0.176467, 105.000000, valhalla::RoadClass::kMotorway, 134,
               134, 20, 22, TripLeg_Traversability_kForward, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
               {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Hershey Road", 0}, {"PA 743 South", 1}}, {}, {}, "", 0.453835, 28, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 158, 198, 0, 4, 0, 13, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {}, {}, {}, "", 0.013000, 1, 280,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 118, 118, 4, 5, 13, 14, 1, 0,
                   0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kRampStraight, {}, {}, {}, "", 0.505000, 36,
                   9, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 127, 130, 5, 7, 14, 20, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 283 East", 1, 0)},
                   {std::make_tuple("Lancaster", 0, 0)}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kMergeLeft, {{"PA 283 East", 1}}, {}, {},
                   "", 0.176467, 6, 4, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 134, 134, 7, 8, 20, 22, 0, 0,
                   0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver5 = maneuvers.back();
  PopulateManeuver(maneuver5, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 8, 8, 22, 22, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Hershey Road", 0}, {"PA 743 South", 1}}, {}, {}, "", 0.453835, 28, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 158, 198, 0, 4, 0, 13, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kRampLeft, {}, {}, {}, "",
                   0.518000, 37, 289, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 127, 130, 4, 7, 13, 20, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 283 East", 1, 0)},
                   {std::make_tuple("Lancaster", 0, 0)}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kMergeLeft, {{"PA 283 East", 1}},
                   {}, {}, "", 0.176467, 6, 4, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 134, 134, 7, 8, 20, 22, 0, 0,
                   0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver4 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 8, 8, 22, 22, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestStraightInternalLeftCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"PA 283 West", 1}}, 0.511447, 105.000000, valhalla::RoadClass::kMotorway, 315,
               316, 0, 3, TripLeg_Traversability_kForward, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
               {});

  // node:1
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {}, 0.397000, 50.000000, valhalla::RoadClass::kSecondary, 322, 330, 3, 12,
               TripLeg_Traversability_kForward, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {{"PA 743", 1}},
               {{"Hershey", 0}, {"Elizabethtown", 0}}, {});

  // node:2
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {}, 0.050000, 50.000000, valhalla::RoadClass::kSecondary, 308, 292, 12, 17,
               TripLeg_Traversability_kForward, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, {},
               {{"PA 743 South", 1}}, {{"Elizabethtown", 0}}, {});

  // node:3 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {}, 0.012000, 50.000000, valhalla::RoadClass::kSecondary, 289, 289, 17, 18,
               TripLeg_Traversability_kForward, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:4
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 South", 1}}, 0.160000, 60.000000,
               valhalla::RoadClass::kSecondary, 198, 198, 18, 21, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:5
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 South", 1}}, 0.084000, 60.000000,
               valhalla::RoadClass::kSecondary, 199, 198, 21, 23, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:6
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 South", 1}}, 0.113000, 60.000000,
               valhalla::RoadClass::kSecondary, 198, 198, 23, 24, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:7
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 South", 1}}, 0.129000, 60.000000,
               valhalla::RoadClass::kSecondary, 196, 196, 24, 25, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:8
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Hershey Road", 0}, {"PA 743 North", 1}}, 0.000000, 60.000000,
               valhalla::RoadClass::kSecondary, 22, 19, 25, 25, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"PA 283 West", 1}}, {}, {}, "",
                   0.511447, 18, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 315, 316, 0, 1, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 1, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kExitRight, {}, {}, {}, "", 0.397000, 29, 6,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 322, 330, 1, 2, 3, 12, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 743", 1, 0)},
                   {std::make_tuple("Hershey", 0, 0), std::make_tuple("Elizabethtown", 0, 0)}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kStayLeft, {}, {}, {}, "", 0.050000, 4, 338,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 308, 292, 2, 3, 12, 17, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 743 South", 1, 0)},
                   {std::make_tuple("Elizabethtown", 0, 0)}, {}, 0, 0, 0, 1);

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kNone, {}, {}, {}, "", 0.012000, 1, 357,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 289, 289, 3, 4, 17, 18, 1, 0, 0, 0,
                   0, 0, 0, 0, 1, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver5 = maneuvers.back();
  PopulateManeuver(maneuver5, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"Hershey Road", 0}, {"PA 743 South", 1}}, {}, {}, "", 0.486000, 30, 269,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 198, 19, 4, 9, 18, 25, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver6 = maneuvers.back();
  PopulateManeuver(maneuver6, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 9, 9, 25, 25, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"PA 283 West", 1}}, {},
                   {}, "", 0.511447, 18, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 315, 316, 0, 1, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 1, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kExitRight, {}, {}, {}, "",
                   0.397000, 29, 6, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 322, 330, 1, 2, 3, 12, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 743", 1, 0)},
                   {std::make_tuple("Hershey", 0, 0), std::make_tuple("Elizabethtown", 0, 0)}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kStayLeft, {}, {}, {}, "",
                   0.050000, 4, 338, Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 308, 292, 2, 3, 12, 17, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 743 South", 1, 0)},
                   {std::make_tuple("Elizabethtown", 0, 0)}, {}, 0, 0, 0, 1);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver4 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver4, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"Hershey Road", 0}, {"PA 743 South", 1}}, {}, {}, "", 0.498000, 31, 266,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 198, 19, 3, 9, 17, 25, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver5 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver5, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 9, 9, 25, 25, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestStraightInternalLeftInternalCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Broken Land Parkway", 0}}, 0.056148, 72.000000,
               valhalla::RoadClass::kSecondary, 26, 24, 0, 2, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Broken Land Parkway", 0}}, 0.081000, 72.000000,
               valhalla::RoadClass::kSecondary, 24, 24, 2, 3, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:2 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Broken Land Parkway", 0}}, 0.017000, 72.000000,
               valhalla::RoadClass::kSecondary, 25, 25, 3, 4, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:3 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Snowden River Parkway", 0}}, 0.030000, 60.000000,
               valhalla::RoadClass::kSecondary, 291, 291, 4, 5, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:4
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Patuxent Woods Drive", 0}}, 0.059840, 40.000000,
               valhalla::RoadClass::kTertiary, 292, 270, 5, 8, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"Broken Land Parkway", 0}}, {},
                   {}, "", 0.137148, 7, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 26, 24, 0, 2, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {}, {}, {}, "", 0.047000, 3, 1,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 25, 291, 2, 4, 3, 5, 0, 0, 0,
                   0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kContinue, {{"Patuxent Woods Drive", 0}},
                   {}, {}, "", 0.059840, 5, 1, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 292, 270, 4, 5, 5, 8, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 5, 5, 8, 8, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Broken Land Parkway", 0}}, {}, {}, "", 0.137148, 7, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 26, 24, 0, 2, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"Patuxent Woods Drive", 0}}, {}, {}, "", 0.106840, 8, 268,
                   Maneuver::RelativeDirection::kLeft, DirectionsLeg_Maneuver_CardinalDirection_kWest,
                   292, 270, 2, 5, 3, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 5, 5, 8, 8, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestStraightInternalStraightCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.120902, 80.000000,
               valhalla::RoadClass::kTrunk, 59, 94, 0, 5, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.086000, 80.000000,
               valhalla::RoadClass::kTrunk, 94, 94, 5, 8, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:2 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.018000, 90.000000,
               valhalla::RoadClass::kTrunk, 96, 96, 8, 9, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:3
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.099000, 80.000000,
               valhalla::RoadClass::kTrunk, 94, 95, 9, 12, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:4
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.774000, 80.000000,
               valhalla::RoadClass::kTrunk, 96, 88, 12, 28, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:5
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.123000, 80.000000,
               valhalla::RoadClass::kTrunk, 90, 90, 28, 32, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:6
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.009000, 80.000000,
               valhalla::RoadClass::kTrunk, 86, 86, 32, 33, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:7 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.015000, 72.000000,
               valhalla::RoadClass::kTrunk, 93, 93, 33, 34, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:8
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.077000, 72.000000,
               valhalla::RoadClass::kTrunk, 90, 90, 34, 35, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:9
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.217965, 72.000000,
               valhalla::RoadClass::kTrunk, 90, 89, 35, 40, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, {}, {}, "", 0.206902, 9, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 59, 94, 0, 2, 0, 8, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {}, {}, {}, "", 0.018000, 1, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 96, 96, 2, 3, 8, 9, 0, 0, 0, 0, 0,
                   0, 0, 0, 1, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kContinue,
                   {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, {}, {}, "", 1.005000, 45, 358,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 94, 86, 3, 7, 9, 33, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kNone, {}, {}, {}, "", 0.015000, 1, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 93, 93, 7, 8, 33, 34, 0, 0, 0, 0,
                   0, 0, 0, 0, 1, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver5 = maneuvers.back();
  PopulateManeuver(maneuver5, DirectionsLeg_Maneuver_Type_kContinue,
                   {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, {}, {}, "", 0.294965, 15, 357,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 90, 89, 8, 10, 34, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver6 = maneuvers.back();
  PopulateManeuver(maneuver6, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 10, 10, 40, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, {}, {}, "", 1.539867, 71, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 59, 10, 0, 10, 0, 40, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 10, 10, 40, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestLeftInternalUturnCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Jonestown Road", 0}, {"US 22", 1}}, 0.062923, 75.000000,
               valhalla::RoadClass::kPrimary, 36, 32, 0, 2, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1 TURN_CHANNEL
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Devonshire Road", 0}}, 0.013000, 50.000000, valhalla::RoadClass::kTertiary,
               299, 299, 2, 3, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, {}, {}, {},
               {});

  // node:2
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Jonestown Road", 0}, {"US 22", 1}}, 0.059697, 75.000000,
               valhalla::RoadClass::kPrimary, 212, 221, 3, 5, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Jonestown Road", 0}, {"US 22", 1}}, {}, {}, "", 0.062923, 3, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 36, 32, 0, 1, 0, 2, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {{"Devonshire Road", 0}}, {}, {}, "",
                   0.013000, 1, 267, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 299, 299, 1, 2, 2, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"Jonestown Road", 0}, {"US 22", 1}}, {}, {}, "", 0.059697, 3, 273,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 212, 221, 2, 3, 3, 5, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 3, 3, 5, 5, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Jonestown Road", 0}, {"US 22", 1}}, {}, {}, "", 0.062923, 3, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 36, 32, 0, 1, 0, 2, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kUturnLeft,
                   {{"Jonestown Road", 0}, {"US 22", 1}}, {}, {{"Devonshire Road", 0}}, "", 0.072697,
                   4, 180, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 212, 221, 1, 3, 2, 5, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 3, 3, 5, 5, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestLeftInternalUturnProperDirectionCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Pulaski Highway", 0}, {"US 40 East", 1}}, 0.067483, 75.000000,
               valhalla::RoadClass::kPrimary, 48, 52, 0, 3, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1 TURN_CHANNEL
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Moravia Park Drive", 0}}, 0.019000, 60.000000,
               valhalla::RoadClass::kSecondary, 317, 317, 3, 4, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:2
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"US 40 West", 1}, {"Pulaski Highway", 0}}, 0.045000, 90.000000,
               valhalla::RoadClass::kTrunk, 229, 229, 4, 5, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:3
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Pulaski Highway", 0}, {"US 40 West", 1}}, 0.000000, 75.000000,
               valhalla::RoadClass::kPrimary, 229, 229, 5, 5, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Pulaski Highway", 0}, {"US 40 East", 1}}, {}, {}, "", 0.067483, 3, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 48, 52, 0, 1, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {{"Moravia Park Drive", 0}}, {}, {},
                   "", 0.019000, 1, 265, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 317, 317, 1, 2, 3, 4, 0, 0, 0,
                   0, 0, 0, 0, 0, 1, {}, {}, {}, {}, 0, 1);

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"US 40 West", 1}, {"Pulaski Highway", 0}}, {}, {}, "", 0.045000, 2, 272,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 229, 229, 2, 4, 4, 5, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1);

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4, 5, 5, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Pulaski Highway", 0}, {"US 40 East", 1}}, {}, {}, "", 0.067483, 3, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 48, 52, 0, 1, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kUturnLeft,
                   {{"US 40 West", 1}, {"Pulaski Highway", 0}}, {}, {{"Moravia Park Drive", 0}}, "",
                   0.064000, 3, 177, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 229, 229, 1, 4, 3, 5, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4, 5, 5, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestStraightInternalLeftInternalStraightInternalUturnCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, 0.071404, 89.000000,
               valhalla::RoadClass::kTrunk, 335, 334, 0, 2, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1 TURN_CHANNEL
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, 0.012000, 89.000000,
               valhalla::RoadClass::kTrunk, 334, 334, 2, 3, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:2
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Bel Air South Parkway", 0}}, 0.025000, 48.000000,
               valhalla::RoadClass::kSecondary, 245, 245, 3, 4, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:3
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, 0.012000, 89.000000,
               valhalla::RoadClass::kTrunk, 153, 153, 4, 5, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:4
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, 0.070695, 89.000000,
               valhalla::RoadClass::kTrunk, 155, 156, 5, 9, TripLeg_Traversability_kBoth, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, {}, {}, "", 0.071404, 3,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 335, 334, 0, 1, 0, 2, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {{"Bel Air South Parkway", 0}}, {},
                   {}, "", 0.049000, 2, 0, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 334, 153, 1, 4, 2, 5, 0, 0, 0,
                   0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kContinue,
                   {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, {}, {}, "", 0.070695, 3,
                   2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 155, 156, 4, 5, 5, 9, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 5, 5, 9, 9, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, {}, {}, "", 0.071404, 3,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 335, 334, 0, 1, 0, 2, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kUturnLeft,
                   {{"MD 24", 1}, {"Vietnam Veterans Memorial Highway", 0}}, {},
                   {{"Bel Air South Parkway", 0}}, "", 0.119695, 5, 181,
                   Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 155, 156, 1, 5, 2, 9, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 5, 5, 9, 9, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestInternalPencilPointUturnProperDirectionCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Stonewall Shops Square", 0}}, 0.027386, 40.000000,
               valhalla::RoadClass::kUnclassified, 352, 343, 0, 2, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1 TURN_CHANNEL
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Old Carolina Road", 0}}, 0.019000, 50.000000, valhalla::RoadClass::kTertiary,
               331, 331, 2, 3, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, {}, {}, {},
               {});

  // node:2
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Stonewall Shops Square", 0}}, 0.021000, 50.000000,
               valhalla::RoadClass::kTertiary, 187, 187, 3, 4, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1, {}, {}, {}, {});

  // node:3
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Stonewall Shops Square", 0}}, 0.025240, 40.000000,
               valhalla::RoadClass::kUnclassified, 162, 149, 4, 6, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"Stonewall Shops Square", 0}}, {},
                   {}, "", 0.027386, 2, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 352, 343, 0, 1, 0, 2, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {{"Stonewall Shops Square", 0}}, {},
                   {}, "", 0.040000, 3, 348, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 331, 187, 1, 3, 2, 4, 0, 0, 0,
                   0, 0, 0, 0, 0, 1, {}, {}, {}, {}, 0, 1);

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kSlightLeft,
                   {{"Stonewall Shops Square", 0}}, {}, {}, "", 0.025240, 2, 335,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 162, 149, 3, 4, 4, 6, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4, 6, 6, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Stonewall Shops Square", 0}}, {}, {}, "", 0.027386, 2, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 352, 343, 0, 1, 0, 2, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kUturnLeft,
                   {{"Stonewall Shops Square", 0}}, {}, {}, "", 0.065240, 5, 179,
                   Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 162, 149, 1, 4, 2, 6, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4, 6, 6, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0);

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

TEST(Maneuversbuilder, TestSimpleRightTurnChannelCombine) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, 0.091237, 80.000000,
               valhalla::RoadClass::kTrunk, 59, 94, 0, 4, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:1 TURN_CHANNEL
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {}, 0.142000, 113.000000, valhalla::RoadClass::kSecondary, 105, 179, 4, 11,
               TripLeg_Traversability_kBoth, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:2
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Perry Hall Boulevard", 0}}, 0.065867, 64.000000,
               valhalla::RoadClass::kSecondary, 188, 188, 11, 14, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  // node:3 end node
  node = path.add_node();

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, {}, {}, "", 0.091237, 4, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 59, 94, 0, 1, 0, 4, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kNone, {}, {}, {}, "", 0.142000, 5, 11,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 105, 179, 1, 2, 4, 11, 0, 1, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kContinue, {{"Perry Hall Boulevard", 0}},
                   {}, {}, "", 0.065867, 4, 9, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 188, 188, 2, 3, 11, 14, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 3, 3, 14, 14, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"MD 43 East", 1}, {"White Marsh Boulevard", 0}}, {}, {}, "", 0.091237, 4, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 59, 94, 0, 1, 0, 4, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kRight,
                   {{"Perry Hall Boulevard", 0}}, {}, {}, "", 0.207867, 9, 94,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 188, 188, 1, 3, 4, 14, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {});

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 3, 3, 14, 14, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {});

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

void TryCountAndSortExitSigns(std::list<Maneuver>& maneuvers,
                              std::list<Maneuver>& expected_maneuvers) {
  ManeuversBuilderTest mbTest;
  mbTest.CountAndSortSigns(maneuvers);

  ASSERT_EQ(maneuvers.size(), expected_maneuvers.size()) << "Incorrect maneuver count";

  for (auto man = maneuvers.begin(), expected_man = expected_maneuvers.begin();
       man != maneuvers.end(); ++man, ++expected_man) {
    EXPECT_EQ(man->signs(), expected_man->signs()) << "Maneuver signs do not match expected";
  }
}

TEST(Maneuversbuilder, TestCountAndSortExitSigns) {

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"I 81 South", 1}, {"US 322 West", 1}, {"American Legion Memorial Highway", 0}},
                   {}, {}, "", 0.158406, 10, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 262, 270, 0, 1, 0, 2, 0, 0, 0, 0,
                   0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kExitRight, {{"US 322 West", 1}}, {}, {},
                   "", 0.348589, 21, 2, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 1, 2, 2, 6, 1, 0, 0, 0,
                   0, 0, 0, 0, 0, {std::make_tuple("67A-B", 0, 0)},
                   {std::make_tuple("US 22 East", 1, 0), std::make_tuple("PA 230 East", 1, 0),
                    std::make_tuple("US 22 West", 1, 0), std::make_tuple("US 322 West", 1, 0),
                    std::make_tuple("Cameron Street", 0, 0)},
                   {std::make_tuple("Harrisburg", 0, 0), std::make_tuple("Lewistown", 0, 0),
                    std::make_tuple("State College", 0, 0)},
                   {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kExitRight, {{"US 322 West", 1}}, {}, {},
                   "", 0.633177, 39, 8, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 286, 353, 2, 4, 6, 31, 1, 0, 0, 0,
                   0, 0, 0, 0, 0, {std::make_tuple("67B", 0, 0)},
                   {std::make_tuple("US 22 West", 1, 0), std::make_tuple("US 322 West", 1, 0)},
                   {std::make_tuple("Lewistown", 0, 0), std::make_tuple("State College", 0, 0)}, {},
                   0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kMergeLeft, {{"US 322 West", 1}}, {}, {},
                   "", 55.286610, 3319, 358, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 351, 348, 4, 57, 31, 1303, 0, 0,
                   0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver5 = maneuvers.back();
  PopulateManeuver(maneuver5, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.000000, 0,
                   0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 57, 57, 1303, 1303, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart,
                   {{"I 81 South", 1}, {"US 322 West", 1}, {"American Legion Memorial Highway", 0}},
                   {}, {}, "", 0.158406, 10, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 262, 270, 0, 1, 0, 2, 0, 0, 0, 0,
                   0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kExitRight, {{"US 322 West", 1}},
                   {}, {}, "", 0.348589, 21, 2, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 1, 2, 2, 6, 1, 0, 0, 0,
                   0, 0, 0, 0, 0, {std::make_tuple("67A-B", 0, 0)},
                   {std::make_tuple("US 322 West", 1, 2), std::make_tuple("US 22 West", 1, 1),
                    std::make_tuple("US 22 East", 1, 0), std::make_tuple("PA 230 East", 1, 0),
                    std::make_tuple("Cameron Street", 0, 0)},
                   {std::make_tuple("Lewistown", 0, 1), std::make_tuple("State College", 0, 1),
                    std::make_tuple("Harrisburg", 0, 0)},
                   {}, 0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kExitRight, {{"US 322 West", 1}},
                   {}, {}, "", 0.633177, 39, 8, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 286, 353, 2, 4, 6, 31, 1, 0, 0, 0,
                   0, 0, 0, 0, 0, {std::make_tuple("67B", 0, 0)},
                   {std::make_tuple("US 322 West", 1, 2), std::make_tuple("US 22 West", 1, 1)},
                   {std::make_tuple("Lewistown", 0, 1), std::make_tuple("State College", 0, 1)}, {},
                   0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver4 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver4, DirectionsLeg_Maneuver_Type_kMergeLeft, {{"US 322 West", 1}},
                   {}, {}, "", 55.286610, 3319, 358, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 351, 348, 4, 57, 31, 1303, 0, 0,
                   0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver5 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver5, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 57, 57, 1303, 1303, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  TryCountAndSortExitSigns(maneuvers, expected_maneuvers);
}

void TryIsIntersectingForwardEdge(ManeuversBuilderTest& mbTest, int node_index, bool expected) {
  auto prev_edge = mbTest.trip_path()->GetPrevEdge(node_index);
  auto curr_edge = mbTest.trip_path()->GetCurrEdge(node_index);

  bool intersecting_forward_link =
      mbTest.IsIntersectingForwardEdge(node_index, prev_edge.get(), curr_edge.get());

  EXPECT_EQ(intersecting_forward_link, expected);
}

TEST(Maneuversbuilder, TestPathRightXStraightIsIntersectingForwardEdge) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Raleigh Road", 0}}, 0.027827, 30.000000, valhalla::RoadClass::kResidential,
               250, 291, 0, 1, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
               {}, TravelMode::kDrive);

  // node:1 Intersecting forward link
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Raleigh Road", 0}}, 0.054344, 30.000000, valhalla::RoadClass::kResidential,
               20, 337, 1, 3, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
               {}, TravelMode::kDrive);
  PopulateIntersectingEdge(node->add_intersecting_edge(), 289, 1, 1, TripLeg_Traversability_kBoth,
                           TripLeg_Traversability_kBoth, TripLeg_Traversability_kBoth);

  // node:2
  node = path.add_node();

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  TryIsIntersectingForwardEdge(mbTest, 1, true);
}

TEST(Maneuversbuilder, TestPathLeftXStraightIsIntersectingForwardEdge) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Raleigh Road", 0}}, 0.047007, 30.000000, valhalla::RoadClass::kResidential,
               108, 108, 0, 1, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
               {}, TravelMode::kDrive);

  // node:1 Intersecting forward link
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Raleigh Road", 0}}, 0.046636, 30.000000, valhalla::RoadClass::kResidential,
               20, 337, 1, 3, TripLeg_Traversability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
               {}, TravelMode::kDrive);
  PopulateIntersectingEdge(node->add_intersecting_edge(), 111, 1, 1, TripLeg_Traversability_kBoth,
                           TripLeg_Traversability_kBoth, TripLeg_Traversability_kBoth);

  // node:2
  node = path.add_node();

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  TryIsIntersectingForwardEdge(mbTest, 1, true);
}

TEST(Maneuversbuilder, TestPathSlightRightXSlightLeftIsIntersectingForwardEdge) {
  Options options;
  TripLeg path;
  TripLeg_Node* node;
  TripLeg_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Horace Greeley Road", 0}}, 0.102593, 30.000000,
               valhalla::RoadClass::kResidential, 23, 13, 0, 6, TripLeg_Traversability_kBoth, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, TravelMode::kDrive);

  // node:1 Intersecting forward link
  node = path.add_node();
  edge = node->mutable_edge();
  PopulateEdge(edge, {{"Horace Greeley Road", 0}}, 0.205258, 30.000000,
               valhalla::RoadClass::kResidential, 45, 19, 6, 12, TripLeg_Traversability_kBoth, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, TravelMode::kDrive);
  PopulateIntersectingEdge(node->add_intersecting_edge(), 3, 0, 0, TripLeg_Traversability_kBoth,
                           TripLeg_Traversability_kBoth, TripLeg_Traversability_kBoth);

  // node:2
  node = path.add_node();

  EnhancedTripLeg etp(path);
  ManeuversBuilderTest mbTest(options, &etp);

  TryIsIntersectingForwardEdge(mbTest, 1, true);
}

void TryCombineRoundaboutManeuvers(std::list<Maneuver>& maneuvers,
                                   const std::list<Maneuver>& expected_maneuvers) {
  Options options;
  options.set_roundabout_exits(false);
  ManeuversBuilderTest mbTest(options);

  mbTest.ProcessRoundabouts(maneuvers);

  ASSERT_EQ(maneuvers.size(), expected_maneuvers.size());

  for (auto man = maneuvers.cbegin(), expected_man = expected_maneuvers.cbegin();
       man != maneuvers.end(); ++man, ++expected_man) {
    // Test specific properties that get combined for roundabouts when
    // roundabout_exit=false
    EXPECT_EQ(man->type(), expected_man->type());
    EXPECT_EQ(man->has_combined_enter_exit_roundabout(),
              expected_man->has_combined_enter_exit_roundabout());
    EXPECT_NEAR(man->roundabout_length(), expected_man->roundabout_length(), .00001);
    EXPECT_NEAR(man->roundabout_exit_length(), expected_man->roundabout_exit_length(), .00001);
    EXPECT_EQ(man->roundabout_exit_begin_heading(), expected_man->roundabout_exit_begin_heading());
    EXPECT_EQ(man->roundabout_exit_turn_degree(), expected_man->roundabout_exit_turn_degree());
    EXPECT_EQ(man->roundabout_exit_shape_index(), expected_man->roundabout_exit_shape_index());
  }
}

TEST(Maneuversbuilder, TestCombineRoundaboutManeuvers) {
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"first st", 0}}, {}, {}, "", 1.0,
                   1, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 100, 0, 0, 0, 5, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kRoundaboutEnter, {}, {}, {}, "", 1.0, 1,
                   32, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 150, 250, 0, 0, 5, 10, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kRoundaboutExit, {}, {}, {}, "", 2.0, 1, 90,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 280, 310, 0, 0, 10, 15, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.0, 1, 0,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 0, 0, 0, 15, 15, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"first st", 0}}, {}, {},
                   "", 1.0, 1, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 100, 0, 0, 0, 5, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kRoundaboutEnter, {}, {}, {}, "",
                   1.0, 2, 32, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 150, 310, 0, 0, 5, 15, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);
  // Manually update remaining maneuver attributes
  expected_maneuver2.set_has_combined_enter_exit_roundabout(true);
  expected_maneuver2.set_roundabout_exit_begin_heading(280);
  expected_maneuver2.set_roundabout_length(1.0);
  expected_maneuver2.set_roundabout_exit_length(2.0);
  expected_maneuver2.set_roundabout_exit_turn_degree(90);
  expected_maneuver2.set_roundabout_exit_shape_index(10);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.0,
                   1, 0, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  TryCombineRoundaboutManeuvers(maneuvers, expected_maneuvers);
}

void TryUnCollapsedRoundaboutManeuvers(std::list<Maneuver>& maneuvers,
                                       const std::list<Maneuver>& expected_maneuvers) {
  Options options;
  options.set_roundabout_exits(true);
  ManeuversBuilderTest mbTest(options);

  mbTest.ProcessRoundabouts(maneuvers);

  ASSERT_EQ(maneuvers.size(), expected_maneuvers.size());

  for (auto man = maneuvers.cbegin(), expected_man = expected_maneuvers.cbegin();
       man != maneuvers.end(); ++man, ++expected_man) {
    // Test specific properties that get combined for roundabouts when
    // roundabout_exit=false
    EXPECT_EQ(man->type(), expected_man->type());
    EXPECT_EQ(man->has_combined_enter_exit_roundabout(),
              expected_man->has_combined_enter_exit_roundabout());
    EXPECT_NEAR(man->roundabout_length(), expected_man->roundabout_length(), .00001);
    EXPECT_NEAR(man->roundabout_exit_length(), expected_man->roundabout_exit_length(), .00001);
    EXPECT_EQ(man->roundabout_exit_begin_heading(), expected_man->roundabout_exit_begin_heading());
    EXPECT_EQ(man->roundabout_exit_turn_degree(), expected_man->roundabout_exit_turn_degree());
  }
}

TEST(Maneuversbuilder, TestUnCollapseRoundaboutManeuvers) {
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"first st", 0}}, {}, {}, "", 1.0,
                   1, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, DirectionsLeg_Maneuver_Type_kRoundaboutEnter, {}, {}, {}, "", 1.0, 1,
                   32, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 150, 250, 0, 0, 0, 0, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, DirectionsLeg_Maneuver_Type_kRoundaboutExit, {}, {}, {}, "", 2.0, 1, 90,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 280, 310, 0, 0, 0, 0, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.0, 1, 0,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  ///////////////////////////////////////////////////////////////////////////
  // Create expected maneuver list with no collapsing
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, DirectionsLeg_Maneuver_Type_kStart, {{"first st", 0}}, {}, {},
                   "", 1.0, 1, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, DirectionsLeg_Maneuver_Type_kRoundaboutEnter, {}, {}, {}, "",
                   1.0, 2, 32, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 150, 310, 0, 0, 0, 0, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, DirectionsLeg_Maneuver_Type_kRoundaboutExit, {}, {}, {}, "",
                   2.0, 1, 90, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 280, 310, 0, 0, 0, 0, 0, 0, 0, 0,
                   1, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver4 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver4, DirectionsLeg_Maneuver_Type_kDestination, {}, {}, {}, "", 0.0,
                   1, 0, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, {}, {}, {}, {}, 0, 0, 0);

  TryUnCollapsedRoundaboutManeuvers(maneuvers, expected_maneuvers);
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
