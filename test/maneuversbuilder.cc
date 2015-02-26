#include "test.h"
#include "valhalla/odin/maneuver.h"
#include "valhalla/odin/maneuversbuilder.h"
#include <valhalla/midgard/util.h>

#include <string>

using namespace std;
using namespace valhalla::odin;

namespace {

// Sub class to test protected methods
class ManeuversBuilderTest : public ManeuversBuilder {
 public:
  ManeuversBuilderTest()
      : ManeuversBuilder(nullptr) {
  }

  ManeuversBuilderTest(EnhancedTripPath* etp)
      : ManeuversBuilder(etp) {
  }

  void Combine(std::list<Maneuver>& maneuvers) {
    ManeuversBuilder::Combine(maneuvers);
  }

  void SetSimpleDirectionalManeuverType(Maneuver& maneuver) {
    ManeuversBuilder::SetSimpleDirectionalManeuverType(maneuver);
  }

  TripDirections_Maneuver_CardinalDirection DetermineCardinalDirection(
      uint32_t heading) {
    return ManeuversBuilder::DetermineCardinalDirection(heading);
  }

  void DetermineRelativeDirection(Maneuver& maneuver) {
    return ManeuversBuilder::DetermineRelativeDirection(maneuver);
  }

  static Maneuver::RelativeDirection DetermineRelativeDirection(
      uint32_t turn_degree) {
    return ManeuversBuilder::DetermineRelativeDirection(turn_degree);
  }

};

void TrySetSimpleDirectionalManeuverType(
    uint32_t turn_degree, TripDirections_Maneuver_Type expected) {
  ManeuversBuilderTest mbTest;
  Maneuver maneuver;
  maneuver.set_turn_degree(turn_degree);
  mbTest.SetSimpleDirectionalManeuverType(maneuver);
  if (maneuver.type() != expected)
    throw std::runtime_error("Incorrect maneuver type");
}

void TestSetSimpleDirectionalManeuverType() {
  // Continue lower bound
  TrySetSimpleDirectionalManeuverType(350,
                                      TripDirections_Maneuver_Type_kContinue);
  // Continue middle
  TrySetSimpleDirectionalManeuverType(0,
                                      TripDirections_Maneuver_Type_kContinue);
  // Continue upper bound
  TrySetSimpleDirectionalManeuverType(10,
                                      TripDirections_Maneuver_Type_kContinue);

  // Slight right lower bound
  TrySetSimpleDirectionalManeuverType(
      11, TripDirections_Maneuver_Type_kSlightRight);
  // Slight right middle
  TrySetSimpleDirectionalManeuverType(
      28, TripDirections_Maneuver_Type_kSlightRight);
  // Slight right upper bound
  TrySetSimpleDirectionalManeuverType(
      44, TripDirections_Maneuver_Type_kSlightRight);

  // Right lower bound
  TrySetSimpleDirectionalManeuverType(45, TripDirections_Maneuver_Type_kRight);
  // Right middle
  TrySetSimpleDirectionalManeuverType(90, TripDirections_Maneuver_Type_kRight);
  // Right upper bound
  TrySetSimpleDirectionalManeuverType(135, TripDirections_Maneuver_Type_kRight);

  // Sharp right lower bound
  TrySetSimpleDirectionalManeuverType(136,
                                      TripDirections_Maneuver_Type_kSharpRight);
  // Sharp right middle
  TrySetSimpleDirectionalManeuverType(158,
                                      TripDirections_Maneuver_Type_kSharpRight);
  // Sharp right upper bound
  TrySetSimpleDirectionalManeuverType(180,
                                      TripDirections_Maneuver_Type_kSharpRight);

  // Sharp left lower bound
  TrySetSimpleDirectionalManeuverType(181,
                                      TripDirections_Maneuver_Type_kSharpLeft);
  // Sharp left middle
  TrySetSimpleDirectionalManeuverType(203,
                                      TripDirections_Maneuver_Type_kSharpLeft);
  // Sharp left upper bound
  TrySetSimpleDirectionalManeuverType(224,
                                      TripDirections_Maneuver_Type_kSharpLeft);

  // Left lower bound
  TrySetSimpleDirectionalManeuverType(225, TripDirections_Maneuver_Type_kLeft);
  // Left middle
  TrySetSimpleDirectionalManeuverType(270, TripDirections_Maneuver_Type_kLeft);
  // Left upper bound
  TrySetSimpleDirectionalManeuverType(315, TripDirections_Maneuver_Type_kLeft);

  // Slight left lower bound
  TrySetSimpleDirectionalManeuverType(316,
                                      TripDirections_Maneuver_Type_kSlightLeft);
  // Slight left middle
  TrySetSimpleDirectionalManeuverType(333,
                                      TripDirections_Maneuver_Type_kSlightLeft);
  // Slight left upper bound
  TrySetSimpleDirectionalManeuverType(349,
                                      TripDirections_Maneuver_Type_kSlightLeft);

}

void TryDetermineCardinalDirection(
    uint32_t heading, TripDirections_Maneuver_CardinalDirection expected) {
  ManeuversBuilderTest mbTest;
  if (mbTest.DetermineCardinalDirection(heading) != expected)
    throw std::runtime_error("Incorrect cardinal direction");
}

void TestDetermineCardinalDirection() {
  // North lower bound
  TryDetermineCardinalDirection(
      337, TripDirections_Maneuver_CardinalDirection_kNorth);
  // North middle
  TryDetermineCardinalDirection(
      0, TripDirections_Maneuver_CardinalDirection_kNorth);
  // North upper bound
  TryDetermineCardinalDirection(
      23, TripDirections_Maneuver_CardinalDirection_kNorth);

  // Northeast lower bound
  TryDetermineCardinalDirection(
      24, TripDirections_Maneuver_CardinalDirection_kNorthEast);
  // Northeast middle
  TryDetermineCardinalDirection(
      45, TripDirections_Maneuver_CardinalDirection_kNorthEast);
  // Northeast upper bound
  TryDetermineCardinalDirection(
      66, TripDirections_Maneuver_CardinalDirection_kNorthEast);

  // East lower bound
  TryDetermineCardinalDirection(
      67, TripDirections_Maneuver_CardinalDirection_kEast);
  // East middle
  TryDetermineCardinalDirection(
      90, TripDirections_Maneuver_CardinalDirection_kEast);
  // East upper bound
  TryDetermineCardinalDirection(
      113, TripDirections_Maneuver_CardinalDirection_kEast);

  // Southeast lower bound
  TryDetermineCardinalDirection(
      114, TripDirections_Maneuver_CardinalDirection_kSouthEast);
  // Southeast middle
  TryDetermineCardinalDirection(
      135, TripDirections_Maneuver_CardinalDirection_kSouthEast);
  // Southeast upper bound
  TryDetermineCardinalDirection(
      156, TripDirections_Maneuver_CardinalDirection_kSouthEast);

  // South lower bound
  TryDetermineCardinalDirection(
      157, TripDirections_Maneuver_CardinalDirection_kSouth);
  // South middle
  TryDetermineCardinalDirection(
      180, TripDirections_Maneuver_CardinalDirection_kSouth);
  // South upper bound
  TryDetermineCardinalDirection(
      203, TripDirections_Maneuver_CardinalDirection_kSouth);

  // Southwest lower bound
  TryDetermineCardinalDirection(
      204, TripDirections_Maneuver_CardinalDirection_kSouthWest);
  // Southwest middle
  TryDetermineCardinalDirection(
      225, TripDirections_Maneuver_CardinalDirection_kSouthWest);
  // Southwest upper bound
  TryDetermineCardinalDirection(
      246, TripDirections_Maneuver_CardinalDirection_kSouthWest);

  // West lower bound
  TryDetermineCardinalDirection(
      247, TripDirections_Maneuver_CardinalDirection_kWest);
  // West middle
  TryDetermineCardinalDirection(
      270, TripDirections_Maneuver_CardinalDirection_kWest);
  // West upper bound
  TryDetermineCardinalDirection(
      293, TripDirections_Maneuver_CardinalDirection_kWest);

  // Northwest lower bound
  TryDetermineCardinalDirection(
      294, TripDirections_Maneuver_CardinalDirection_kNorthWest);
  // Northwest middle
  TryDetermineCardinalDirection(
      315, TripDirections_Maneuver_CardinalDirection_kNorthWest);
  // Northwest upper bound
  TryDetermineCardinalDirection(
      336, TripDirections_Maneuver_CardinalDirection_kNorthWest);

}

void TryDetermineRelativeDirection_Maneuver(
    uint32_t prev_heading, uint32_t curr_heading,
    const vector<uint32_t>& intersecting_headings,
    Maneuver::RelativeDirection expected) {
  TripPath path;
  TripPath_Node* node;

  // node:0
  node = path.add_node();
  node->add_edge()->set_end_heading(prev_heading);

  // node:1
  node = path.add_node();
  node->add_edge()->set_begin_heading(curr_heading);
  for (auto intersecting_heading : intersecting_headings)
    node->add_edge()->set_begin_heading(intersecting_heading);

  // node:2 dummy last node
  node = path.add_node();

  ManeuversBuilderTest mbTest(static_cast<EnhancedTripPath*>(&path));
  Maneuver maneuver;
  maneuver.set_begin_node_index(1);
  maneuver.set_turn_degree(
      valhalla::midgard::GetTurnDegree(prev_heading, curr_heading));
  mbTest.DetermineRelativeDirection(maneuver);
  if (maneuver.begin_relative_direction() != expected)
    throw std::runtime_error("Incorrect relative direction");
}

void TestDetermineRelativeDirection_Maneuver() {
  // Path straight, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(
      0, 5, { 355 }, Maneuver::RelativeDirection::kKeepRight);

  // Path straight, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(
      0, 355, { 5 }, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight right, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(
      0, 11, { 0 }, Maneuver::RelativeDirection::kKeepRight);

  // Path slight right, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(
      90, 105, { 85 }, Maneuver::RelativeDirection::kKeepRight);

  // Path slight left, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(
      0, 345, { 355 }, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight left, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(
      270, 255, { 275 }, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight left, intersecting right and left - thus keep straight
  TryDetermineRelativeDirection_Maneuver(
      80, 60, { 157, 337 }, Maneuver::RelativeDirection::kKeepStraight);

  // Path sharp right, intersecting right and left - thus right
  TryDetermineRelativeDirection_Maneuver(180, 339, { 355, 270, 180, 90, 10 },
                                         Maneuver::RelativeDirection::kRight);

  // Path sharp left, intersecting right and left - thus left
  TryDetermineRelativeDirection_Maneuver(180, 21, { 90, 180, 270, 352, 355, 5 },
                                         Maneuver::RelativeDirection::kLeft);

  // Path reverse right, intersecting right and left - thus reverse
  TryDetermineRelativeDirection_Maneuver(180, 352, { 355, 270, 180, 90, 10 },
                                         Maneuver::RelativeDirection::KReverse);

  // Path reverse left, intersecting right and left - thus reverse
  TryDetermineRelativeDirection_Maneuver(180, 15, { 355, 270, 180, 90, 10 },
                                         Maneuver::RelativeDirection::KReverse);

}

void TryDetermineRelativeDirection(uint32_t turn_degree,
                                   Maneuver::RelativeDirection expected) {
  if (ManeuversBuilderTest::DetermineRelativeDirection(turn_degree) != expected)
    throw std::runtime_error("Incorrect relative direction");
}

void TestDetermineRelativeDirection() {
  // kKeepStraight lower bound
  TryDetermineRelativeDirection(330,
                                Maneuver::RelativeDirection::kKeepStraight);
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

void TryCombine(ManeuversBuilderTest& mbTest, std::list<Maneuver>& maneuvers,
                std::list<Maneuver>& expected_maneuvers) {
  mbTest.Combine(maneuvers);

  if (maneuvers.size() != expected_maneuvers.size())
    throw std::runtime_error("Incorrect maneuver count");
  for (auto man = maneuvers.begin(), expected_man = expected_maneuvers.begin();
      man != maneuvers.end(); ++man, ++expected_man) {
    if (man->type() != expected_man->type())
      throw std::runtime_error("Incorrect maneuver type");
    if (man->distance() != expected_man->distance())
      throw std::runtime_error("Incorrect maneuver distance");
    if (man->time() != expected_man->time())
      throw std::runtime_error("Incorrect maneuver time");
  }
}

void PopulateEdge(TripPath_Edge* edge, std::vector<std::string> names,
                  float length, float speed, TripPath_RoadClass road_class,
                  ::google::protobuf::uint32 begin_heading,
                  ::google::protobuf::uint32 end_heading,
                  ::google::protobuf::uint32 begin_shape_index,
                  ::google::protobuf::uint32 end_shape_index,
                  TripPath_Driveability driveability, bool ramp,
                  bool turn_channel, bool ferry, bool rail_ferry, bool toll,
                  bool unpaved, bool tunnel, bool bridge, bool roundabout,
                  bool internal_intersection,
                  ::google::protobuf::uint32 end_node_index,
                  std::vector<std::string> exit_numbers,
                  std::vector<std::string> exit_branches,
                  std::vector<std::string> exit_towards,
                  std::vector<std::string> exit_names) {
  for (auto& name : names) {
    edge->add_name(name);
  }
  edge->set_length(length);
  edge->set_speed(speed);
  edge->set_road_class(road_class);
  edge->set_begin_heading(begin_heading);
  edge->set_end_heading(end_heading);
  edge->set_begin_shape_index(begin_shape_index);
  edge->set_end_shape_index(end_shape_index);
  edge->set_driveability(driveability);
  edge->set_ramp(ramp);
  edge->set_turn_channel(turn_channel);
  edge->set_ferry(ferry);
  edge->set_rail_ferry(rail_ferry);
  edge->set_toll(toll);
  edge->set_unpaved(unpaved);
  edge->set_tunnel(tunnel);
  edge->set_bridge(bridge);
  edge->set_roundabout(roundabout);
  edge->set_internal_intersection(internal_intersection);
  edge->set_end_node_index(end_node_index);
  TripPath_Sign* sign = edge->mutable_sign();
  for (auto& exit_number : exit_numbers) {
    sign->add_exit_number(exit_number);
  }
  for (auto& exit_branch : exit_branches) {
    sign->add_exit_branch(exit_branch);
  }
  for (auto& exit_toward : exit_towards) {
    sign->add_exit_toward(exit_toward);
  }
  for (auto& exit_name : exit_names) {
    sign->add_exit_name(exit_name);
  }
}

void PopulateManeuver(
    Maneuver& maneuver, TripDirections_Maneuver_Type type,
    std::vector<std::string> street_names,
    std::vector<std::string> begin_street_names, float distance, uint32_t time,
    uint32_t turn_degree, Maneuver::RelativeDirection begin_relative_direction,
    TripDirections_Maneuver_CardinalDirection begin_cardinal_direction,
    uint32_t begin_heading, uint32_t end_heading, uint32_t begin_node_index,
    uint32_t end_node_index, uint32_t begin_shape_index,
    uint32_t end_shape_index, bool ramp, bool ferry, bool rail_ferry,
    bool roundabout, bool portions_toll, bool portions_unpaved,
    bool portions_highway, bool internal_intersection,
    std::vector<std::vector<std::string>> exit_numbers,
    std::vector<std::vector<std::string>> exit_branches,
    std::vector<std::vector<std::string>> exit_towards,
    std::vector<std::vector<std::string>> exit_names) {

  maneuver.set_type(type);

  // street_names
  StreetNames* street_name_list = maneuver.mutable_street_names();
  for (auto& street_name : street_names)
    street_name_list->emplace_back(street_name);

  // begin_street_names
  StreetNames* begin_street_name_list = maneuver.mutable_begin_street_names();
  for (auto& begin_street_name : begin_street_names)
    begin_street_name_list->emplace_back(begin_street_name);

  maneuver.set_distance(distance);
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
  maneuver.set_ferry(ferry);
  maneuver.set_rail_ferry(rail_ferry);
  maneuver.set_roundabout(roundabout);
  maneuver.set_portions_toll(portions_toll);
  maneuver.set_portions_unpaved(portions_unpaved);
  maneuver.set_portions_highway(portions_highway);
  maneuver.set_internal_intersection(internal_intersection);

  // exit_numbers
  std::vector<Sign>* exit_number_list = maneuver.mutable_signs()
      ->mutable_exit_number_list();
  for (auto& sign_items : exit_numbers) {
    exit_number_list->emplace_back(sign_items[0]);
    Sign& sign = exit_number_list->back();
    sign.set_consecutive_count(std::stoi(sign_items[1]));
  }

  // exit_branches,
  std::vector<Sign>* exit_branch_list = maneuver.mutable_signs()
      ->mutable_exit_branch_list();
  for (auto& sign_items : exit_branches) {
    exit_branch_list->emplace_back(sign_items[0]);
    Sign& sign = exit_branch_list->back();
    sign.set_consecutive_count(std::stoi(sign_items[1]));
  }

  //  exit_towards,
  std::vector<Sign>* exit_toward_list = maneuver.mutable_signs()
      ->mutable_exit_toward_list();
  for (auto& sign_items : exit_towards) {
    exit_toward_list->emplace_back(sign_items[0]);
    Sign& sign = exit_toward_list->back();
    sign.set_consecutive_count(std::stoi(sign_items[1]));
  }

  //  exit_names
  std::vector<Sign>* exit_name_list = maneuver.mutable_signs()
      ->mutable_exit_name_list();
  for (auto& sign_items : exit_names) {
    exit_name_list->emplace_back(sign_items[0]);
    Sign& sign = exit_name_list->back();
    sign.set_consecutive_count(std::stoi(sign_items[1]));
  }

}

void TestLeftInternalStraightCombine() {
  TripPath path;
  TripPath_Node* node;
  TripPath_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743", "PA 341 Truck" }, 0.033835,
               60.000000, TripPath_RoadClass_kSecondary, 158, 180, 0, 3,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:1
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 South" }, 0.181000, 60.000000,
               TripPath_RoadClass_kSecondary, 187, 192, 3, 8,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:2
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 South" }, 0.079000, 60.000000,
               TripPath_RoadClass_kSecondary, 196, 196, 8, 10,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:3
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 South" }, 0.160000, 60.000000,
               TripPath_RoadClass_kSecondary, 198, 198, 10, 13,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:4 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { }, 0.013000, 50.000000, TripPath_RoadClass_kSecondary,
               118, 118, 13, 14, TripPath_Driveability_kForward, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 1, 0, { }, { }, { }, { });

  // node:5
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { }, 0.073000, 50.000000, TripPath_RoadClass_kSecondary,
               127, 127, 14, 15, TripPath_Driveability_kForward, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, { }, { "PA 283 East" }, { "Lancaster" }, { });

  // node:6
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { }, 0.432000, 50.000000, TripPath_RoadClass_kSecondary,
               127, 130, 15, 20, TripPath_Driveability_kForward, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  // node:7
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "PA 283 East" }, 0.176467, 105.000000,
               TripPath_RoadClass_kMotorway, 134, 134, 20, 22,
               TripPath_Driveability_kForward, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  ManeuversBuilderTest mbTest(static_cast<EnhancedTripPath*>(&path));

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, TripDirections_Maneuver_Type_kStart, {
                       "Hershey Road", "PA 743 South" },
                   { }, 0.453835, 28, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 158, 198,
                   0, 4, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, TripDirections_Maneuver_Type_kNone, { }, { },
                   0.013000, 1, 280, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 118,
                   118, 4, 5, 13, 14, 1, 0, 0, 0, 0, 0, 0, 1, { }, { }, { },
                   { });

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, TripDirections_Maneuver_Type_kRampStraight, { },
                   { }, 0.505000, 36, 9,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 127,
                   130, 5, 7, 14, 20, 1, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "PA 283 East", "0" } },
                   { { "Lancaster", "0" } }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, TripDirections_Maneuver_Type_kMerge, {
                       "PA 283 East" },
                   { }, 0.176467, 6, 4,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 134,
                   134, 7, 8, 20, 22, 0, 0, 0, 0, 0, 0, 1, 0, { }, { }, { },
                   { });

  maneuvers.emplace_back();
  Maneuver& maneuver5 = maneuvers.back();
  PopulateManeuver(maneuver5, TripDirections_Maneuver_Type_kDestination, { },
                   { }, 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 8, 8,
                   22, 22, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, TripDirections_Maneuver_Type_kStart, {
                       "Hershey Road", "PA 743 South" },
                   { }, 0.453835, 28, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 158, 198,
                   0, 4, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, TripDirections_Maneuver_Type_kRampLeft,
                   { }, { }, 0.518000, 37, 289,
                   Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 127,
                   130, 4, 7, 13, 20, 1, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "PA 283 East", "0" } },
                   { { "Lancaster", "0" } }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, TripDirections_Maneuver_Type_kMerge, {
                       "PA 283 East" },
                   { }, 0.176467, 6, 4,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 134,
                   134, 7, 8, 20, 22, 0, 0, 0, 0, 0, 0, 1, 0, { }, { }, { },
                   { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver4 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver4,
                   TripDirections_Maneuver_Type_kDestination, { }, { },
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 8, 8,
                   22, 22, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

void TestStraightInternalLeftCombine() {
  TripPath path;
  TripPath_Node* node;
  TripPath_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "PA 283 West" }, 0.511447, 105.000000,
               TripPath_RoadClass_kMotorway, 315, 316, 0, 3,
               TripPath_Driveability_kForward, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:1
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { }, 0.397000, 50.000000, TripPath_RoadClass_kSecondary,
               322, 330, 3, 12, TripPath_Driveability_kForward, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, { }, { "PA 743" },
               { "Hershey", "Elizabethtown" }, { });

  // node:2
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { }, 0.050000, 50.000000, TripPath_RoadClass_kSecondary,
               308, 292, 12, 17, TripPath_Driveability_kForward, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, { }, { "PA 743 South" }, { "Elizabethtown" },
               { });

  // node:3 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { }, 0.012000, 50.000000, TripPath_RoadClass_kSecondary,
               289, 289, 17, 18, TripPath_Driveability_kForward, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 1, 0, { }, { }, { }, { });

  // node:4
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 South" }, 0.160000, 60.000000,
               TripPath_RoadClass_kSecondary, 198, 198, 18, 21,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:5
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 South" }, 0.084000, 60.000000,
               TripPath_RoadClass_kSecondary, 199, 198, 21, 23,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:6
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 South" }, 0.113000, 60.000000,
               TripPath_RoadClass_kSecondary, 198, 198, 23, 24,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:7
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 South" }, 0.129000, 60.000000,
               TripPath_RoadClass_kSecondary, 196, 196, 24, 25,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:8
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Hershey Road", "PA 743 North" }, 0.000000, 60.000000,
               TripPath_RoadClass_kSecondary, 22, 19, 25, 25,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  ManeuversBuilderTest mbTest(static_cast<EnhancedTripPath*>(&path));

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, TripDirections_Maneuver_Type_kStart, {
                       "PA 283 West" },
                   { }, 0.511447, 18, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 315,
                   316, 0, 1, 0, 3, 0, 0, 0, 0, 0, 0, 1, 0, { }, { }, { }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, TripDirections_Maneuver_Type_kExitRight, { }, { },
                   0.397000, 29, 6, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 322,
                   330, 1, 2, 3, 12, 1, 0, 0, 0, 0, 0, 0, 0, { }, { { "PA 743",
                       "0" } },
                   { { "Hershey", "0" }, { "Elizabethtown", "0" } }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, TripDirections_Maneuver_Type_kRampLeft, { }, { },
                   0.050000, 4, 338, Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 308,
                   292, 2, 3, 12, 17, 1, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "PA 743 South", "0" } },
                   { { "Elizabethtown", "0" } }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, TripDirections_Maneuver_Type_kNone, { }, { },
                   0.012000, 1, 357, Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 289, 289, 3,
                   4, 17, 18, 1, 0, 0, 0, 0, 0, 0, 1, { }, { }, { }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver5 = maneuvers.back();
  PopulateManeuver(maneuver5, TripDirections_Maneuver_Type_kLeft, {
                       "Hershey Road", "PA 743 South" },
                   { }, 0.486000, 30, 269, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 198, 19, 4,
                   9, 18, 25, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver6 = maneuvers.back();
  PopulateManeuver(maneuver6, TripDirections_Maneuver_Type_kDestination, { },
                   { }, 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 9, 9,
                   25, 25, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, TripDirections_Maneuver_Type_kStart, {
                       "PA 283 West" },
                   { }, 0.511447, 18, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 315,
                   316, 0, 1, 0, 3, 0, 0, 0, 0, 0, 0, 1, 0, { }, { }, { }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, TripDirections_Maneuver_Type_kExitRight,
                   { }, { }, 0.397000, 29, 6,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 322,
                   330, 1, 2, 3, 12, 1, 0, 0, 0, 0, 0, 0, 0, { }, { { "PA 743",
                       "0" } },
                   { { "Hershey", "0" }, { "Elizabethtown", "0" } }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3, TripDirections_Maneuver_Type_kRampLeft,
                   { }, { }, 0.050000, 4, 338,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 308,
                   292, 2, 3, 12, 17, 1, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "PA 743 South", "0" } },
                   { { "Elizabethtown", "0" } }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver4 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver4, TripDirections_Maneuver_Type_kLeft, {
                       "Hershey Road", "PA 743 South" },
                   { }, 0.498000, 31, 266, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 198, 19, 3,
                   9, 17, 25, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver5 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver5,
                   TripDirections_Maneuver_Type_kDestination, { }, { },
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 9, 9,
                   25, 25, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

void TestStraightInternalLeftInternalCombine() {
  TripPath path;
  TripPath_Node* node;
  TripPath_Edge* edge;

  ///////////////////////////////////////////////////////////////////////////
  // node:0
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Broken Land Parkway" }, 0.056148, 72.000000,
               TripPath_RoadClass_kSecondary, 26, 24, 0, 2,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:1
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Broken Land Parkway" }, 0.081000, 72.000000,
               TripPath_RoadClass_kSecondary, 24, 24, 2, 3,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  // node:2 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Broken Land Parkway" }, 0.017000, 72.000000,
               TripPath_RoadClass_kSecondary, 25, 25, 3, 4,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
               { }, { }, { }, { });

  // node:3 INTERNAL_INTERSECTION
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Snowden River Parkway" }, 0.030000, 60.000000,
               TripPath_RoadClass_kSecondary, 291, 291, 4, 5,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
               { }, { }, { }, { });

  // node:4
  node = path.add_node();
  edge = node->add_edge();
  PopulateEdge(edge, { "Patuxent Woods Drive" }, 0.059840, 40.000000,
               TripPath_RoadClass_kTertiaryUnclassified, 292, 270, 5, 8,
               TripPath_Driveability_kBoth, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               { }, { }, { }, { });

  ManeuversBuilderTest mbTest(static_cast<EnhancedTripPath*>(&path));

  ///////////////////////////////////////////////////////////////////////////
  // Create maneuver list
  std::list<Maneuver> maneuvers;
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, TripDirections_Maneuver_Type_kStart, {
                       "Broken Land Parkway" },
                   { }, 0.137148, 7, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 26, 24,
                   0, 2, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, TripDirections_Maneuver_Type_kNone, { }, { },
                   0.047000, 3, 1, Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 25,
                   291, 2, 4, 3, 5, 0, 0, 0, 0, 0, 0, 0, 1, { }, { }, { }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver3 = maneuvers.back();
  PopulateManeuver(maneuver3, TripDirections_Maneuver_Type_kContinue, {
                       "Patuxent Woods Drive" },
                   { }, 0.059840, 5, 1,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 292, 270, 4,
                   5, 5, 8, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  maneuvers.emplace_back();
  Maneuver& maneuver4 = maneuvers.back();
  PopulateManeuver(maneuver4, TripDirections_Maneuver_Type_kDestination, { },
                   { }, 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 5, 5,
                   8, 8, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  ///////////////////////////////////////////////////////////////////////////
  // Create expected combined maneuver list
  std::list<Maneuver> expected_maneuvers;

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver1 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver1, TripDirections_Maneuver_Type_kStart, {
                       "Broken Land Parkway" },
                   { }, 0.137148, 7, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 26, 24,
                   0, 2, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver2 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver2, TripDirections_Maneuver_Type_kLeft, {
                       "Patuxent Woods Drive" },
                   { }, 0.106840, 8, 268, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kWest, 292, 270, 2,
                   5, 3, 8, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  expected_maneuvers.emplace_back();
  Maneuver& expected_maneuver3 = expected_maneuvers.back();
  PopulateManeuver(expected_maneuver3,
                   TripDirections_Maneuver_Type_kDestination, { }, { },
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 5, 5,
                   8, 8, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { });

  TryCombine(mbTest, maneuvers, expected_maneuvers);
}

}

int main() {
  test::suite suite("maneuversbuilder");

  // SetSimpleDirectionalManeuverType
  suite.test(TEST_CASE(TestSetSimpleDirectionalManeuverType));

  // DetermineCardinalDirection
  suite.test(TEST_CASE(TestDetermineCardinalDirection));

  // DetermineRelativeDirection_Maneuver
  suite.test(TEST_CASE(TestDetermineRelativeDirection_Maneuver));

  // DetermineRelativeDirection
  suite.test(TEST_CASE(TestDetermineRelativeDirection));

  // LeftInternalStraightCombine
  suite.test(TEST_CASE(TestLeftInternalStraightCombine));

  // StraightInternalLeftCombine
  suite.test(TEST_CASE(TestStraightInternalLeftCombine));

  // StraightInternalLeftInternalCombine
  suite.test(TEST_CASE(TestStraightInternalLeftInternalCombine));

  return suite.tear_down();
}
