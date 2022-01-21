#include <cstdint>
#include <regex>

#include "baldr/verbal_text_formatter_factory.h"

#include "odin/enhancedtrippath.h"
#include "odin/maneuver.h"
#include "odin/markup_formatter.h"
#include "odin/narrative_builder_factory.h"
#include "odin/narrative_dictionary.h"
#include "odin/narrativebuilder.h"
#include "odin/sign.h"
#include "odin/signs.h"
#include "odin/util.h"

#include "proto/common.pb.h"
#include "proto/directions.pb.h"
#include "proto/trip.pb.h"

#include "test.h"

using namespace std;
using namespace valhalla;
using namespace valhalla::odin;

namespace {

constexpr size_t TEXT = 0;
constexpr size_t IS_ROUTE_NUMBER = 1;
constexpr size_t CONSECUTIVE_COUNT = 2;

// Sub class to test protected methods
class NarrativeBuilderTest : public NarrativeBuilder {
public:
  NarrativeBuilderTest(const Options& options,
                       const NarrativeDictionary& dictionary,
                       const EnhancedTripLeg* trip_path = nullptr)
      : NarrativeBuilder(options, trip_path, dictionary, MarkupFormatter()) {
  }

  std::string FormRampStraightInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormRampStraightInstruction(maneuver);
  }

  std::string FormRampInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormRampInstruction(maneuver);
  }

  std::string FormExitInstruction(Maneuver& maneuver) {
    return NarrativeBuilder::FormExitInstruction(maneuver);
  }

  std::string
  FormVerbalPostTransitionInstruction(Maneuver& maneuver,
                                      bool include_street_names = false,
                                      uint32_t element_max_count = kVerbalPostElementMaxCount,
                                      const std::string& delim = kVerbalDelim) {
    return NarrativeBuilder::FormVerbalPostTransitionInstruction(maneuver, include_street_names,
                                                                 element_max_count, delim);
  }

  std::string FormVerbalMultiCue(Maneuver& maneuver, Maneuver& next_maneuver) {
    return NarrativeBuilder::FormVerbalMultiCue(maneuver, next_maneuver);
  }
};

const NarrativeDictionary& GetNarrativeDictionary(const Options& options) {
  // Get the locale dictionary
  const auto phrase_dictionary = get_locales().find(options.language());

  // If language tag is not found then throw error
  if (phrase_dictionary == get_locales().end()) {
    throw std::runtime_error("Invalid language tag.");
  }

  return *phrase_dictionary->second;
}

void PopulateManeuver(Maneuver& maneuver,
                      const std::string& country_code,
                      const std::string& state_code,
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
                      bool imminent_verbal_multi_cue = false,
                      bool drive_on_right = true,
                      bool has_long_street_name = false) {

  maneuver.set_verbal_formatter(VerbalTextFormatterFactory::Create(country_code, state_code));

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
  maneuver.set_drive_on_right(drive_on_right);
  maneuver.set_long_street_name(has_long_street_name);

  // exit_numbers
  auto* exit_number_list = maneuver.mutable_signs()->mutable_exit_number_list();
  for (auto& sign_items : exit_numbers) {
    exit_number_list->emplace_back(std::get<TEXT>(sign_items), std::get<IS_ROUTE_NUMBER>(sign_items));
    exit_number_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(sign_items));
  }

  // exit_branches
  auto* exit_branch_list = maneuver.mutable_signs()->mutable_exit_branch_list();
  for (auto& sign_items : exit_branches) {
    exit_branch_list->emplace_back(std::get<TEXT>(sign_items), std::get<IS_ROUTE_NUMBER>(sign_items));
    exit_branch_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(sign_items));
  }

  //  exit_towards
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

void PopulateTransitInfo(valhalla::odin::TransitRouteInfo* transit_info,
                         const std::string& onestop_id,
                         uint32_t block_id,
                         uint32_t trip_id,
                         const std::string& short_name,
                         const std::string& long_name,
                         const std::string& headsign,
                         uint32_t color,
                         uint32_t text_color,
                         const std::string& description,
                         const std::string& operator_onestop_id,
                         const std::string& operator_name,
                         const std::string& operator_url) {
  transit_info->onestop_id = onestop_id;
  transit_info->block_id = block_id;
  transit_info->trip_id = trip_id;
  transit_info->short_name = short_name;
  transit_info->long_name = long_name;
  transit_info->headsign = headsign;
  transit_info->color = color;
  transit_info->text_color = text_color;
  transit_info->description = description;
  transit_info->operator_onestop_id = operator_onestop_id;
  transit_info->operator_name = operator_name;
  transit_info->operator_url = operator_url;
}

// TOOD - remove is_parent_stop
// TODO - add station_onestop_id and station_name
TransitPlatformInfo GetTransitPlatformInfo(TransitPlatformInfo_Type type,
                                           const std::string& onestop_id,
                                           const std::string& name,
                                           const std::string& arrival_date_time,
                                           const std::string& departure_date_time,
                                           bool assumed_schedule,
                                           float lat,
                                           float lng) {
  TransitPlatformInfo transit_platform_info;

  transit_platform_info.set_type(type);
  if (!onestop_id.empty())
    transit_platform_info.set_onestop_id(onestop_id);
  if (!name.empty())
    transit_platform_info.set_name(name);
  if (!arrival_date_time.empty())
    transit_platform_info.set_arrival_date_time(arrival_date_time);
  if (!departure_date_time.empty())
    transit_platform_info.set_departure_date_time(departure_date_time);
  if (assumed_schedule)
    transit_platform_info.set_assumed_schedule(assumed_schedule);
  transit_platform_info.mutable_ll()->set_lat(lat);
  transit_platform_info.mutable_ll()->set_lng(lng);
  // TODO - add station_onestop_id and station_name as parameters
  transit_platform_info.set_station_onestop_id("TBD");
  transit_platform_info.set_station_name(name);

  return transit_platform_info;
}

void TryBuild(const Options& options,
              std::list<Maneuver>& maneuvers,
              std::list<Maneuver>& expected_maneuvers,
              const EnhancedTripLeg* etp = nullptr) {
  std::unique_ptr<NarrativeBuilder> narrative_builder =
      NarrativeBuilderFactory::Create(options, etp, MarkupFormatter());
  narrative_builder->Build(maneuvers);

  // Check maneuver list sizes
  ASSERT_EQ(maneuvers.size(), expected_maneuvers.size());

  for (auto man = maneuvers.begin(), expected_man = expected_maneuvers.begin();
       man != maneuvers.end(); ++man, ++expected_man) {

    // Check maneuver type
    EXPECT_EQ(man->type(), expected_man->type());

    // Check maneuver instruction
    EXPECT_EQ(man->instruction(), expected_man->instruction());

    // Check maneuver verbal_succinct_transition_instruction
    EXPECT_EQ(man->verbal_succinct_transition_instruction(),
              expected_man->verbal_succinct_transition_instruction());

    // Check maneuver verbal_transition_alert_instruction
    EXPECT_EQ(man->verbal_transition_alert_instruction(),
              expected_man->verbal_transition_alert_instruction());

    // Check maneuver verbal_pre_transition_instruction
    EXPECT_EQ(man->verbal_pre_transition_instruction(),
              expected_man->verbal_pre_transition_instruction());

    // Check maneuver verbal_post_transition_instruction
    EXPECT_EQ(man->verbal_post_transition_instruction(),
              expected_man->verbal_post_transition_instruction());

    // Check maneuver depart_instruction
    EXPECT_EQ(man->depart_instruction(), expected_man->depart_instruction());

    // Check maneuver verbal_depart_instruction
    EXPECT_EQ(man->verbal_depart_instruction(), expected_man->verbal_depart_instruction());

    // Check maneuver arrive_instruction
    EXPECT_EQ(man->arrive_instruction(), expected_man->arrive_instruction());

    // Check maneuver verbal_arrive_instruction
    EXPECT_EQ(man->verbal_arrive_instruction(), expected_man->verbal_arrive_instruction());
  }
}

void VerifyToStayOn(const Maneuver& maneuver, bool expected_to_stay_on) {
  // Check to stay on attribute
  EXPECT_EQ(maneuver.to_stay_on(), expected_to_stay_on);
}

void PopulateStartManeuverList_0(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulateStartManeuverList_1(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_2(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulateStartManeuverList_3(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_4(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"US 222", 1}, {"PA 272", 1}},
                   {{"North Prince Street", 0}, {"US 222", 1}, {"PA 272", 1}}, {}, "", 5.098166, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 173, 143, 0, 45, 0, 88, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulateStartManeuverList_5(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulateStartManeuverList_6(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_7(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulateStartManeuverList_8(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_9(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"US 222", 1}, {"PA 272", 1}},
                   {{"North Prince Street", 0}, {"US 222", 1}, {"PA 272", 1}}, {}, "", 5.098166, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 173, 143, 0, 45, 0, 88, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulateStartManeuverList_10(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulateStartManeuverList_11(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_12(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulateStartManeuverList_13(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_13_unnamed_walkway(std::list<Maneuver>& maneuvers,
                                                  const std::string& country_code,
                                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.050737, 35, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 213, 209, 0, 3, 0, 4, 0, 0, 0,
                   0, 0, 0, 1, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 1, 0, 0, 36,
                   0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_13_pedestrian_crossing(std::list<Maneuver>& maneuvers,
                                                      const std::string& country_code,
                                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.050737, 35, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 213, 209, 0, 3, 0, 4, 0, 0, 0,
                   0, 0, 0, 1, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 1, 0, 0, 36,
                   0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_pedestrian_crossing(true);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_14(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"US 222", 1}, {"PA 272", 1}},
                   {{"North Prince Street", 0}, {"US 222", 1}, {"PA 272", 1}}, {}, "", 5.098166, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 173, 143, 0, 45, 0, 88, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulateStartManeuverList_15(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulateStartManeuverList_16(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 88, 80, 0, 1, 0, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_17(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulateStartManeuverList_18(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"5th Avenue", 0}}, {}, {}, "", 0.224001, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 209, 209, 0, 3, 0, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_18_unnamed_cycleway(std::list<Maneuver>& maneuvers,
                                                   const std::string& country_code,
                                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 2.675882, 386, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 84, 70, 0, 2, 0, 93, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 1, 0, 482, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_18_unnamed_mountain_bike_trail(std::list<Maneuver>& maneuvers,
                                                              const std::string& country_code,
                                                              const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart, {}, {}, {},
                   "", 0.200000, 29, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 269, 221, 0, 2, 0, 21, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 1, 36,
                   0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateStartManeuverList_19(std::list<Maneuver>& maneuvers,
                                  const std::string& country_code,
                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"US 222", 1}, {"PA 272", 1}},
                   {{"North Prince Street", 0}, {"US 222", 1}, {"PA 272", 1}}, {}, "", 5.098166, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 173, 143, 0, 45, 0, 88, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulateDestinationManeuverList_0(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kDestination, {},
                   {}, {}, "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 6, 6, 7, 7, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
}

void PopulateDestinationManeuverList_1(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kDestination, {},
                   {}, {}, "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 120, 120, 1756, 1756, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
}

void PopulateDestinationManeuverList_2(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kDestinationRight,
                   {}, {}, {}, "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4, 6, 6, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
}

void PopulateDestinationManeuverList_3(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kDestinationLeft,
                   {}, {}, {}, "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4, 6, 6, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
}

void PopulateBecomesManeuverList_0(std::list<Maneuver>& maneuvers,
                                   const std::string& country_code,
                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kSlightRight,
                   {{"Vine Street", 0}}, {}, {}, "", 0.365000, 25, 25,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 355, 25, 47, 49, 497, 504, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   19, 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kBecomes,
                   {{"Middletown Road", 0}}, {}, {}, "", 1.489000, 98, 4,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 29, 30, 49, 65, 504, 529, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0,
                   0, 97, 0);
}

void PopulateContinueManeuverList_0(std::list<Maneuver>& maneuvers,
                                    const std::string& country_code,
                                    const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kContinue, {}, {},
                   {}, "", 0.097000, 26, 0, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 291, 323, 1, 3, 3, 11, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
}

void PopulateContinueManeuverList_1(std::list<Maneuver>& maneuvers,
                                    const std::string& country_code,
                                    const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kContinue, {}, {},
                   {}, "", 0.097000, 26, 0, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 291, 323, 1, 3, 3, 11, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateContinueManeuverList_2(std::list<Maneuver>& maneuvers,
                                    const std::string& country_code,
                                    const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kContinue,
                   {{"10th Avenue", 0}}, {}, {}, "", 0.481000, 34, 6,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 185, 218, 2, 5, 4, 11, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
}

void PopulateContinueManeuverList_3(std::list<Maneuver>& maneuvers,
                                    const std::string& country_code,
                                    const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kContinue,
                   {{"10th Avenue", 0}}, {}, {}, "", 0.481000, 34, 6,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 185, 218, 2, 5, 4, 11, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateTurnManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kLeft, {}, {}, {},
                   "", 0.824000, 60, 282, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 260, 268, 1, 2, 1, 8, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 1, "", "", "", "", 0);
}

void PopulateTurnManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"Middletown Road", 0}}, {}, {}, "", 2.011000, 152, 271,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 28, 14, 5, 10, 40, 83, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 1, 0, 0, 0,
                   129, 0);
}

void PopulateTurnManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"MD 924", 1}}, {{"North Bond Street", 0}, {"US 1 Business", 1}, {"MD 924", 1}},
                   {}, "", 0.840369, 111, 282, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 141, 144, 2, 16, 2, 27, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 0);
}

void PopulateTurnManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kRight,
                   {{"Sunstone Drive", 0}}, {}, {}, "", 0.077000, 49, 89,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 75, 77, 1, 3, 1, 3, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 1, "", "", "", "", 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kRight,
                   {{"Sunstone Drive", 0}}, {}, {}, "", 0.038000, 12, 90,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 167, 167, 3, 4, 3, 4, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 0, "", "", "", "", 1);
  maneuver2.set_to_stay_on(true);
}

void PopulateSharpManeuverList_0(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kSharpLeft, {}, {},
                   {}, "", 0.824000, 60, 201, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 260, 268, 1, 2, 1, 8, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 1, "", "", "", "", 0);
}

void PopulateSharpManeuverList_1(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kSharpRight,
                   {{"Flatbush Avenue", 0}}, {}, {}, "", 0.192229, 44, 147,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 322, 322, 1, 4, 1, 4, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 1, "", "", "", "", 0);
}

void PopulateSharpManeuverList_2(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kSharpLeft,
                   {{"MD 924", 1}}, {{"North Bond Street", 0}, {"US 1 Business", 1}, {"MD 924", 1}},
                   {}, "", 0.840369, 111, 201, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 141, 144, 2, 16, 2, 27, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 0);
}

void PopulateSharpManeuverList_3(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kSharpRight,
                   {{"Sunstone Drive", 0}}, {}, {}, "", 0.077000, 49, 147,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 75, 77, 1, 3, 1, 3, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 1, "", "", "", "", 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kSharpRight,
                   {{"Sunstone Drive", 0}}, {}, {}, "", 0.038000, 12, 149,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 167, 167, 3, 4, 3, 4, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 0, "", "", "", "", 1);
  maneuver2.set_to_stay_on(true);
}

void PopulateBearManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kSlightRight, {},
                   {}, {}, "", 0.018000, 9, 37, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 303, 303, 3, 4, 12, 13, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 0, "", "", "", "", 0);
}

void PopulateBearManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kSlightLeft,
                   {{"Arlen Road", 0}}, {}, {}, "", 0.118000, 22, 323,
                   Maneuver::RelativeDirection::kLeft, DirectionsLeg_Maneuver_CardinalDirection_kEast,
                   86, 126, 210, 212, 1566, 1576, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0,
                   0, 1, 0, "", "", "", "", 0);
}

void PopulateBearManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kSlightRight,
                   {{"US 1 Business", 1}}, {{"Belair Road", 0}, {"US 1 Business", 1}}, {}, "",
                   3.431836, 275, 30, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 36, 115, 82, 115, 257, 338, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0);
}

void PopulateBearManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                   {{"US 15", 1}}, {{"Catoctin Mountain Highway", 0}, {"US 15", 1}}, {}, "",
                   18.278002, 928, 34, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 201, 204, 161, 187, 1461, 1805, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 900, 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kSlightLeft,
                   {{"US 15 South", 1}}, {}, {}, "", 4.137000, 232, 349,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 193, 197, 187, 197, 1805, 1878, 0,
                   0, 0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 1, "", "", "", "", 0, 0, 0,
                   0, 200, 0);
  maneuver2.set_to_stay_on(true);
}

void PopulateUturnManeuverList_0(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kUturnLeft, {}, {},
                   {}, "", 0.592000, 28, 180, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 76, 76, 3, 4, 24, 25, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
}

void PopulateUturnManeuverList_1(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kUturnRight,
                   {{"Bunker Hill Road", 0}}, {}, {}, "", 0.592000, 28, 180,
                   Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 76, 76, 3, 4, 24, 25, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0);
}

void PopulateUturnManeuverList_2(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kRight,
                   {{"Bunker Hill Road", 0}}, {}, {}, "", 0.287000, 28, 81,
                   Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 335, 337, 2, 3, 36, 46, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 1, "", "", "", "", 0, 0, 0, 0,
                   20, 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kUturnLeft,
                   {{"Bunker Hill Road", 0}}, {}, {}, "", 0.287000, 25, 180,
                   Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 157, 155, 3, 4, 46, 56, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 20,
                   0);
  maneuver2.set_to_stay_on(true);
}

void PopulateUturnManeuverList_3(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kUturnLeft, {}, {},
                   {{"Devonshire Road", 0}}, "", 0.072697, 47, 180,
                   Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 212, 221, 1, 3, 1, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 0);
}

void PopulateUturnManeuverList_4(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kUturnLeft,
                   {{"Jonestown Road", 0}, {"US 22", 1}}, {}, {{"Devonshire Road", 0}}, "", 0.072697,
                   47, 180, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 212, 221, 1, 3, 1, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 0);
}

void PopulateUturnManeuverList_5(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kStart,
                   {{"Jonestown Road", 0}, {"US 22", 1}}, {}, {}, "", 0.062923, 2, 0,
                   Maneuver::RelativeDirection::kNone,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 36, 32, 0, 1, 0, 1, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 2,
                   0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kUturnLeft,
                   {{"Jonestown Road", 0}, {"US 22", 1}}, {}, {{"Devonshire Road", 0}}, "", 0.072697,
                   47, 180, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 212, 221, 1, 3, 1, 3, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 40,
                   0);
  maneuver2.set_to_stay_on(true);
}

void PopulateRampStraightManeuverList_0(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampStraight, {},
                   {}, {}, "", 2.4, 37, 340, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 60, 57, 9, 10, 88, 92, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   30, 0);
}

void PopulateRampStraightManeuverList_1(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampStraight, {},
                   {}, {}, "", 0.374000, 37, 340, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 60, 57, 9, 10, 88, 92, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("US 322 East", 1, 1)}, {}, {}, 0, 0, 0,
                   0, 1, 0, "", "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampStraightManeuverList_2(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampStraight, {},
                   {}, {}, "", 0.374000, 37, 340, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 60, 57, 9, 10, 88, 92, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {std::make_tuple("Hershey", 0, 0)}, {}, 0, 0, 0, 0, 1,
                   0, "", "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampStraightManeuverList_3(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampStraight, {},
                   {}, {}, "", 0.374000, 37, 340, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 60, 57, 9, 10, 88, 92, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {},
                   {std::make_tuple("US 322 East", 1, 1), std::make_tuple("US 422 East", 1, 1),
                    std::make_tuple("US 522 East", 1, 1), std::make_tuple("US 622 East", 1, 1),
                    std::make_tuple("US 722 East", 1, 1)},
                   {std::make_tuple("Hershey", 0, 1), std::make_tuple("Palmdale", 0, 1),
                    std::make_tuple("Palmyra", 0, 1), std::make_tuple("Campbelltown", 0, 1),
                    std::make_tuple("Eprata", 0, 1)},
                   {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 30, 0);
} // namespace

void PopulateRampStraightManeuverList_4(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampStraight, {},
                   {}, {}, "", 0.374000, 37, 340, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 60, 57, 9, 10, 88, 92, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}, 0, 0,
                   0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 99, 137, 14, 15, 61, 71, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 9,
                   0, false);
}

void PopulateRampManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 99, 137, 14, 15, 61, 71, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {std::make_tuple("I 95", 1, 0)}, {}, {}, 0, 0, 0, 0, 1, 0,
                   "", "", "", "", 0, 0, 0, 0, 9, 0, false);
}

void PopulateRampManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampLeft,
                   {{"NY 27 East", 1}, {"South Conduit Avenue", 0}}, {}, {}, "", 0.124000, 6, 353,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 105, 102, 24, 25, 204, 206, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {std::make_tuple("JFK", 0, 0)}, {}, 0, 0, 0, 0, 1, 0,
                   "", "", "", "", 0, 0, 0, 0, 6, 0, true);
}

void PopulateRampManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampLeft,
                   {{"NY 27 East", 1}, {"South Conduit Avenue", 0}}, {}, {}, "", 0.124000, 6, 353,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 105, 102, 24, 25, 204, 206, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("South Conduit Avenue", 0, 0)},
                   {std::make_tuple("JFK", 0, 0)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   6, 0, true);
}

void PopulateRampManeuverList_4(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 99, 137, 14, 15, 61, 71, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}, 0, 0, 0,
                   0, 1, 0, "", "", "", "", 0, 0, 0, 0, 9, 0, false);
}

void PopulateRampManeuverList_5(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.256000, 46, 92, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 86, 129, 11, 13, 51, 62, 1, 0, 0,
                   0, 0, 1, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23,
                   0);
}

void PopulateRampManeuverList_6(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampLeft, {}, {},
                   {}, "", 0.539000, 31, 266, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 86, 277, 24, 26, 60, 79, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 283 West", 1, 1)}, {}, {}, 0, 1, 0, 0,
                   1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_7(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampLeft, {}, {},
                   {}, "", 0.539000, 31, 266, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 86, 277, 24, 26, 60, 79, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {},
                   {std::make_tuple("Harrisburg", 0, 0),
                    std::make_tuple("Harrisburg International Airport", 0, 0)},
                   {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_8(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampLeft, {}, {},
                   {}, "", 0.539000, 31, 266, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 86, 277, 24, 26, 60, 79, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {std::make_tuple("PA 283 West", 1, 1)},
                   {std::make_tuple("Harrisburg", 0, 0),
                    std::make_tuple("Harrisburg International Airport", 0, 0)},
                   {}, 0, 1, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_9(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.256000, 46, 92, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 86, 129, 11, 13, 51, 62, 1, 0, 0,
                   0, 0, 1, 0, 0, 0, {}, {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}, 1, 0, 0,
                   0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_10(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 99, 137, 14, 15, 61, 71, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 9,
                   0);
}

void PopulateRampManeuverList_11(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code,
                                 bool drive_on_right = true) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 99, 137, 14, 15, 61, 71, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {std::make_tuple("I 95", 1, 0)}, {}, {}, 0, 0, 0, 0, 1, 0,
                   "", "", "", "", 0, 0, 0, 0, 9, 0, drive_on_right);
}

void PopulateRampManeuverList_12(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight,
                   {{"NY 27 East", 1}, {"South Conduit Avenue", 0}}, {}, {}, "", 0.124000, 6, 353,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 105, 102, 24, 25, 204, 206, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {std::make_tuple("JFK", 0, 0)}, {}, 0, 0, 0, 0, 1, 0,
                   "", "", "", "", 0, 0, 0, 0, 6, 0);
}

void PopulateRampManeuverList_13(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampLeft,
                   {{"NY 27 East", 1}, {"South Conduit Avenue", 0}}, {}, {}, "", 0.124000, 6, 353,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 105, 102, 24, 25, 204, 206, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("South Conduit Avenue", 0, 0)},
                   {std::make_tuple("JFK", 0, 0)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   6, 0);
}

void PopulateRampManeuverList_14(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRampRight, {}, {},
                   {}, "", 0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 99, 137, 14, 15, 61, 71, 1, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}, 0, 0, 0,
                   0, 1, 0, "", "", "", "", 0, 0, 0, 0, 9, 0);
}

void PopulateExitManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   23, 0, false);
}

void PopulateExitManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)}, {}, {}, {}, 0, 0, 0, 0, 1,
                   0, "", "", "", "", 0, 0, 0, 0, 23, 0, false);
}

void PopulateExitManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("US 322 West", 1, 2)}, {}, {}, 0, 0, 0,
                   0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0, false);
}

void PopulateExitManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)},
                   {std::make_tuple("US 322 West", 1, 2)}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 23, 0, false);
}

void PopulateExitManeuverList_4(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {std::make_tuple("Lewistown", 0, 1)}, {}, 0, 0, 0, 0,
                   1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0, false);
}

void PopulateExitManeuverList_5(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)}, {},
                   {std::make_tuple("Lewistown", 0, 1)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 23, 0, false);
}

void PopulateExitManeuverList_6(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("US 322 West", 1, 2)},
                   {std::make_tuple("Lewistown", 0, 1)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 23, 0, false);
}

void PopulateExitManeuverList_7(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)},
                   {std::make_tuple("US 322 West", 1, 2), std::make_tuple("US 22 West", 1, 1),
                    std::make_tuple("US 22 East", 1, 0), std::make_tuple("PA 230 East", 1, 0),
                    std::make_tuple("Cameron Street", 0, 0)},
                   {std::make_tuple("Lewistown", 0, 1), std::make_tuple("State College", 0, 1),
                    std::make_tuple("Harrisburg", 0, 0)},
                   {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0, false);
}

void PopulateExitManeuverList_8(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0);
}

void PopulateExitManeuverList_10(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("MD 43 East", 1, 1)}, {},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0);
}

void PopulateExitManeuverList_12(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {std::make_tuple("White Marsh", 0, 0)},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0);
}

void PopulateExitManeuverList_14(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("MD 43 East", 1, 1)},
                   {std::make_tuple("White Marsh", 0, 0)},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0);
}

void PopulateExitManeuverList_15(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   23, 0, true);
}

void PopulateExitManeuverList_16(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)}, {}, {}, {}, 0, 0, 0, 0, 1,
                   0, "", "", "", "", 0, 0, 0, 0, 23, 0, true);
}

void PopulateExitManeuverList_17(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("US 322 West", 1, 2)}, {}, {}, 0, 0, 0,
                   0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0, true);
}

void PopulateExitManeuverList_18(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)},
                   {std::make_tuple("US 322 West", 1, 2)}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 23, 0, true);
}

void PopulateExitManeuverList_19(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {std::make_tuple("Lewistown", 0, 1)}, {}, 0, 0, 0, 0,
                   1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0, true);
}

void PopulateExitManeuverList_20(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)}, {},
                   {std::make_tuple("Lewistown", 0, 1)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 23, 0, true);
}

void PopulateExitManeuverList_21(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("US 322 West", 1, 2)},
                   {std::make_tuple("Lewistown", 0, 1)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 23, 0, true);
}

void PopulateExitManeuverList_22(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight,
                   {{"US 322 West", 1}}, {}, {}, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 272, 278, 42, 43, 260, 264, 1, 0,
                   0, 0, 0, 0, 0, 0, 0, {std::make_tuple("67 B-A", 0, 0)},
                   {std::make_tuple("US 322 West", 1, 2), std::make_tuple("US 22 West", 1, 1),
                    std::make_tuple("US 22 East", 1, 0), std::make_tuple("PA 230 East", 1, 0),
                    std::make_tuple("Cameron Street", 0, 0)},
                   {std::make_tuple("Lewistown", 0, 1), std::make_tuple("State College", 0, 1),
                    std::make_tuple("Harrisburg", 0, 0)},
                   {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 23, 0, true);
}

void PopulateExitManeuverList_23(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0, false);
}

void PopulateExitManeuverList_25(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("MD 43 East", 1, 1)}, {},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0, false);
}

void PopulateExitManeuverList_27(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {std::make_tuple("White Marsh", 0, 0)},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0, false);
}

void PopulateExitManeuverList_29(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitLeft,
                   {{"MD 43 East", 1}}, {}, {}, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 135, 83, 158, 160, 1420, 1444,
                   1, 0, 0, 0, 0, 0, 0, 0, 0, {}, {std::make_tuple("MD 43 East", 1, 1)},
                   {std::make_tuple("White Marsh", 0, 0)},
                   {std::make_tuple("White Marsh Boulevard", 0, 0)}, 0, 0, 0, 0, 1, 0, "", "", "", "",
                   0, 0, 0, 0, 46, 0, false);
}

void PopulateKeepManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayStraight, {},
                   {}, {}, "", 0.068000, 4, 1, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kEast, 91, 97, 2, 3, 8, 10, 1, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 1, 1, 0, "", "", "", "", 0, 0, 0, 0, 4, 0);
}

void PopulateKeepManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayRight, {}, {},
                   {}, "", 14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 221, 214, 21, 45, 148, 323, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {std::make_tuple("62", 0, 0)}, {}, {}, {}, 0, 0, 0, 1, 1,
                   0, "", "", "", "", 0, 0, 0, 0, 581, 0);
}

void PopulateKeepManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayRight,
                   {{"I 895 South", 1}}, {}, {}, "", 14.464000, 634, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 221, 214, 21, 45, 148, 323, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 1, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 581, 0);
}

void PopulateKeepManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayRight,
                   {{"I 895 South", 1}}, {}, {}, "", 14.464000, 634, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 221, 214, 21, 45, 148, 323, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {std::make_tuple("62", 0, 0)}, {}, {}, {}, 0, 0, 0, 1, 1,
                   0, "", "", "", "", 0, 0, 0, 0, 581, 0);
}

void PopulateKeepManeuverList_4(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayRight, {}, {},
                   {}, "", 14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 221, 214, 21, 45, 148, 323, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {}, {}, {std::make_tuple("Annapolis", 0, 0)}, {}, 0, 0, 0,
                   1, 1, 0, "", "", "", "", 0, 0, 0, 0, 581, 0);
}

void PopulateKeepManeuverList_5(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayRight, {}, {},
                   {}, "", 14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 221, 214, 21, 45, 148, 323, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {std::make_tuple("62", 0, 0)}, {},
                   {std::make_tuple("Annapolis", 0, 0)}, {}, 0, 0, 0, 1, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 581, 0);
}

void PopulateKeepManeuverList_6(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayRight, {}, {},
                   {}, "", 14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 221, 214, 21, 45, 148, 323, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {},
                   {std::make_tuple("I 895 South", 1, 1),
                    std::make_tuple("Baltimore Harbor Tunnel Thruway", 0, 0)},
                   {std::make_tuple("Annapolis", 0, 0)}, {}, 0, 0, 0, 1, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 581, 0);
}

void PopulateKeepManeuverList_7(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayRight,
                   {{"I 895 South", 1}}, {}, {}, "", 14.464000, 634, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 221, 214, 21, 45, 148, 323, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {std::make_tuple("62", 0, 0)},
                   {std::make_tuple("I 895 South", 1, 1),
                    std::make_tuple("Baltimore Harbor Tunnel Thruway", 0, 0)},
                   {std::make_tuple("Annapolis", 0, 0)}, {}, 0, 0, 0, 1, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 581, 0);
}

void PopulateKeepToStayOnManeuverList_0(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kMerge,
                   {{"I 95 South", 1}, {"John F. Kennedy Memorial Highway", 0}}, {}, {}, "",
                   23.639002, 843, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 234, 219, 24, 34, 210, 380, 0,
                   0, 0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayLeft,
                   {{"I 95 South", 1}}, {}, {}, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 219, 232, 34, 45, 380, 491, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {}, {std::make_tuple("I 95 South", 1, 0)}, {}, {}, 0, 0, 0,
                   1, 0, 0, "", "", "", "", 0, 0, 0, 0, 334, 0);
  maneuver2.set_to_stay_on(true);
}

void PopulateKeepToStayOnManeuverList_1(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kMerge,
                   {{"I 95 South", 1}, {"John F. Kennedy Memorial Highway", 0}}, {}, {}, "",
                   23.639002, 843, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 234, 219, 24, 34, 210, 380, 0,
                   0, 0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayLeft,
                   {{"I 95 South", 1}}, {}, {}, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 219, 232, 34, 45, 380, 491, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {std::make_tuple("62", 0, 0)},
                   {std::make_tuple("I 95 South", 1, 0)}, {}, {}, 0, 0, 0, 1, 0, 0, "", "", "", "", 0,
                   0, 0, 0, 334, 0);
  maneuver2.set_to_stay_on(true);
}

void PopulateKeepToStayOnManeuverList_2(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kMerge,
                   {{"I 95 South", 1}, {"John F. Kennedy Memorial Highway", 0}}, {}, {}, "",
                   23.639002, 843, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 234, 219, 24, 34, 210, 380, 0,
                   0, 0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayLeft,
                   {{"I 95 South", 1}}, {}, {}, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 219, 232, 34, 45, 380, 491, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {}, {std::make_tuple("I 95 South", 1, 0)},
                   {std::make_tuple("Baltimore", 0, 0)}, {}, 0, 0, 0, 1, 0, 0, "", "", "", "", 0, 0,
                   0, 0, 334, 0);
  maneuver2.set_to_stay_on(true);
}

void PopulateKeepToStayOnManeuverList_3(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kMerge,
                   {{"I 95 South", 1}, {"John F. Kennedy Memorial Highway", 0}}, {}, {}, "",
                   23.639002, 843, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 234, 219, 24, 34, 210, 380, 0,
                   0, 0, 0, 0, 0, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kStayLeft,
                   {{"I 95 South", 1}}, {}, {}, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 219, 232, 34, 45, 380, 491, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {std::make_tuple("62", 0, 0)},
                   {std::make_tuple("I 95 South", 1, 0)}, {std::make_tuple("Baltimore", 0, 0)}, {}, 0,
                   0, 0, 1, 0, 0, "", "", "", "", 0, 0, 0, 0, 334, 0);
  maneuver2.set_to_stay_on(true);
}

void PopulateMergeManeuverList_0(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight, {},
                   {}, {}, "", 0.864000, 34, 6, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 174, 241, 39, 41, 158, 180, 1, 0,
                   0, 0, 0, 1, 0, 0, 0, {}, {std::make_tuple("I 76 West", 1, 1)},
                   {std::make_tuple("Pittsburgh", 0, 0)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 34, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kMerge, {}, {},
                   {}, "", 7.624001, 245, 1, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 242, 293, 41, 42, 180, 236, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 243, 0);
}

void PopulateMergeManeuverList_1_1(std::list<Maneuver>& maneuvers,
                                   const std::string& country_code,
                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight, {},
                   {}, {}, "", 0.864000, 34, 6, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 174, 241, 39, 41, 158, 180, 1, 0,
                   0, 0, 0, 1, 0, 0, 0, {}, {std::make_tuple("I 76 West", 1, 1)},
                   {std::make_tuple("Pittsburgh", 0, 0)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 34, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kMerge,
                   {{"I 76 West", 1}, {"Pennsylvania Turnpike", 0}}, {}, {}, "", 7.624001, 245, 1,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 242, 293, 41, 42, 180, 236, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 243, 0);
}

void PopulateMergeManeuverList_1_2(std::list<Maneuver>& maneuvers,
                                   const std::string& country_code,
                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kExitRight, {},
                   {}, {}, "", 2.1, 34, 6, Maneuver::RelativeDirection::kKeepRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 174, 241, 39, 41, 158, 180, 1, 0,
                   0, 0, 0, 1, 0, 0, 0, {}, {std::make_tuple("I 76 West", 1, 1)},
                   {std::make_tuple("Pittsburgh", 0, 0)}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0,
                   0, 0, 34, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kMerge,
                   {{"I 76 West", 1}, {"Pennsylvania Turnpike", 0}}, {}, {}, "", 7.624001, 245, 1,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 242, 293, 41, 42, 180, 236, 0,
                   0, 0, 0, 0, 1, 0, 1, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 243, 0);
}

void PopulateEnterRoundaboutManeuverList_0(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                   {}, {}, {}, "", 0.043000, 2, 41, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 264, 167, 135, 139, 1457, 1464, 0,
                   0, 0, 0, 1, 0, 0, 0, 0, {}, {}, {}, {}, 1, 2, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 2, 0);
}

void PopulateEnterRoundaboutManeuverList_1(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code,
                                           uint32_t roundabout_exit_count) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRoundaboutEnter,
                   {}, {}, {}, "", 0.043000, 2, 41, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 264, 167, 135, 139, 1457, 1464, 0,
                   0, 0, 0, 1, 0, 0, 0, 0, {}, {}, {}, {}, 1, 2, roundabout_exit_count, 0, 1, 0, "",
                   "", "", "", 0, 0, 0, 0, 2, 0);
}

void PopulateExitRoundaboutManeuverList_0(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                   {}, {}, {}, "", 1.041000, 69, 24, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 224, 262, 8, 11, 32, 60, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   64, 0);
}

void PopulateExitRoundaboutManeuverList_1(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                   {{"Philadelphia Road", 0}, {"MD 7", 1}}, {}, {}, "", 1.041000, 69, 24,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 224, 262, 8, 11, 32, 60, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   64, 0);
}

void PopulateExitRoundaboutManeuverList_2(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kRoundaboutExit,
                   {{"US 15", 1}}, {{"Catoctin Mountain Highway", 0}, {"US 15", 1}}, {}, "",
                   18.278002, 923, 34, Maneuver::RelativeDirection::kRight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 201, 204, 139, 154, 1464, 1808, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 914, 0);
}

void PopulateEnterFerryManeuverList_0(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryEnter, {},
                   {}, {}, "", 1.446000, 822, 4, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 280, 280, 5, 6, 8, 9, 0, 0, 1, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 521,
                   0);
}

void PopulateEnterFerryManeuverList_1(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryEnter,
                   {{"Millersburg FERRY", 0}}, {}, {}, "", 1.446000, 822, 4,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 280, 280, 5, 6, 8, 9, 0, 0, 1, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 521,
                   0);
}

void PopulateEnterFerryManeuverList_2(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryEnter,
                   {{"Bridgeport - Port Jefferson", 0}}, {}, {}, "", 27.731001, 3628, 24,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 151, 142, 3, 4, 15, 30, 0, 0,
                   1, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   3328, 0);
}

void PopulateExitFerryManeuverList_0(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulateExitFerryManeuverList_1(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_2(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulateExitFerryManeuverList_3(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_4(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"US 9", 1}}, {{"Cape May-Lewes Ferry Entrance", 0}, {"US 9", 1}}, {}, "",
                   0.099000, 7, 356, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 31, 62, 23, 25, 71, 75, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   5, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulateExitFerryManeuverList_5(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulateExitFerryManeuverList_6(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_7(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulateExitFerryManeuverList_8(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kDrive);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_9(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"US 9", 1}}, {{"Cape May-Lewes Ferry Entrance", 0}, {"US 9", 1}}, {}, "",
                   0.099000, 7, 356, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 31, 62, 23, 25, 71, 75, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   5, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulateExitFerryManeuverList_10(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulateExitFerryManeuverList_11(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_12(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulateExitFerryManeuverList_13(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_14(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"US 9", 1}}, {{"Cape May-Lewes Ferry Entrance", 0}, {"US 9", 1}}, {}, "",
                   0.099000, 7, 356, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 31, 62, 23, 25, 71, 75, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   5, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulateExitFerryManeuverList_15(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulateExitFerryManeuverList_16(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit, {}, {},
                   {}, "", 0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthEast, 144, 94, 4, 5, 30, 32, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   12, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_17(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulateExitFerryManeuverList_18(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"Ferry Lane", 0}}, {}, {}, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kWest, 287, 262, 6, 13, 9, 40, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 70,
                   0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulateExitFerryManeuverList_19(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kFerryExit,
                   {{"US 9", 1}}, {{"Cape May-Lewes Ferry Entrance", 0}, {"US 9", 1}}, {}, "",
                   0.099000, 7, 356, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 31, 62, 23, 25, 71, 75, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 1, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0,
                   5, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulateTransitConnectionStartManeuverList_0(std::list<Maneuver>& maneuvers,
                                                  const std::string& country_code,
                                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionStart, {{"Broadway", 0}}, {}, {}, "",
                   0.036000, 28, 0, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 213, 212, 2, 3, 2, 4, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 25,
                   0);
}

void PopulateTransitConnectionStartManeuverList_1(std::list<Maneuver>& maneuvers,
                                                  const std::string& country_code,
                                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionStart, {{"Townsend Street", 0}}, {},
                   {}, "", 0.084000, 60, 204, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 226, 175, 27, 28, 733, 736, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 0, 0, "", "", "", "", 0, 0, 0,
                   0, 59, 0);
  maneuver.set_transit_connection_platform_info(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation,
                             "s-9q8yyv42k3-caltrain~sanfranciscostation",
                             "CALTRAIN - SAN FRANCISCO STATION", "", "2016-03-29T08:57-04:00", 1,
                             0.0f, 0.0f));
}

void PopulateTransitConnectionStartManeuverList_2(std::list<Maneuver>& maneuvers,
                                                  const std::string& country_code,
                                                  const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionStart, {{"Broadway", 0}}, {}, {}, "",
                   0.036000, 28, 0, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 213, 212, 2, 3, 2, 4, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 25,
                   0);
  maneuver.set_transit_connection_platform_info(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                             "8 St - NYU", "", "2016-03-29T08:02-04:00", 0, 0.0f, 0.0f));
}

void PopulateTransitConnectionTransferManeuverList_0(std::list<Maneuver>& maneuvers,
                                                     const std::string& country_code,
                                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionTransfer, {{"Broadway", 0}}, {}, {},
                   "", 0.036000, 25, 196, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 32, 33, 11, 12, 16, 18, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   25, 0);
}

void PopulateTransitConnectionTransferManeuverList_1(std::list<Maneuver>& maneuvers,
                                                     const std::string& country_code,
                                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionTransfer, {{"Townsend Street", 0}},
                   {}, {}, "", 0.084000, 60, 204, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 226, 175, 27, 28, 733, 736, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 0, 0, "", "", "", "", 0, 0, 0,
                   0, 59, 0);
  maneuver.set_transit_connection_platform_info(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation,
                             "s-9q8yyv42k3-caltrain~sanfranciscostation",
                             "CALTRAIN - SAN FRANCISCO STATION", "", "2016-03-29T08:57-04:00", 1,
                             0.0f, 0.0f));
}

void PopulateTransitConnectionTransferManeuverList_2(std::list<Maneuver>& maneuvers,
                                                     const std::string& country_code,
                                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionTransfer, {{"Broadway", 0}}, {}, {},
                   "", 0.036000, 25, 196, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 32, 33, 11, 12, 16, 18, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   25, 0);
  maneuver.set_transit_connection_platform_info(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21s",
                             "8 St - NYU", "2016-03-29T08:19-04:00", "", 0, 0.0f, 0.0f));
}

void PopulateTransitConnectionDestinationManeuverList_0(std::list<Maneuver>& maneuvers,
                                                        const std::string& country_code,
                                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionDestination, {{"Broadway", 0}}, {},
                   {}, "", 0.036000, 25, 196, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 32, 33, 11, 12, 16, 18, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   25, 0);
}

void PopulateTransitConnectionDestinationManeuverList_1(std::list<Maneuver>& maneuvers,
                                                        const std::string& country_code,
                                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionDestination,
                   {{"Townsend Street", 0}}, {}, {}, "", 0.084000, 60, 204,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 226, 175, 27, 28, 733, 736, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 0, 0, "", "", "", "", 0, 0, 0,
                   0, 59, 0);
  maneuver.set_transit_connection_platform_info(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation,
                             "s-9q8yyv42k3-caltrain~sanfranciscostation",
                             "CALTRAIN - SAN FRANCISCO STATION", "", "2016-03-29T08:57-04:00", 1,
                             0.0f, 0.0f));
}

void PopulateTransitConnectionDestinationManeuverList_2(std::list<Maneuver>& maneuvers,
                                                        const std::string& country_code,
                                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kTransitConnectionDestination, {{"Broadway", 0}}, {},
                   {}, "", 0.036000, 25, 196, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 32, 33, 11, 12, 16, 18, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   25, 0);
  maneuver.set_transit_connection_platform_info(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21s",
                             "8 St - NYU", "2016-03-29T08:19-04:00", "", 0, 0.0f, 0.0f));
}

void PopulateTransitManeuverList_0_train(std::list<Maneuver>& maneuvers,
                                         const std::string& country_code,
                                         const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransit, {}, {},
                   {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "", "", "", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));

  maneuver.set_transit_type(valhalla::TransitType::kRail);
}

void PopulateTransitManeuverList_0(std::list<Maneuver>& maneuvers,
                                   const std::string& country_code,
                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransit, {}, {},
                   {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "R", "Broadway Local",
                      "", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulateTransitManeuverList_1_cable_car(std::list<Maneuver>& maneuvers,
                                             const std::string& country_code,
                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransit, {}, {},
                   {}, "", 0.826000, 502, 289, Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 171, 171, 2, 9, 3, 24, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 595,
                   0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-9q8zn-powell~hyde", 0, 26285, "", "",
                      "Powell & Market", 16777215, 0, "", "o-9q8y-sfmta",
                      "San Francisco Municipal Transportation Agency", "http://www.sfmta.com/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn2c3gg-hydest~vallejost",
                             "Hyde St & Vallejo St", "2016-05-17T08:06-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn2cpr9-hydest~greenst",
                             "Hyde St & Green St", "2016-05-17T08:06-04:00", "2016-05-17T08:06-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn31h5q-hydest~unionst",
                             "Hyde St & Union St", "2016-05-17T08:06-04:00", "2016-05-17T08:06-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn32fnv-hydest~filbertst",
                             "Hyde St & Filbert St", "2016-05-17T08:05-04:00",
                             "2016-05-17T08:05-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn32yg5-hydest~greenwichst",
                             "Hyde St & Greenwich St", "2016-05-17T08:05-04:00",
                             "2016-05-17T08:05-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn38ez1-hydest~lombardst",
                             "Hyde St & Lombard St", "2016-05-17T08:04-04:00",
                             "2016-05-17T08:04-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn3b9cv-hydest~chestnutst",
                             "Hyde St & Chestnut St", "2016-05-17T08:04-04:00",
                             "2016-05-17T08:04-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-9q8zn60kc1-hydest~bayst",
                             "Hyde St & Bay St", "", "2016-05-17T08:03-04:00", 1, 0.0f, 0.0f)));

  maneuver.set_transit_type(valhalla::TransitType::kCableCar);
}

void PopulateTransitManeuverList_1_stop_count_1(std::list<Maneuver>& maneuvers,
                                                const std::string& country_code,
                                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransit, {}, {},
                   {}, "", 0.794000, 395, 10, Maneuver::RelativeDirection::kKeepLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthEast, 27, 27, 2, 3, 3, 5, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   572, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84508, "R", "Broadway Local",
                      "FOREST HILLS - 71 AV", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation,
                             "s-dr5rkw4r8c-atlanticav~barclaysctr<r31n", "Atlantic Av - Barclays Ctr",
                             "2016-05-17T08:08-04:00", "", 0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rkmp4s9-unionst<r32n",
                                       "Union St", "", "2016-05-17T08:06-04:00", 0, 0.0f, 0.0f)));
}

void PopulateTransitManeuverList_1_stop_count_2(std::list<Maneuver>& maneuvers,
                                                const std::string& country_code,
                                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransit, {}, {},
                   {}, "", 1.097000, 277, 179, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouth, 187, 189, 4, 6, 7, 9, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 0, "", "", "", "", 0, 0, 0, 0, 790,
                   0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 135877, "R", "Broadway Local",
                      "BAY RIDGE - 95 ST", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20s",
                             "14 St - Union Sq", "2016-05-17T08:08-04:00", "2016-05-17T08:11-04:00",
                             0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19s", "23 St",
                             "2016-05-17T08:07-04:00", "2016-05-17T08:07-04:00", 0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18s",
                                       "28 St", "", "2016-05-17T08:05-04:00", 0, 0.0f, 0.0f)));
}

void PopulateTransitManeuverList_1_stop_count_4(std::list<Maneuver>& maneuvers,
                                                const std::string& country_code,
                                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransit, {}, {},
                   {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "R", "Broadway Local",
                      "FOREST HILLS - 71 AV", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulateTransitManeuverList_1_stop_count_8(std::list<Maneuver>& maneuvers,
                                                const std::string& country_code,
                                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransit, {}, {},
                   {}, "", 7.685000, 1350, 175, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorthWest, 307, 29, 10, 18, 11, 108, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0,
                   0, 5534, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-m", 0, 134164, "M",
                      "QNS BLVD-6th AVE/ Myrtle Local", "FOREST HILLS - 71 AV", 16737049, 0,
                      "Trains operate between Middle Village-Metropolitan Avenue, Queens and "
                      "Myrtle Avenue, Brooklyn at all times. Service is extended weekdays (except "
                      "late nights) Continental Ave, Queens, All trains provide local service.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru3006q-23st<d18n",
                                       "23 St", "2016-05-17T08:32-04:00", "", 0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru0jt7k-14st<d19n", "14 St",
                             "2016-05-17T08:30-04:00", "2016-05-17T08:30-04:00", 0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsp47p6-w4st<d20n", "W 4 St",
                             "2016-05-17T08:29-04:00", "2016-05-17T08:29-04:00", 0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation,
                             "s-dr5rsjvd53-broadway~lafayettest<d21n", "Broadway-Lafayette St",
                             "2016-05-17T08:26-04:00", "2016-05-17T08:26-04:00", 0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rskecru-essexst<m18n",
                                       "Essex St", "2016-05-17T08:23-04:00", "2016-05-17T08:23-04:00",
                                       0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rt49x7k-marcyav<m16n",
                                       "Marcy Av", "2016-05-17T08:16-04:00", "2016-05-17T08:16-04:00",
                                       0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rt4ky8n-hewesst<m14n",
                                       "Hewes St", "2016-05-17T08:15-04:00", "2016-05-17T08:15-04:00",
                                       0, 0.0f, 0.0f)));

  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rt3cjhx-lorimerst<m13n",
                             "Lorimer St", "2016-05-17T08:13-04:00", "2016-05-17T08:13-04:00", 0,
                             0.0f, 0.0f)));

  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rt3m8ny-flushingav<m12n",
                             "Flushing Av", "", "2016-05-17T08:11-04:00", 0, 0.0f, 0.0f)));
}

void PopulateTransitTransferManeuverList_0_no_name(std::list<Maneuver>& maneuvers,
                                                   const std::string& country_code,
                                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransitTransfer,
                   {}, {}, {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "", "", "", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulateTransitTransferManeuverList_0(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransitTransfer,
                   {}, {}, {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "R", "Broadway Local",
                      "", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulateTransitTransferManeuverList_1(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransitTransfer,
                   {}, {}, {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "R", "Broadway Local",
                      "FOREST HILLS - 71 AV", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulateTransitRemainOnManeuverList_0_no_name(std::list<Maneuver>& maneuvers,
                                                   const std::string& country_code,
                                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransitRemainOn,
                   {}, {}, {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "", "", "", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulateTransitRemainOnManeuverList_0(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransitRemainOn,
                   {}, {}, {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "R", "Broadway Local",
                      "", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulateTransitRemainOnManeuverList_1(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code, DirectionsLeg_Maneuver_Type_kTransitRemainOn,
                   {}, {}, {}, "", 2.180000, 362, 164, Maneuver::RelativeDirection::KReverse,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 16, 7, 3, 7, 4, 11, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0, 1570, 0);

  PopulateTransitInfo(maneuver.mutable_transit_info(), "r-dr5r-r", 0, 84452, "R", "Broadway Local",
                      "FOREST HILLS - 71 AV", 16567306, 0,
                      "Trains operate local between Forest Hills-71 Av, Queens, and 95 St/4 Av, "
                      "Brooklyn, at all times except late nights. During late nights, trains "
                      "operate only in Brooklyn between 36 St and 95 St/4 Av.",
                      "o-dr5r-nyct", "MTA New York City Transit", "http://web.mta.info/");

  // Insert the transit stops in reverse order (end to begin of line)
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru65x7v-34st~heraldsq<r17n",
                             "34 St - Herald Sq", "2016-03-29T08:08-04:00", "", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru37pdw-28st<r18n", "28 St",
                             "2016-03-29T08:07-04:00", "2016-03-29T08:07-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5ru2dx73-23st<r19n", "23 St",
                             "2016-03-29T08:06-04:00", "2016-03-29T08:06-04:00", 1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(std::move(
      GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsr9wyg-14st~unionsq<r20n",
                             "14 St - Union Sq", "2016-03-29T08:04-04:00", "2016-03-29T08:04-04:00",
                             1, 0.0f, 0.0f)));
  maneuver.InsertTransitStop(
      std::move(GetTransitPlatformInfo(TransitPlatformInfo_Type_kStation, "s-dr5rsq8pqg-8st~nyu<r21n",
                                       "8 St - NYU", "", "2016-03-29T08:02-04:00", 1, 0.0f, 0.0f)));
}

void PopulatePostTransitConnectionDestinationManeuverList_0(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulatePostTransitConnectionDestinationManeuverList_1(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_2(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulatePostTransitConnectionDestinationManeuverList_3(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_4(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {{"6th Avenue", 0}},
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kTransit); // So it will just say Head
}

void PopulatePostTransitConnectionDestinationManeuverList_5(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulatePostTransitConnectionDestinationManeuverList_6(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_7(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulatePostTransitConnectionDestinationManeuverList_8(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_9(std::list<Maneuver>& maneuvers,
                                                            const std::string& country_code,
                                                            const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {{"6th Avenue", 0}},
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kDrive);
}

void PopulatePostTransitConnectionDestinationManeuverList_10(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulatePostTransitConnectionDestinationManeuverList_11(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_12(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulatePostTransitConnectionDestinationManeuverList_13(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_14(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {{"6th Avenue", 0}},
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kPedestrian);
}

void PopulatePostTransitConnectionDestinationManeuverList_15(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulatePostTransitConnectionDestinationManeuverList_16(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {}, {}, {}, "",
                   0.088000, 66, 2, Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_17(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulatePostTransitConnectionDestinationManeuverList_18(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination,
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
  maneuver.set_include_verbal_pre_transition_length(true);
}

void PopulatePostTransitConnectionDestinationManeuverList_19(std::list<Maneuver>& maneuvers,
                                                             const std::string& country_code,
                                                             const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination, {{"6th Avenue", 0}},
                   {{"6th Avenue", 0}, {"Avenue of the Americas", 0}}, {}, "", 0.088000, 66, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   DirectionsLeg_Maneuver_CardinalDirection_kSouthWest, 210, 211, 8, 11, 13, 17, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 0, 0, 0, 1, 0, "", "", "", "", 0, 0, 0, 0,
                   62, 0);
  maneuver.set_travel_mode(TravelMode::kBicycle);
}

void PopulateVerbalMultiCueManeuverList_0(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"North Plum Street", 0}}, {}, {}, "", 0.074000, 19, 270,
                   Maneuver::RelativeDirection::kLeft,
                   DirectionsLeg_Maneuver_CardinalDirection_kNorth, 352, 352, 2, 3, 2, 3, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 1, "", "", "", "", 0, 0, 0, 0, 4, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code, DirectionsLeg_Maneuver_Type_kLeft,
                   {{"East Fulton Street", 0}}, {}, {}, "", 0.120478, 29, 269,
                   Maneuver::RelativeDirection::kLeft, DirectionsLeg_Maneuver_CardinalDirection_kWest,
                   261, 263, 3, 5, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, {}, {}, {}, {}, 0, 1, 0, 0, 1, 1,
                   "", "", "", "", 0, 0, 0, 0, 12, 0);
}

void SetExpectedManeuverInstructions(std::list<Maneuver>& expected_maneuvers,
                                     const string& instruction,
                                     const string& verbal_succinct_transition_instruction,
                                     const string& verbal_transition_alert_instruction,
                                     const string& verbal_pre_transition_instruction,
                                     const string& verbal_post_transition_instruction,
                                     const string& depart_instruction = "",
                                     const string& verbal_depart_instruction = "",
                                     const string& arrive_instruction = "",
                                     const string& verbal_arrive_instruction = "") {
  Maneuver& maneuver = expected_maneuvers.back();
  maneuver.set_instruction(instruction);
  maneuver.set_verbal_succinct_transition_instruction(verbal_succinct_transition_instruction);
  maneuver.set_verbal_transition_alert_instruction(verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(verbal_post_transition_instruction);
  maneuver.set_depart_instruction(depart_instruction);
  maneuver.set_verbal_depart_instruction(verbal_depart_instruction);
  maneuver.set_arrive_instruction(arrive_instruction);
  maneuver.set_verbal_arrive_instruction(verbal_arrive_instruction);
}

void SetExpectedPreviousManeuverInstructions(std::list<Maneuver>& expected_maneuvers,
                                             const string& instruction,
                                             const string& verbal_succinct_transition_instruction,
                                             const string& verbal_transition_alert_instruction,
                                             const string& verbal_pre_transition_instruction,
                                             const string& verbal_post_transition_instruction) {
  Maneuver& maneuver = expected_maneuvers.front();
  maneuver.set_instruction(instruction);
  maneuver.set_verbal_succinct_transition_instruction(verbal_succinct_transition_instruction);
  maneuver.set_verbal_transition_alert_instruction(verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(verbal_post_transition_instruction);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head east.", "Head east.", "", "Head east.",
                                  "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head east.", "Head east for a half mile.", "",
                                  "Head east for a half mile.", "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southwest on 5th Avenue.",
                                  "Head southwest.", "", "Head southwest on 5th Avenue.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southwest on 5th Avenue.",
                                  "Head southwest for 700 feet.", "",
                                  "Head southwest on 5th Avenue for 700 feet.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Head south on North Prince Street/US 222/PA 272. Continue on US 222/PA 272.", "Head south.",
      "", "Head south on North Prince Street, U.S. 2 22.",
      "Continue on U.S. 2 22, Pennsylvania 2 72 for 3 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_5_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive east.", "Drive east.", "", "Drive east.",
                                  "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_6_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive east.", "Drive east for a half mile.",
                                  "", "Drive east for a half mile.", "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_7_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive southwest on 5th Avenue.",
                                  "Drive southwest.", "", "Drive southwest on 5th Avenue.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_8_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive southwest on 5th Avenue.",
                                  "Drive southwest for 700 feet.", "",
                                  "Drive southwest on 5th Avenue for 700 feet.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_9_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_9(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_9(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Drive south on North Prince Street/US 222/PA 272. Continue on US 222/PA 272.", "Drive south.",
      "", "Drive south on North Prince Street, U.S. 2 22.",
      "Continue on U.S. 2 22, Pennsylvania 2 72 for 3 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_10_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_10(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_10(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk east.", "Walk east.", "", "Walk east.",
                                  "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_11_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_11(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_11(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk east.", "Walk east for a half mile.", "",
                                  "Walk east for a half mile.", "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_12_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_12(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_12(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southwest on 5th Avenue.",
                                  "Walk southwest.", "", "Walk southwest on 5th Avenue.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_13_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_13(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_13(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southwest on 5th Avenue.",
                                  "Walk southwest for 700 feet.", "",
                                  "Walk southwest on 5th Avenue for 700 feet.",
                                  "Continue for 700 feet.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_13_unnamed_walkway_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_13_unnamed_walkway(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_13_unnamed_walkway(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southwest on the walkway.",
                                  "Walk southwest for 200 feet.", "",
                                  "Walk southwest on the walkway for 200 feet.",
                                  "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_13_pedestrian_crossing_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_13_pedestrian_crossing(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_13_pedestrian_crossing(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southwest on the crosswalk.",
                                  "Walk southwest for 200 feet.", "",
                                  "Walk southwest on the crosswalk for 200 feet.",
                                  "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_14_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_14(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_14(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Walk south on North Prince Street/US 222/PA 272. Continue on US 222/PA 272.", "Walk south.",
      "", "Walk south on North Prince Street, U.S. 2 22.",
      "Continue on U.S. 2 22, Pennsylvania 2 72 for 3 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_15_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_15(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_15(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike east.", "Bike east.", "", "Bike east.",
                                  "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_16_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_16(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_16(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike east.", "Bike east for a half mile.", "",
                                  "Bike east for a half mile.", "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_17_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_17(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_17(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike southwest on 5th Avenue.",
                                  "Bike southwest.", "", "Bike southwest on 5th Avenue.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_18_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_18(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_18(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike southwest on 5th Avenue.",
                                  "Bike southwest for 700 feet.", "",
                                  "Bike southwest on 5th Avenue for 700 feet.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_18_unnamed_cycleway_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_18_unnamed_cycleway(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_18_unnamed_cycleway(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike east on the cycleway.",
                                  "Bike east for 1.5 miles.", "",
                                  "Bike east on the cycleway for 1.5 miles.",
                                  "Continue for 1.5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_18_unnamed_mountain_bike_trail_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_18_unnamed_mountain_bike_trail(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_18_unnamed_mountain_bike_trail(expected_maneuvers, country_code,
                                                           state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike west on the mountain bike trail.",
                                  "Bike west for 700 feet.", "",
                                  "Bike west on the mountain bike trail for 700 feet.",
                                  "Continue for 700 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_19_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_19(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_19(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Bike south on North Prince Street/US 222/PA 272. Continue on US 222/PA 272.", "Bike south.",
      "", "Bike south on North Prince Street, U.S. 2 22.",
      "Continue on U.S. 2 22, Pennsylvania 2 72 for 3 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_0_kilometers_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head east.", "Head east.", "", "Head east.",
                                  "Continue for 800 meters.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_1_kilometers_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head east.", "Head east for 800 meters.", "",
                                  "Head east for 800 meters.", "Continue for 800 meters.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_2_kilometers_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southwest on 5th Avenue.",
                                  "Head southwest.", "", "Head southwest on 5th Avenue.",
                                  "Continue for 200 meters.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_3_kilometers_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southwest on 5th Avenue.",
                                  "Head southwest for 200 meters.", "",
                                  "Head southwest on 5th Avenue for 200 meters.",
                                  "Continue for 200 meters.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

TEST(NarrativeBuilder, TestBuildStartInstructions_4_kilometers_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Head south on North Prince Street/US 222/PA 272. Continue on US 222/PA 272.", "Head south.",
      "", "Head south on North Prince Street, U.S. 2 22.",
      "Continue on U.S. 2 22, Pennsylvania 2 72 for 5 kilometers.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 0 "You have arrived at your destination."
// 0 "You will arrive at your destination."
// 0 "You have arrived at your destination."
TEST(NarrativeBuilder, TestBuildDestinationInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "You have arrived at your destination.", "",
                                  "You will arrive at your destination.",
                                  "You have arrived at your destination.", "");

  // Add location info to trip path
  TripLeg path;
  // origin
  path.add_location();
  // destination
  path.add_location();

  EnhancedTripLeg etp(path);
  TryBuild(options, maneuvers, expected_maneuvers, &etp);
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
// 1 "You will arrive at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
// 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
TEST(NarrativeBuilder, TestBuildDestinationInstructions_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "You have arrived at 3206 Powelton Avenue.", "",
                                  "You will arrive at 32 o6 Powelton Avenue.",
                                  "You have arrived at 32 o6 Powelton Avenue.", "");

  // Add location info to trip path
  TripLeg path;
  Location* location;
  // origin
  path.add_location();
  // destination
  location = path.add_location();
  location->set_street("3206 Powelton Avenue");

  EnhancedTripLeg etp(path);
  TryBuild(options, maneuvers, expected_maneuvers, &etp);
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 2 "Your destination is on the <SOS>."
// 2 "Your destination will be on the <SOS>."
// 2 "Your destination is on the <SOS>."
TEST(NarrativeBuilder, TestBuildDestinationInstructions_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Your destination is on the right.", "",
                                  "Your destination will be on the right.",
                                  "Your destination is on the right.", "");

  // Add location info to trip path
  TripLeg path;
  Location* location;
  // origin
  path.add_location();
  // destination
  location = path.add_location();
  location->set_side_of_street(Location::kRight);

  EnhancedTripLeg etp(path);
  TryBuild(options, maneuvers, expected_maneuvers, &etp);
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."
// 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> will be on the <SOS>"
// 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."
TEST(NarrativeBuilder, TestBuildDestinationInstructions_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Lancaster Brewing Company is on the left.", "",
                                  "Lancaster Brewing Company will be on the left.",
                                  "Lancaster Brewing Company is on the left.", "");

  // Add location info to trip path
  TripLeg path;
  Location* location;
  // origin
  path.add_location();
  // destination
  location = path.add_location();
  location->set_name("Lancaster Brewing Company");
  location->set_side_of_street(Location::kLeft);

  EnhancedTripLeg etp(path);
  TryBuild(options, maneuvers, expected_maneuvers, &etp);
}

///////////////////////////////////////////////////////////////////////////////
// FormBecomesInstruction
// "0": "<PREV_STREET_NAMES> becomes <STREET_NAMES>."
// no verbal alert
// "0": "<PREV_STREET_NAMES> becomes <STREET_NAMES>."
TEST(NarrativeBuilder, TestBuildBecomesInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "VA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBecomesManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBecomesManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(expected_maneuvers, "Bear right onto Vine Street.",
                                          "Bear right.", "Bear right onto Vine Street.",
                                          "Bear right onto Vine Street.",
                                          "Continue for a quarter mile.");
  SetExpectedManeuverInstructions(expected_maneuvers, "Vine Street becomes Middletown Road.", "", "",
                                  "Vine Street becomes Middletown Road.", "Continue for 1 mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildContinueInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateContinueManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateContinueManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Continue.", "", "Continue.", "Continue.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildContinueInstructions_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateContinueManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateContinueManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Continue.", "", "Continue.",
                                  "Continue for 300 feet.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildContinueInstructions_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateContinueManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateContinueManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Continue on 10th Avenue.", "",
                                  "Continue on 10th Avenue.", "Continue on 10th Avenue.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildContinueInstructions_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateContinueManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateContinueManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Continue on 10th Avenue.", "",
                                  "Continue on 10th Avenue.",
                                  "Continue on 10th Avenue for a quarter mile.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 0 "Turn <RELATIVE_DIRECTION>."
// 0 "Turn <RELATIVE_DIRECTION>."
// 0 "Turn <RELATIVE_DIRECTION>."
TEST(NarrativeBuilder, TestBuildTurnInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Turn left.", "Turn left.", "Turn left.",
                                  "Turn left.", "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), false);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildTurnInstructions_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Turn left onto Middletown Road.", "Turn left.",
                                  "Turn left onto Middletown Road.",
                                  "Turn left onto Middletown Road.", "Continue for 1 mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), false);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildTurnInstructions_1_miles_cs_CZ) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("cs-CZ");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Odbočte vlevo na Middletown Road.",
                                  "Odbočte vlevo.", "Odbočte vlevo na Middletown Road.",
                                  "Odbočte vlevo na Middletown Road.", "Pokračujte 1 míli.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildTurnInstructions_1_miles_de_DE) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("de-DE");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Links auf Middletown Road abbiegen.",
                                  "Links abbiegen.", "Links auf Middletown Road abbiegen.",
                                  "Links auf Middletown Road abbiegen.",
                                  "eine Meile weiter der Route folgen.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildTurnInstructions_1_miles_it_IT) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("it-IT");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Svolta a sinistra su Middletown Road.",
                                  "Svolta a sinistra.", "Svolta a sinistra su Middletown Road.",
                                  "Svolta a sinistra su Middletown Road.", "Continua per 1 miglio.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 2 "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// 2 "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(1)>."
// 2 "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildTurnInstructions_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn left onto North Bond Street/US 1 Business/MD 924. Continue on MD 924.", "Turn left.",
      "Turn left onto North Bond Street.", "Turn left onto North Bond Street, U.S. 1 Business.",
      "Continue on Maryland 9 24 for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), false);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 3 "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
// 3 "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES(1)>."
// 3 "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildTurnInstructions_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "VA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Turn right onto Sunstone Drive.",
      "Turn right. Then Turn right to stay on Sunstone Drive.", "Turn right onto Sunstone Drive.",
      "Turn right onto Sunstone Drive. Then Turn right to stay on Sunstone Drive.",
      "Continue for 300 feet.");
  SetExpectedManeuverInstructions(expected_maneuvers, "Turn right to stay on Sunstone Drive.",
                                  "Turn right.", "Turn right to stay on Sunstone Drive.",
                                  "Turn right to stay on Sunstone Drive.", "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 0 "Make a sharp <RELATIVE_DIRECTION>."
// 0 "Make a sharp <RELATIVE_DIRECTION>."
// 0 "Make a sharp <RELATIVE_DIRECTION>."
TEST(NarrativeBuilder, TestBuildSharpInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateSharpManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateSharpManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Make a sharp left.", "Make a sharp left.",
                                  "Make a sharp left.", "Make a sharp left.",
                                  "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 1 "Make a sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Make a sharp <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Make a sharp <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildSharpInstructions_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateSharpManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateSharpManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Make a sharp right onto Flatbush Avenue.",
                                  "Make a sharp right.", "Make a sharp right onto Flatbush Avenue.",
                                  "Make a sharp right onto Flatbush Avenue.",
                                  "Continue for 600 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 2 "Make a sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// 2 "Make a sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(1)>."
// 2 "Make a sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildSharpInstructions_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateSharpManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateSharpManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a sharp left onto North Bond Street/US 1 Business/MD 924. Continue on MD 924.",
      "Make a sharp left.", "Make a sharp left onto North Bond Street.",
      "Make a sharp left onto North Bond Street, U.S. 1 Business.",
      "Continue on Maryland 9 24 for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 3 "Make a sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
// 3 "Make a sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES(1)>."
// 3 "Make a sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES(2)>."
// todo: check why the test case has been disabled in previous test suite and why it doesn't pass now
TEST(NarrativeBuilder, DISABLED_TestBuildSharpInstructions_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "VA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateSharpManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateSharpManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Turn right onto Sunstone Drive.", "Turn right.",
      "Turn right onto Sunstone Drive.",
      "Turn right onto Sunstone Drive. Then Turn right to stay on Sunstone Drive.",
      "Continue for 300 feet.");
  SetExpectedManeuverInstructions(expected_maneuvers, "Make a sharp right to stay on Sunstone Drive.",
                                  "", "Make a sharp right to stay on Sunstone Drive.",
                                  "Make a sharp right to stay on Sunstone Drive.",
                                  "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 0 "Bear <RELATIVE_DIRECTION>."
// 0 "Bear <RELATIVE_DIRECTION>."
// 0 "Bear <RELATIVE_DIRECTION>."
TEST(NarrativeBuilder, TestBuildBearInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bear right.", "Bear right.", "Bear right.",
                                  "Bear right.", "Continue for 60 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 1 "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildBearInstructions_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bear left onto Arlen Road.", "Bear left.",
                                  "Bear left onto Arlen Road.", "Bear left onto Arlen Road.",
                                  "Continue for 400 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 2 "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// 2 "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(1)>."
// 2 "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildBearInstructions_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Bear right onto Belair Road/US 1 Business. Continue on US 1 Business.",
      "Bear right.", "Bear right onto Belair Road.", "Bear right onto Belair Road, U.S. 1 Business.",
      "Continue on U.S. 1 Business for 2 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 3 "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
// 3 "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES(1)>."
// 3 "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildBearInstructions_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "VA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Exit the roundabout onto Catoctin Mountain Highway/US 15. Continue on US 15.",
      "Exit the roundabout.", "", "Exit the roundabout onto Catoctin Mountain Highway, U.S. 15.",
      "Continue on U.S. 15 for 11 miles.");
  SetExpectedManeuverInstructions(expected_maneuvers, "Bear left to stay on US 15 South.",
                                  "Bear left.", "Bear left to stay on U.S. 15 South.",
                                  "Bear left to stay on U.S. 15 South.", "Continue for 3 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 0 "Make a <RELATIVE_DIRECTION> U-turn."
// 0 "Make a <RELATIVE_DIRECTION> U-turn."
// 0 "Make a <RELATIVE_DIRECTION> U-turn."
TEST(NarrativeBuilder, TestBuildUturnInstructions_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Make a left U-turn.", "Make a left U-turn.",
                                  "Make a left U-turn.", "Make a left U-turn.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 1 "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>."
// 1 "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES(1)>."
// 1 "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildUturnInstructions_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Make a right U-turn onto Bunker Hill Road.",
                                  "Make a right U-turn.",
                                  "Make a right U-turn onto Bunker Hill Road.",
                                  "Make a right U-turn onto Bunker Hill Road.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 2 "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>."
// 2 "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES(1)>."
// 2 "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildUturnInstructions_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(expected_maneuvers, "Turn right onto Bunker Hill Road.",
                                          "Turn right.", "Turn right onto Bunker Hill Road.",
                                          "Turn right onto Bunker Hill Road.",
                                          "Continue for 900 feet.");
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Make a left U-turn to stay on Bunker Hill Road.",
                                  "Make a left U-turn.",
                                  "Make a left U-turn to stay on Bunker Hill Road.",
                                  "Make a left U-turn to stay on Bunker Hill Road.",
                                  "Continue for 900 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(1)>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildUturnInstructions_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Make a left U-turn at Devonshire Road.",
                                  "Make a left U-turn.", "Make a left U-turn at Devonshire Road.",
                                  "Make a left U-turn at Devonshire Road.", "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 4 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(1)>."
// 4 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(2)> onto <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildUturnInstructions_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Make a left U-turn at Devonshire Road onto Jonestown Road/US 22.",
      "Make a left U-turn.", "Make a left U-turn at Devonshire Road.",
      "Make a left U-turn at Devonshire Road onto Jonestown Road, U.S. 22.",
      "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 5 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(1)>."
// 5 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(2)> to stay on <STREET_NAMES(2)>."
TEST(NarrativeBuilder, TestBuildUturnInstructions_5_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Drive northeast on Jonestown Road/US 22.",
      "Drive northeast. Then Make a left U-turn at Devonshire Road.", "",
      "Drive northeast on Jonestown Road, U.S. 22. Then Make a left U-turn at Devonshire Road.",
      "Continue for 200 feet.");
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Make a left U-turn at Devonshire Road to stay on Jonestown Road/US 22.",
      "Make a left U-turn.", "Make a left U-turn at Devonshire Road.",
      "Make a left U-turn at Devonshire Road to stay on Jonestown Road, U.S. 22.",
      "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 0 "Stay straight to take the ramp."
// 0 "Stay straight to take the ramp."
// 0 "Stay straight to take the ramp."
TEST(NarrativeBuilder, TestBuildRampStraight_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Stay straight to take the ramp.", "",
                                  "Stay straight to take the ramp.",
                                  "Stay straight to take the ramp.", "Continue for 1.5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 1 "Stay straight to take the <BRANCH_SIGN> ramp."
// 1 "Stay straight to take the <BRANCH_SIGN> ramp."
// 1 "Stay straight to take the <BRANCH_SIGN> ramp."
TEST(NarrativeBuilder, TestBuildRampStraight_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Stay straight to take the US 322 East ramp.",
                                  "", "Stay straight to take the U.S. 3 22 East ramp.",
                                  "Stay straight to take the U.S. 3 22 East ramp.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
// 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
// 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
TEST(NarrativeBuilder, TestBuildRampStraight_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Stay straight to take the ramp toward Hershey.", "",
                                  "Stay straight to take the ramp toward Hershey.",
                                  "Stay straight to take the ramp toward Hershey.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 3 "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
// 1 "Stay straight to take the <BRANCH_SIGN> ramp"
// 3 "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
TEST(NarrativeBuilder, TestBuildRampStraight_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Stay straight to take the US 322 East/US 422 East/US 522 East/US 622 East ramp toward "
      "Hershey/Palmdale/Palmyra/Campbelltown.",
      "", "Stay straight to take the U.S. 3 22 East ramp.",
      "Stay straight to take the U.S. 3 22 East, U.S. 4 22 East ramp toward Hershey, Palmdale.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 4 "Stay straight to take the <NAME_SIGN> ramp."
// 3 "Stay straight to take the <NAME_SIGN> ramp."
// 4 "Stay straight to take the <NAME_SIGN> ramp."
TEST(NarrativeBuilder, TestBuildRampStraight_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Stay straight to take the Gettysburg Pike ramp.", "",
                                  "Stay straight to take the Gettysburg Pike ramp.",
                                  "Stay straight to take the Gettysburg Pike ramp.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
// "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
// "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
TEST(NarrativeBuilder, TestBuildRamp_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the ramp on the right.", "",
                                  "Take the ramp on the right.", "Take the ramp on the right.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
TEST(NarrativeBuilder, TestBuildRamp_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the I 95 ramp on the right.", "",
                                  "Take the Interstate 95 ramp on the right.",
                                  "Take the Interstate 95 ramp on the right.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildRamp_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the ramp on the left toward JFK.", "",
                                  "Take the ramp on the left toward JFK.",
                                  "Take the ramp on the left toward JFK.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>",
// "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildRamp_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the South Conduit Avenue ramp on the left toward JFK.", "",
                                  "Take the South Conduit Avenue ramp on the left.",
                                  "Take the South Conduit Avenue ramp on the left toward JFK.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
TEST(NarrativeBuilder, TestBuildRamp_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the Gettysburg Pike ramp on the right.",
                                  "", "Take the Gettysburg Pike ramp on the right.",
                                  "Take the Gettysburg Pike ramp on the right.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
// "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
// "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
TEST(NarrativeBuilder, TestBuildRamp_5_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Turn right to take the ramp.", "",
                                  "Turn right to take the ramp.", "Turn right to take the ramp.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
TEST(NarrativeBuilder, TestBuildRamp_6_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Turn left to take the PA 283 West ramp.", "",
                                  "Turn left to take the Pennsylvania 2 83 West ramp.",
                                  "Turn left to take the Pennsylvania 2 83 West ramp.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
// "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
// "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildRamp_7_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn left to take the ramp toward Harrisburg/Harrisburg International Airport.", "",
      "Turn left to take the ramp toward Harrisburg.",
      "Turn left to take the ramp toward Harrisburg, Harrisburg International Airport.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
// "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildRamp_8_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn left to take the PA 283 West ramp toward Harrisburg/Harrisburg International Airport.",
      "", "Turn left to take the Pennsylvania 2 83 West ramp.",
      "Turn left to take the Pennsylvania 2 83 West ramp toward Harrisburg, Harrisburg "
      "International Airport.",
      "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
// "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
// "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
TEST(NarrativeBuilder, TestBuildRamp_9_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_9(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_9(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Turn right to take the Gettysburg Pike ramp.",
                                  "", "Turn right to take the Gettysburg Pike ramp.",
                                  "Turn right to take the Gettysburg Pike ramp.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "10": "Take the ramp.",
// "10": "Take the ramp.",
// "10": "Take the ramp.",
TEST(NarrativeBuilder, TestBuildRamp_10_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_10(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_10(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the ramp.", "", "Take the ramp.",
                                  "Take the ramp.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "11": "Take the <BRANCH_SIGN> ramp.",
// "11": "Take the <BRANCH_SIGN> ramp.",
// "11": "Take the <BRANCH_SIGN> ramp.",
TEST(NarrativeBuilder, TestBuildRamp_11_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_11(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_11(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the I 95 ramp.", "",
                                  "Take the Interstate 95 ramp.", "Take the Interstate 95 ramp.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "12": "Take the ramp toward <TOWARD_SIGN>.",
// "12": "Take the ramp toward <TOWARD_SIGN>.",
// "12": "Take the ramp toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildRamp_12_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_12(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_12(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the ramp toward JFK.", "",
                                  "Take the ramp toward JFK.", "Take the ramp toward JFK.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "13": "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
// "10": "Take the <BRANCH_SIGN> ramp",
// "13": "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildRamp_13_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_13(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_13(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the South Conduit Avenue ramp on the left toward JFK.", "",
                                  "Take the South Conduit Avenue ramp on the left.",
                                  "Take the South Conduit Avenue ramp on the left toward JFK.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "14": "Take the <NAME_SIGN> ramp.",
// "14": "Take the <NAME_SIGN> ramp.",
// "14": "Take the <NAME_SIGN> ramp.",
TEST(NarrativeBuilder, TestBuildRamp_14_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_14(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_14(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the Gettysburg Pike ramp.", "",
                                  "Take the Gettysburg Pike ramp.", "Take the Gettysburg Pike ramp.",
                                  "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "0": "Take the exit on the <RELATIVE_DIRECTION>.",
// "0": "Take the exit on the <RELATIVE_DIRECTION>.",
// "0": "Take the exit on the <RELATIVE_DIRECTION>.",
TEST(NarrativeBuilder, TestBuildExit_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the exit on the right.", "",
                                  "Take the exit on the right.", "Take the exit on the right.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
TEST(NarrativeBuilder, TestBuildExit_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take exit 67 B-A on the right.", "",
                                  "Take exit 67 B-A on the right.", "Take exit 67 B-A on the right.",
                                  "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
TEST(NarrativeBuilder, TestBuildExit_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the US 322 West exit on the right.", "",
                                  "Take the U.S. 3 22 West exit on the right.",
                                  "Take the U.S. 3 22 West exit on the right.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "3": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "3": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take exit 67 B-A on the right onto US 322 West.", "",
                                  "Take exit 67 B-A on the right.",
                                  "Take exit 67 B-A on the right onto U.S. 3 22 West.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the exit on the right toward Lewistown.",
                                  "", "Take the exit on the right toward Lewistown.",
                                  "Take the exit on the right toward Lewistown.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "5": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "5": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_5_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take exit 67 B-A on the right toward Lewistown.", "",
                                  "Take exit 67 B-A on the right.",
                                  "Take exit 67 B-A on the right toward Lewistown.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "6": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "6": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_6_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the US 322 West exit on the right toward Lewistown.", "",
                                  "Take the U.S. 3 22 West exit on the right.",
                                  "Take the U.S. 3 22 West exit on the right toward Lewistown.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "7": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward
// <TOWARD_SIGN>.", "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.", "7": "Take exit
// <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_7_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take exit 67 B-A on the right onto US 322 West toward Lewistown/State College.", "",
      "Take exit 67 B-A on the right.",
      "Take exit 67 B-A on the right onto U.S. 3 22 West toward Lewistown, State College.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.",
TEST(NarrativeBuilder, TestBuildExit_8_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the White Marsh Boulevard exit on the left.", "",
                                  "Take the White Marsh Boulevard exit on the left.",
                                  "Take the White Marsh Boulevard exit on the left.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "10": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "10": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_10_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_10(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_10(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take the White Marsh Boulevard exit on the left onto MD 43 East.", "",
      "Take the Maryland 43 East exit on the left.",
      "Take the White Marsh Boulevard exit on the left onto Maryland 43 East.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "12": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "12": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_12_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_12(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_12(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take the White Marsh Boulevard exit on the left toward White Marsh.", "",
      "Take the exit on the left toward White Marsh.",
      "Take the White Marsh Boulevard exit on the left toward White Marsh.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "14": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward
// <TOWARD_SIGN>." "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.", "14": "Take the
// <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
TEST(NarrativeBuilder, TestBuildExit_14_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_14(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_14(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take the White Marsh Boulevard exit on the left onto MD 43 East toward White Marsh.", "",
      "Take the Maryland 43 East exit on the left.",
      "Take the White Marsh Boulevard exit on the left onto Maryland 43 East toward White Marsh.",
      "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "15": "Take the exit.",
// "15": "Take the exit.",
// "15": "Take the exit.",
TEST(NarrativeBuilder, TestBuildExit_15_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_15(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_15(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the exit.", "", "Take the exit.",
                                  "Take the exit.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "16": "Take exit <NUMBER_SIGN>.",
// "16": "Take exit <NUMBER_SIGN>.",
// "16": "Take exit <NUMBER_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_16_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_16(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_16(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take exit 67 B-A.", "", "Take exit 67 B-A.",
                                  "Take exit 67 B-A.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "17": "Take the <BRANCH_SIGN> exit.",
// "17": "Take the <BRANCH_SIGN> exit.",
// "17": "Take the <BRANCH_SIGN> exit.",
TEST(NarrativeBuilder, TestBuildExit_17_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_17(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_17(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the US 322 West exit.", "",
                                  "Take the U.S. 3 22 West exit.", "Take the U.S. 3 22 West exit.",
                                  "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "18": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN>.",
// "15": "Take exit <NUMBER_SIGN>.",
// "18": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_18_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_18(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_18(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take exit 67 B-A onto US 322 West.", "",
                                  "Take exit 67 B-A.", "Take exit 67 B-A onto U.S. 3 22 West.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "19": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "19": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "19": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_19_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_19(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_19(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the exit toward Lewistown.", "",
                                  "Take the exit toward Lewistown.",
                                  "Take the exit toward Lewistown.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "20": "Take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
// "16": "Take exit <NUMBER_SIGN>.",
// "20": "Take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_20_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_20(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_20(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take exit 67 B-A toward Lewistown.", "",
                                  "Take exit 67 B-A.", "Take exit 67 B-A toward Lewistown.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "21": "Take the <BRANCH_SIGN> exit toward <TOWARD_SIGN>.",
// "17": "Take the <BRANCH_SIGN> exit.",
// "21": "Take the <BRANCH_SIGN> exit toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_21_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_21(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_21(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the US 322 West exit toward Lewistown.",
                                  "", "Take the U.S. 3 22 West exit.",
                                  "Take the U.S. 3 22 West exit toward Lewistown.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "22": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
// "16": "Take exit <NUMBER_SIGN>.",
// "22": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_22_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_22(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_22(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take exit 67 B-A onto US 322 West toward Lewistown/State College.", "",
      "Take exit 67 B-A.", "Take exit 67 B-A onto U.S. 3 22 West toward Lewistown, State College.",
      "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "23": "Take the <NAME_SIGN> exit.",
// "23": "Take the <NAME_SIGN> exit.",
// "23": "Take the <NAME_SIGN> exit.",
TEST(NarrativeBuilder, TestBuildExit_23_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_23(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_23(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the White Marsh Boulevard exit.", "",
                                  "Take the White Marsh Boulevard exit.",
                                  "Take the White Marsh Boulevard exit.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "25": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN>.",
// "17": "Take the <BRANCH_SIGN> exit.",
// "25": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_25_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_25(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_25(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the White Marsh Boulevard exit onto MD 43 East.", "",
                                  "Take the Maryland 43 East exit.",
                                  "Take the White Marsh Boulevard exit onto Maryland 43 East.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "27": "Take the <NAME_SIGN> exit toward <TOWARD_SIGN>.",
// "19": "Take the exit toward <TOWARD_SIGN>.",
// "27": "Take the <NAME_SIGN> exit toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_27_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_27(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_27(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the White Marsh Boulevard exit toward White Marsh.", "",
                                  "Take the exit toward White Marsh.",
                                  "Take the White Marsh Boulevard exit toward White Marsh.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "29": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
// "17": "Take the <BRANCH_SIGN> exit.",
// "29": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildExit_29_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_29(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_29(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take the White Marsh Boulevard exit onto MD 43 East toward White Marsh.",
      "", "Take the Maryland 43 East exit.",
      "Take the White Marsh Boulevard exit onto Maryland 43 East toward White Marsh.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "0": "Keep <RELATIVE_DIRECTION> at the fork.",
// "0": "Keep <RELATIVE_DIRECTION> at the fork.",
// "0": "Keep <RELATIVE_DIRECTION> at the fork.",
TEST(NarrativeBuilder, TestBuildKeep_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Keep straight at the fork.", "",
                                  "Keep straight at the fork.", "Keep straight at the fork.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
TEST(NarrativeBuilder, TestBuildKeep_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Keep right to take exit 62.", "",
                                  "Keep right to take exit 62.", "Keep right to take exit 62.",
                                  "Continue for 9 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
TEST(NarrativeBuilder, TestBuildKeep_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Keep right to take I 895 South.", "",
                                  "Keep right to take Interstate 8 95 South.",
                                  "Keep right to take Interstate 8 95 South.",
                                  "Continue for 9 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>.",
TEST(NarrativeBuilder, TestBuildKeep_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Keep right to take exit 62 onto I 895 South.",
                                  "", "Keep right to take exit 62.",
                                  "Keep right to take exit 62 onto Interstate 8 95 South.",
                                  "Continue for 9 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildKeep_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Keep right toward Annapolis.", "",
                                  "Keep right toward Annapolis.", "Keep right toward Annapolis.",
                                  "Continue for 9 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildKeep_5_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Keep right to take exit 62 toward Annapolis.",
                                  "", "Keep right to take exit 62.",
                                  "Keep right to take exit 62 toward Annapolis.",
                                  "Continue for 9 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "6": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>.",
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
// "6": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildKeep_6_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep right to take I 895 South toward Annapolis.", "",
                                  "Keep right to take Interstate 8 95 South.",
                                  "Keep right to take Interstate 8 95 South toward Annapolis.",
                                  "Continue for 9 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "7": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward
// <TOWARD_SIGN>." "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.", "7": "Keep
// <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward <TOWARD_SIGN>."
TEST(NarrativeBuilder, TestBuildKeep_7_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Keep right to take exit 62 onto I 895 South toward Annapolis.", "",
      "Keep right to take exit 62.",
      "Keep right to take exit 62 onto Interstate 8 95 South toward Annapolis.",
      "Continue for 9 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
TEST(NarrativeBuilder, TestBuildKeepToStayOn_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "Merge.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.", "Continue for 15 miles.");
  SetExpectedManeuverInstructions(expected_maneuvers, "Keep left to stay on I 95 South.", "",
                                  "Keep left to stay on Interstate 95 South.",
                                  "Keep left to stay on Interstate 95 South.",
                                  "Continue for 5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>.",
TEST(NarrativeBuilder, TestBuildKeepToStayOn_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "Merge.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.", "Continue for 15 miles.");
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep left to take exit 62 to stay on I 95 South.", "",
                                  "Keep left to stay on Interstate 95 South.",
                                  "Keep left to take exit 62 to stay on Interstate 95 South.",
                                  "Continue for 5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "2": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "2": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>.",
TEST(NarrativeBuilder, TestBuildKeepToStayOn_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "Merge.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.", "Continue for 15 miles.");
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep left to stay on I 95 South toward Baltimore.", "",
                                  "Keep left to stay on Interstate 95 South.",
                                  "Keep left to stay on Interstate 95 South toward Baltimore.",
                                  "Continue for 5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward
// <TOWARD_SIGN>." "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.", "3": "Keep
// <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
TEST(NarrativeBuilder, TestBuildKeepToStayOn_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "Merge.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.", "Continue for 15 miles.");
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Keep left to take exit 62 to stay on I 95 South toward Baltimore.", "",
      "Keep left to stay on Interstate 95 South.",
      "Keep left to take exit 62 to stay on Interstate 95 South toward Baltimore.",
      "Continue for 5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
  VerifyToStayOn(maneuvers.back(), true);
}

///////////////////////////////////////////////////////////////////////////////
// FormMergeInstruction
// "0": "Merge.",
// No alert since prior maneuver is not > 2 km
// "0": "Merge.",
TEST(NarrativeBuilder, TestBuildMerge_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateMergeManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateMergeManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(expected_maneuvers,
                                          "Take the I 76 West exit toward Pittsburgh.", "",
                                          "Take the Interstate 76 West exit.",
                                          "Take the Interstate 76 West exit toward Pittsburgh.", "");
  SetExpectedManeuverInstructions(expected_maneuvers, "Merge.", "Merge.", "", "Merge.",
                                  "Continue for 5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormMergeInstruction
// "1": "Merge onto <STREET_NAMES>."
// No alert since prior maneuver is not > 2 km
// "1": "Merge onto <STREET_NAMES>."
TEST(NarrativeBuilder, TestBuildMerge_1_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateMergeManeuverList_1_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateMergeManeuverList_1_1(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(expected_maneuvers,
                                          "Take the I 76 West exit toward Pittsburgh.", "",
                                          "Take the Interstate 76 West exit.",
                                          "Take the Interstate 76 West exit toward Pittsburgh.", "");
  SetExpectedManeuverInstructions(expected_maneuvers, "Merge onto I 76 West/Pennsylvania Turnpike.",
                                  "Merge.", "",
                                  "Merge onto Interstate 76 West, Pennsylvania Turnpike.",
                                  "Continue for 5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormMergeInstruction
// "1": "Merge onto <STREET_NAMES>."
// "1": "Merge onto <STREET_NAMES>."
// "1": "Merge onto <STREET_NAMES>."
TEST(NarrativeBuilder, TestBuildMerge_1_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateMergeManeuverList_1_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateMergeManeuverList_1_2(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(expected_maneuvers,
                                          "Take the I 76 West exit toward Pittsburgh.", "",
                                          "Take the Interstate 76 West exit.",
                                          "Take the Interstate 76 West exit toward Pittsburgh.",
                                          "Continue for 1.5 miles.");
  SetExpectedManeuverInstructions(expected_maneuvers, "Merge onto I 76 West/Pennsylvania Turnpike.",
                                  "Merge.", "Merge onto Interstate 76 West.",
                                  "Merge onto Interstate 76 West, Pennsylvania Turnpike.",
                                  "Continue for 5 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterRoundaboutInstruction
// "0": "Enter the roundabout.",
// "0": "Enter the roundabout.",
// "0": "Enter the roundabout.",
TEST(NarrativeBuilder, TestBuildEnterRoundabout_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterRoundaboutManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterRoundaboutManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Enter the roundabout.",
                                  "Enter the roundabout.", "Enter the roundabout.",
                                  "Enter the roundabout.", "");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterRoundaboutInstruction
// "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."
// "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."
// "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."
TEST(NarrativeBuilder, TestBuildEnterRoundabout_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  uint32_t roundabout_exit_count = 1;
  const std::vector<std::string> kExpectedOrdinalValues = {"1st", "2nd", "3rd", "4th", "5th",
                                                           "6th", "7th", "8th", "9th", "10th"};

  for (auto& ordinal_value : kExpectedOrdinalValues) {
    // Configure maneuvers
    std::list<Maneuver> maneuvers;
    PopulateEnterRoundaboutManeuverList_1(maneuvers, country_code, state_code, roundabout_exit_count);

    // Configure expected maneuvers based on directions options
    std::list<Maneuver> expected_maneuvers;
    PopulateEnterRoundaboutManeuverList_1(expected_maneuvers, country_code, state_code,
                                          roundabout_exit_count++);
    SetExpectedManeuverInstructions(expected_maneuvers,
                                    "Enter the roundabout and take the " + ordinal_value + " exit.",
                                    "Enter the roundabout and take the " + ordinal_value + " exit.",
                                    "Enter the roundabout and take the " + ordinal_value + " exit.",
                                    "Enter the roundabout and take the " + ordinal_value + " exit.",
                                    "");

    TryBuild(options, maneuvers, expected_maneuvers);
  }
}

///////////////////////////////////////////////////////////////////////////////
// FormExitRoundaboutInstruction
// "0": "Exit the roundabout.",
// No verbal alert
// "0": "Exit the roundabout.",
TEST(NarrativeBuilder, TestBuildExitRoundabout_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitRoundaboutManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitRoundaboutManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Exit the roundabout.", "Exit the roundabout.",
                                  "", "Exit the roundabout.", "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitRoundaboutInstruction
// "1": "Exit the roundabout onto <STREET_NAMES>.",
// No verbal alert
// "1": "Exit the roundabout onto <STREET_NAMES>.",
TEST(NarrativeBuilder, TestBuildExitRoundabout_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitRoundaboutManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitRoundaboutManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Exit the roundabout onto Philadelphia Road/MD 7.",
                                  "Exit the roundabout.", "",
                                  "Exit the roundabout onto Philadelphia Road, Maryland 7.",
                                  "Continue for a half mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitRoundaboutInstruction
// "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// No verbal alert
// "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>.",
TEST(NarrativeBuilder, TestBuildExitRoundabout_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitRoundaboutManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitRoundaboutManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Exit the roundabout onto Catoctin Mountain Highway/US 15. Continue on US 15.",
      "Exit the roundabout.", "", "Exit the roundabout onto Catoctin Mountain Highway, U.S. 15.",
      "Continue on U.S. 15 for 11 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterFerryInstruction
// "0": "Take the Ferry.",
// "0": "Take the Ferry.",
// "0": "Take the Ferry.",
TEST(NarrativeBuilder, TestBuildEnterFerry_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterFerryManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterFerryManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the Ferry.", "", "Take the Ferry.",
                                  "Take the Ferry.", "Continue for 1 mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterFerryInstruction
// "1": "Take the <STREET_NAMES>.",
// "1": "Take the <STREET_NAMES>.",
// "1": "Take the <STREET_NAMES>.",
TEST(NarrativeBuilder, TestBuildEnterFerry_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterFerryManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterFerryManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the Millersburg FERRY.", "",
                                  "Take the Millersburg FERRY.", "Take the Millersburg FERRY.",
                                  "Continue for 1 mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterFerryInstruction
// "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
// "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
// "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
TEST(NarrativeBuilder, TestBuildEnterFerry_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterFerryManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterFerryManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the Bridgeport - Port Jefferson Ferry.",
                                  "", "Take the Bridgeport - Port Jefferson Ferry.",
                                  "Take the Bridgeport - Port Jefferson Ferry.",
                                  "Continue for 17 miles.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southeast.", "Head southeast.", "",
                                  "Head southeast.", "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southeast.",
                                  "Head southeast for 200 feet.", "", "Head southeast for 200 feet.",
                                  "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head west on Ferry Lane.", "Head west.", "",
                                  "Head west on Ferry Lane.", "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head west on Ferry Lane.",
                                  "Head west for a quarter mile.", "",
                                  "Head west on Ferry Lane for a quarter mile.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Head northeast on Cape May-Lewes Ferry Entrance/US 9. Continue on US 9.",
      "Head northeast.", "", "Head northeast on Cape May-Lewes Ferry Entrance, U.S. 9.",
      "Continue on U.S. 9 for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_5_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive southeast.", "Drive southeast.", "",
                                  "Drive southeast.", "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_6_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive southeast.",
                                  "Drive southeast for 200 feet.", "",
                                  "Drive southeast for 200 feet.", "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_7_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive west on Ferry Lane.", "Drive west.", "",
                                  "Drive west on Ferry Lane.", "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_8_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive west on Ferry Lane.",
                                  "Drive west for a quarter mile.", "",
                                  "Drive west on Ferry Lane for a quarter mile.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_9_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_9(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_9(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Drive northeast on Cape May-Lewes Ferry Entrance/US 9. Continue on US 9.",
      "Drive northeast.", "", "Drive northeast on Cape May-Lewes Ferry Entrance, U.S. 9.",
      "Continue on U.S. 9 for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_10_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_10(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_10(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southeast.", "Walk southeast.", "",
                                  "Walk southeast.", "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_11_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_11(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_11(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southeast.",
                                  "Walk southeast for 200 feet.", "", "Walk southeast for 200 feet.",
                                  "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_12_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_12(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_12(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk west on Ferry Lane.", "Walk west.", "",
                                  "Walk west on Ferry Lane.", "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_13_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_13(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_13(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk west on Ferry Lane.",
                                  "Walk west for a quarter mile.", "",
                                  "Walk west on Ferry Lane for a quarter mile.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_14_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_14(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_14(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Walk northeast on Cape May-Lewes Ferry Entrance/US 9. Continue on US 9.",
      "Walk northeast.", "", "Walk northeast on Cape May-Lewes Ferry Entrance, U.S. 9.",
      "Continue on U.S. 9 for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_15_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_15(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_15(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike southeast.", "Bike southeast.", "",
                                  "Bike southeast.", "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_16_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_16(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_16(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike southeast.",
                                  "Bike southeast for 200 feet.", "", "Bike southeast for 200 feet.",
                                  "Continue for 200 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_17_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_17(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_17(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike west on Ferry Lane.", "Bike west.", "",
                                  "Bike west on Ferry Lane.", "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_18_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_18(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_18(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike west on Ferry Lane.",
                                  "Bike west for a quarter mile.", "",
                                  "Bike west on Ferry Lane for a quarter mile.",
                                  "Continue for a quarter mile.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildExitFerry_19_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_19(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_19(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Bike northeast on Cape May-Lewes Ferry Entrance/US 9. Continue on US 9.",
      "Bike northeast.", "", "Bike northeast on Cape May-Lewes Ferry Entrance, U.S. 9.",
      "Continue on U.S. 9 for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionStartInstruction
// "0": "Enter the station.",
// No verbal alert
// "0": "Enter the station.",
TEST(NarrativeBuilder, TestBuildTransitConnectionStart_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionStartManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionStartManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Enter the station.", "", "",
                                  "Enter the station.", "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionStartInstruction
// "1": "Enter the <TRANSIT_STOP>."
// No verbal alert
// "1": "Enter the <TRANSIT_STOP>."
TEST(NarrativeBuilder, TestBuildTransitConnectionStart_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionStartManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionStartManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Enter the CALTRAIN - SAN FRANCISCO STATION.",
                                  "", "", "Enter the CALTRAIN - SAN FRANCISCO STATION.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionStartInstruction
// "2": "Enter the <TRANSIT_STOP> <STATION_LABEL>."
// No verbal alert
// "2": "Enter the <TRANSIT_STOP> <STATION_LABEL>."
TEST(NarrativeBuilder, TestBuildTransitConnectionStart_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionStartManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionStartManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Enter the 8 St - NYU Station.", "", "",
                                  "Enter the 8 St - NYU Station.", "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionTransferInstruction
// "0": "Transfer at the station.",
// No verbal alert
// "0": "Transfer at the station.",
TEST(NarrativeBuilder, TestBuildTransitConnectionTransfer_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionTransferManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionTransferManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Transfer at the station.", "", "",
                                  "Transfer at the station.", "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionTransferInstruction
// "1": "Transfer at the <TRANSIT_STOP>."
// No verbal alert
// "1": "Transfer at the <TRANSIT_STOP>."
TEST(NarrativeBuilder, TestBuildTransitConnectionTransfer_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionTransferManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionTransferManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Transfer at the CALTRAIN - SAN FRANCISCO STATION.", "", "",
                                  "Transfer at the CALTRAIN - SAN FRANCISCO STATION.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionTransferInstruction
// "2": "Transfer at the <TRANSIT_STOP> <STATION_LABEL>."
// No verbal alert
// "2": "Transfer at the <TRANSIT_STOP> <STATION_LABEL>."
TEST(NarrativeBuilder, TestBuildTransitConnectionTransfer_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionTransferManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionTransferManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Transfer at the 8 St - NYU Station.", "", "",
                                  "Transfer at the 8 St - NYU Station.", "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionDestinationInstruction
// "0": "Exit the station.",
// No verbal alert
// "0": "Exit the station.",
TEST(NarrativeBuilder, TestBuildTransitConnectionDestination_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionDestinationManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionDestinationManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Exit the station.", "", "",
                                  "Exit the station.", "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionDestinationInstruction
// "1": "Exit the <TRANSIT_STOP>."
// No verbal alert
// "1": "Exit the <TRANSIT_STOP>."
TEST(NarrativeBuilder, TestBuildTransitConnectionDestination_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionDestinationManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionDestinationManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Exit the CALTRAIN - SAN FRANCISCO STATION.",
                                  "", "", "Exit the CALTRAIN - SAN FRANCISCO STATION.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionDestinationInstruction
// "2": "Exit the <TRANSIT_STOP> <STATION_LABEL>."
// No verbal alert
// "2": "Exit the <TRANSIT_STOP> <STATION_LABEL>."
TEST(NarrativeBuilder, TestBuildTransitConnectionDestination_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitConnectionDestinationManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitConnectionDestinationManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Exit the 8 St - NYU Station.", "", "",
                                  "Exit the 8 St - NYU Station.", "Continue for 100 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "0": "Take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
// No verbal alert
// "0": "Take the <TRANSIT_NAME>.",
TEST(NarrativeBuilder, TestBuildTransit_0_train_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_0_train(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_0_train(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the train. (4 stops)", "", "",
                                  "Take the train.", "Travel 4 stops.",
                                  "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "0": "Take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
// No verbal alert
// "0": "Take the <TRANSIT_NAME>.",
TEST(NarrativeBuilder, TestBuildTransit_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the R. (4 stops)", "", "", "Take the R.",
                                  "Travel 4 stops.", "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_cable_car_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_cable_car(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_cable_car(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the cable car toward Powell & Market. (7 stops)", "", "",
                                  "Take the cable car toward Powell & Market.", "Travel 7 stops.",
                                  "Depart: 8:03 AM from Hyde St & Bay St.",
                                  "Depart at 8:03 AM from Hyde St & Bay St.",
                                  "Arrive: 8:06 AM at Hyde St & Vallejo St.",
                                  "Arrive at 8:06 AM at Hyde St & Vallejo St.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the R toward FOREST HILLS - 71 AV. (1 stop)", "", "",
                                  "Take the R toward FOREST HILLS - 71 AV.", "Travel 1 stop.",
                                  "Depart: 8:06 AM from Union St.",
                                  "Depart at 8:06 AM from Union St.",
                                  "Arrive: 8:08 AM at Atlantic Av - Barclays Ctr.",
                                  "Arrive at 8:08 AM at Atlantic Av - Barclays Ctr.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the R toward BAY RIDGE - 95 ST. (2 stops)", "", "",
                                  "Take the R toward BAY RIDGE - 95 ST.", "Travel 2 stops.",
                                  "Depart: 8:05 AM from 28 St.", "Depart at 8:05 AM from 28 St.",
                                  "Arrive: 8:08 AM at 14 St - Union Sq.",
                                  "Arrive at 8:08 AM at 14 St - Union Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the R toward FOREST HILLS - 71 AV. (4 stops)", "", "",
                                  "Take the R toward FOREST HILLS - 71 AV.", "Travel 4 stops.",
                                  "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_8_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the M toward FOREST HILLS - 71 AV. (8 stops)", "", "",
                                  "Take the M toward FOREST HILLS - 71 AV.", "Travel 8 stops.",
                                  "Depart: 8:11 AM from Flushing Av.",
                                  "Depart at 8:11 AM from Flushing Av.", "Arrive: 8:32 AM at 23 St.",
                                  "Arrive at 8:32 AM at 23 St.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_1_miles_cs_CZ) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("cs-CZ");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Jeďte R směrem na FOREST HILLS - 71 AV. (1 zastávka)", "", "",
                                  "Jeďte R směrem na FOREST HILLS - 71 AV.", "Cestování 1 zastávka.",
                                  "Vyjeďte: 08:06 z Union St.", "Vyjeďte v 08:06 z Union St.",
                                  "Přijeďte: 08:08 do Atlantic Av - Barclays Ctr.",
                                  "Přijeďte v 08:08 do Atlantic Av - Barclays Ctr.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_2_miles_cs_CZ) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("cs-CZ");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Jeďte R směrem na BAY RIDGE - 95 ST. (2 zastávky)", "", "",
                                  "Jeďte R směrem na BAY RIDGE - 95 ST.", "Cestování 2 zastávky.",
                                  "Vyjeďte: 08:05 z 28 St.", "Vyjeďte v 08:05 z 28 St.",
                                  "Přijeďte: 08:08 do 14 St - Union Sq.",
                                  "Přijeďte v 08:08 do 14 St - Union Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_4_miles_cs_CZ) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("cs-CZ");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Jeďte R směrem na FOREST HILLS - 71 AV. (4 zastávky)", "", "",
                                  "Jeďte R směrem na FOREST HILLS - 71 AV.", "Cestování 4 zastávky.",
                                  "Vyjeďte: 08:02 z 8 St - NYU.", "Vyjeďte v 08:02 z 8 St - NYU.",
                                  "Přijeďte: 08:08 do 34 St - Herald Sq.",
                                  "Přijeďte v 08:08 do 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitInstruction
// "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransit_1_stop_count_8_miles_cs_CZ) {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("cs-CZ");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitManeuverList_1_stop_count_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitManeuverList_1_stop_count_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Jeďte M směrem na FOREST HILLS - 71 AV. (8 zastávky)", "", "",
                                  "Jeďte M směrem na FOREST HILLS - 71 AV.", "Cestování 8 zastávky.",
                                  "Vyjeďte: 08:11 z Flushing Av.", "Vyjeďte v 08:11 z Flushing Av.",
                                  "Přijeďte: 08:32 do 23 St.", "Přijeďte v 08:32 do 23 St.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitTransferInstruction
// "0": "Transfer to take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
// No verbal alert
// "0": "Transfer to take the <TRANSIT_NAME>.",
TEST(NarrativeBuilder, TestBuildTransitTransfer_0_no_name_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitTransferManeuverList_0_no_name(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitTransferManeuverList_0_no_name(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Transfer to take the train. (4 stops)", "", "",
                                  "Transfer to take the train.", "Travel 4 stops.",
                                  "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitTransferInstruction
// "0": "Transfer to take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
// No verbal alert
// "0": "Transfer to take the <TRANSIT_NAME>.",
TEST(NarrativeBuilder, TestBuildTransitTransfer_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitTransferManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitTransferManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Transfer to take the R. (4 stops)", "", "",
                                  "Transfer to take the R.", "Travel 4 stops.",
                                  "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitTransferInstruction
// "1": "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Transfer to take the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransitTransfer_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitTransferManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitTransferManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Transfer to take the R toward FOREST HILLS - 71 AV. (4 stops)", "",
                                  "", "Transfer to take the R toward FOREST HILLS - 71 AV.",
                                  "Travel 4 stops.", "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitRemainOnInstruction
// "0": "Remain on the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
// No verbal alert
// "0": "Remain on the <TRANSIT_NAME>.",
TEST(NarrativeBuilder, TestBuildTransitRemainOn_0_no_name_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitRemainOnManeuverList_0_no_name(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitRemainOnManeuverList_0_no_name(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Remain on the train. (4 stops)", "", "",
                                  "Remain on the train.", "Travel 4 stops.",
                                  "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitRemainOnInstruction
// "0": "Remain on the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
// No verbal alert
// "0": "Remain on the <TRANSIT_NAME>.",
TEST(NarrativeBuilder, TestBuildTransitRemainOn_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitRemainOnManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitRemainOnManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Remain on the R. (4 stops)", "", "",
                                  "Remain on the R.", "Travel 4 stops.",
                                  "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitRemainOnInstruction
// "1": "Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
// <TRANSIT_STOP_COUNT_LABEL>)" No verbal alert "1": "Remain on the <TRANSIT_NAME> toward
// <TRANSIT_HEADSIGN>."
TEST(NarrativeBuilder, TestBuildTransitRemainOn_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTransitRemainOnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTransitRemainOnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Remain on the R toward FOREST HILLS - 71 AV. (4 stops)", "", "",
                                  "Remain on the R toward FOREST HILLS - 71 AV.", "Travel 4 stops.",
                                  "Depart: 8:02 AM from 8 St - NYU.",
                                  "Depart at 8:02 AM from 8 St - NYU.",
                                  "Arrive: 8:08 AM at 34 St - Herald Sq.",
                                  "Arrive at 8:08 AM at 34 St - Herald Sq.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_0(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southwest.", "Head southwest.", "",
                                  "Head southwest.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_1_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_1(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head southwest.",
                                  "Head southwest for 300 feet.", "", "Head southwest for 300 feet.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_2_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_2(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Head southwest on 6th Avenue/Avenue of the Americas.",
                                  "Head southwest.", "",
                                  "Head southwest on 6th Avenue, Avenue of the Americas.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_3_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_3(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Head southwest on 6th Avenue/Avenue of the Americas.",
      "Head southwest for 300 feet.", "",
      "Head southwest on 6th Avenue, Avenue of the Americas for 300 feet.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_4_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_4(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Head southwest on 6th Avenue/Avenue of the Americas. Continue on 6th Avenue.",
      "Head southwest.", "", "Head southwest on 6th Avenue, Avenue of the Americas.",
      "Continue on 6th Avenue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_5_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_5(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive southwest.", "Drive southwest.", "",
                                  "Drive southwest.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_6_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_6(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Drive southwest.",
                                  "Drive southwest for 300 feet.", "",
                                  "Drive southwest for 300 feet.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_7_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_7(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Drive southwest on 6th Avenue/Avenue of the Americas.",
                                  "Drive southwest.", "",
                                  "Drive southwest on 6th Avenue, Avenue of the Americas.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_8_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_8(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Drive southwest on 6th Avenue/Avenue of the Americas.",
      "Drive southwest for 300 feet.", "",
      "Drive southwest on 6th Avenue, Avenue of the Americas for 300 feet.",
      "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_9_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_9(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_9(expected_maneuvers, country_code,
                                                         state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Drive southwest on 6th Avenue/Avenue of the Americas. Continue on 6th Avenue.",
      "Drive southwest.", "", "Drive southwest on 6th Avenue, Avenue of the Americas.",
      "Continue on 6th Avenue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_10_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_10(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_10(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southwest.", "Walk southwest.", "",
                                  "Walk southwest.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_11_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_11(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_11(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Walk southwest.",
                                  "Walk southwest for 300 feet.", "", "Walk southwest for 300 feet.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_12_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_12(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_12(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Walk southwest on 6th Avenue/Avenue of the Americas.",
                                  "Walk southwest.", "",
                                  "Walk southwest on 6th Avenue, Avenue of the Americas.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_13_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_13(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_13(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Walk southwest on 6th Avenue/Avenue of the Americas.",
      "Walk southwest for 300 feet.", "",
      "Walk southwest on 6th Avenue, Avenue of the Americas for 300 feet.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_14_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_14(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_14(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Walk southwest on 6th Avenue/Avenue of the Americas. Continue on 6th Avenue.",
      "Walk southwest.", "", "Walk southwest on 6th Avenue, Avenue of the Americas.",
      "Continue on 6th Avenue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_15_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_15(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_15(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike southwest.", "Bike southwest.", "",
                                  "Bike southwest.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_16_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_16(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_16(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Bike southwest.",
                                  "Bike southwest for 300 feet.", "", "Bike southwest for 300 feet.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_17_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_17(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_17(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Bike southwest on 6th Avenue/Avenue of the Americas.",
                                  "Bike southwest.", "",
                                  "Bike southwest on 6th Avenue, Avenue of the Americas.",
                                  "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_18_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_18(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_18(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Bike southwest on 6th Avenue/Avenue of the Americas.",
      "Bike southwest for 300 feet.", "",
      "Bike southwest on 6th Avenue, Avenue of the Americas for 300 feet.", "Continue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
TEST(NarrativeBuilder, TestBuildPostTransitConnectionDestination_19_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_19(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulatePostTransitConnectionDestinationManeuverList_19(expected_maneuvers, country_code,
                                                          state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Bike southwest on 6th Avenue/Avenue of the Americas. Continue on 6th Avenue.",
      "Bike southwest.", "", "Bike southwest on 6th Avenue, Avenue of the Americas.",
      "Continue on 6th Avenue for 300 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormVerbalMultiCue
// 0 "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>"
TEST(NarrativeBuilder, TestBuildVerbalMultiCue_0_miles_en_US) {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateVerbalMultiCueManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateVerbalMultiCueManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers, "Turn left onto North Plum Street.",
      "Turn left. Then Turn left onto East Fulton Street.", "Turn left onto North Plum Street.",
      "Turn left onto North Plum Street. Then Turn left onto East Fulton Street.",
      "Continue for 200 feet.");
  SetExpectedManeuverInstructions(expected_maneuvers, "Turn left onto East Fulton Street.",
                                  "Turn left.", "Turn left onto East Fulton Street.",
                                  "Turn left onto East Fulton Street.", "Continue for 400 feet.");

  TryBuild(options, maneuvers, expected_maneuvers);
}

Maneuver
CreateVerbalPostManeuver(const std::vector<std::pair<std::string, bool>>& street_names,
                         float kilometers,
                         DirectionsLeg_Maneuver_Type type = DirectionsLeg_Maneuver_Type_kRight) {
  Maneuver maneuver;
  maneuver.set_street_names(street_names);
  maneuver.set_length(kilometers);
  maneuver.set_type(type);

  return maneuver;
}

void TryFormVerbalPostTransitionInstruction(NarrativeBuilderTest& nbt,
                                            Maneuver maneuver,
                                            bool include_street_names,
                                            const std::string& expected) {
  EXPECT_EQ(nbt.FormVerbalPostTransitionInstruction(maneuver, include_street_names), expected);
}

TEST(NarrativeBuilder, TestFormVerbalPostTransitionInstruction) {
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt_km(options, dictionary);

  // Verify kilometer whole number
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 4.0f),
                                         false, "Continue for 4 kilometers.");

  // Verify kilometers round down
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 3.4f),
                                         false, "Continue for 3 kilometers.");

  // Verify kilometers round up
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 3.6f),
                                         false, "Continue for 4 kilometers.");

  // Verify kilometers street name
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 2.8f),
                                         true, "Continue on Main Street for 3 kilometers.");

  // Verify 2.5 kilometers round down
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 2.7f),
                                         false, "Continue for 2.5 kilometers.");

  // Verify 2.5 kilometers round up
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 2.3f),
                                         false, "Continue for 2.5 kilometers.");

  // Verify 2 kilometers round down
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 2.2f),
                                         false, "Continue for 2 kilometers.");

  // Verify 2 kilometers round up
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 1.8f),
                                         false, "Continue for 2 kilometers.");

  // Verify 1.5 kilometers round down
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 1.7f),
                                         false, "Continue for 1.5 kilometers.");

  // Verify 1.5 kilometers round up
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 1.3f),
                                         false, "Continue for 1.5 kilometers.");

  // Verify 1 kilometer round down
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 1.1f),
                                         false, "Continue for 1 kilometer.");

  // Verify 1 kilometer round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.95f), false,
                                         "Continue for 1 kilometer.");

  // Verify 1 kilometer street name
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 1.0f),
                                         true, "Continue on Main Street for 1 kilometer.");

  // Verify 900 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.94f), false,
                                         "Continue for 900 meters.");

  // Verify 900 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.85f), false,
                                         "Continue for 900 meters.");

  // Verify 900 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 0.9f),
                                         true, "Continue on Main Street for 900 meters.");

  // Verify 400 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.44f), false,
                                         "Continue for 400 meters.");

  // Verify 400 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.35f), false,
                                         "Continue for 400 meters.");

  // Verify 400 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 0.4f),
                                         true, "Continue on Main Street for 400 meters.");

  // Verify 100 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.14f), false,
                                         "Continue for 100 meters.");

  // Verify 100 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.095f),
                                         false, "Continue for 100 meters.");

  // Verify 100 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km, CreateVerbalPostManeuver({{"Main Street", 0}}, 0.1f),
                                         true, "Continue on Main Street for 100 meters.");

  // Verify 90 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.094f),
                                         false, "Continue for 90 meters.");

  // Verify 90 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.085f),
                                         false, "Continue for 90 meters.");

  // Verify 90 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.09f), true,
                                         "Continue on Main Street for 90 meters.");

  // Verify 30 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.034f),
                                         false, "Continue for 30 meters.");

  // Verify 30 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.025f),
                                         false, "Continue for 30 meters.");

  // Verify 30 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.03f), true,
                                         "Continue on Main Street for 30 meters.");

  // Verify 10 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.012f),
                                         false, "Continue for 10 meters.");

  // Verify 10 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.0096f),
                                         false, "Continue for 10 meters.");

  // Verify 10 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.01f), true,
                                         "Continue on Main Street for 10 meters.");

  // Verify less than 10 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.0094f),
                                         false, "Continue for less than 10 meters.");

  // Verify less than 10 meters
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.0088f),
                                         false, "Continue for less than 10 meters.");

  // Verify less than 10 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.005f), true,
                                         "Continue on Main Street for less than 10 meters.");

  /////////////////////////////////////////////////////////////////////////////

  options.set_units(Options::miles);

  NarrativeBuilderTest nbt_mi(options, dictionary);

  // Verify mile whole number
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 4.828032f),
                                         false, "Continue for 3 miles.");

  // Verify large number
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 92.21541f),
                                         false, "Continue for 57 miles.");

  // Verify miles round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 3.604931f),
                                         false, "Continue for 2 miles.");

  // Verify miles round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 2.848539f),
                                         false, "Continue for 2 miles.");

  // Verify miles street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 3.637117f),
                                         true, "Continue on Main Street for 2 miles.");

  // Verify 1.5 miles round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 2.73588f),
                                         false, "Continue for 1.5 miles.");

  // Verify 1.5 miles round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 2.09215f),
                                         false, "Continue for 1.5 miles.");

  // Verify 1.5 mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 2.41402f),
                                         true, "Continue on Main Street for 1.5 miles.");

  // Verify 1 mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 1.657624f),
                                         false, "Continue for 1 mile.");

  // Verify 1 mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 1.561064f),
                                         false, "Continue for 1 mile.");

  // Verify 1 mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 1.60934f),
                                         true, "Continue on Main Street for 1 mile.");

  // Verify half mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.965606f),
                                         false, "Continue for a half mile.");

  // Verify half mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.643738f),
                                         false, "Continue for a half mile.");

  // Verify half mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.804672f),
                                         true, "Continue on Main Street for a half mile.");

  // Verify quarter mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.5632704f),
                                         false, "Continue for a quarter mile.");

  // Verify quarter mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}},
                                                                  0.30632398537f),
                                         false, "Continue for a quarter mile.");

  // Verify quarter mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.402336f),
                                         true, "Continue on Main Street for a quarter mile.");

  // Verify 500 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.155448f),
                                         false, "Continue for 500 feet.");

  // Verify 500 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.149352f),
                                         false, "Continue for 500 feet.");

  // Verify 500 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.1524f),
                                         true, "Continue on Main Street for 500 feet.");

  // Verify 100 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.036576f),
                                         false, "Continue for 100 feet.");

  // Verify 100 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.028956f),
                                         false, "Continue for 100 feet.");

  // Verify 100 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.03048f),
                                         true, "Continue on Main Street for 100 feet.");

  // Verify 90 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.0283464f),
                                         false, "Continue for 90 feet.");

  // Verify 90 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.0268224f),
                                         false, "Continue for 90 feet.");

  // Verify 90 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.027432f),
                                         true, "Continue on Main Street for 90 feet.");

  // Verify 10 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.0036576f),
                                         false, "Continue for 10 feet.");

  // Verify 10 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.00292608f),
                                         false, "Continue for 10 feet.");

  // Verify 10 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.003048f),
                                         true, "Continue on Main Street for 10 feet.");

  // Verify less than 10 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.00280416f),
                                         false, "Continue for less than 10 feet.");

  // Verify less than 10 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.00268224f),
                                         false, "Continue for less than 10 feet.");

  // Verify less than 10 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
                                         CreateVerbalPostManeuver({{"Main Street", 0}}, 0.001524f),
                                         true, "Continue on Main Street for less than 10 feet.");
}

Maneuver CreateVerbalMultiCueCurrentManeuver(const std::string& verbal_pre_transition_instruction,
                                             bool is_distant_verbal_multi_cue = false,
                                             float kilometers = 0.f) {
  Maneuver maneuver;
  maneuver.set_verbal_pre_transition_instruction(verbal_pre_transition_instruction);
  if (is_distant_verbal_multi_cue) {
    maneuver.set_distant_verbal_multi_cue(true);
  } else {
    maneuver.set_imminent_verbal_multi_cue(true);
  }
  maneuver.set_length(kilometers);

  return maneuver;
}

Maneuver CreateVerbalMultiCueNextManeuver(const std::string& verbal_transition_alert_instruction) {
  Maneuver maneuver;
  maneuver.set_verbal_transition_alert_instruction(verbal_transition_alert_instruction);

  return maneuver;
}

void TryFormVerbalMultiCue(NarrativeBuilderTest& nbt,
                           Maneuver current_maneuver,
                           Maneuver next_maneuver,
                           const std::string& expected) {
  EXPECT_EQ(nbt.FormVerbalMultiCue(current_maneuver, next_maneuver), expected);
}

TEST(NarrativeBuilder, TestFormVerbalMultiCue) {
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt_km(options, dictionary);

  TryFormVerbalMultiCue(nbt_km,
                        CreateVerbalMultiCueCurrentManeuver("Turn left onto North Plum Street."),
                        CreateVerbalMultiCueNextManeuver("Turn right onto East Fulton Street."),
                        "Turn left onto North Plum Street. Then Turn right onto East Fulton Street.");

  TryFormVerbalMultiCue(
      nbt_km, CreateVerbalMultiCueCurrentManeuver("Turn left onto North Plum Street.", true, 0.16f),
      CreateVerbalMultiCueNextManeuver("Turn right onto East Fulton Street."),
      "Turn left onto North Plum Street. Then, in 200 meters, Turn right onto East Fulton Street.");

  /////////////////////////////////////////////////////////////////////////////

  options.set_units(Options::miles);

  NarrativeBuilderTest nbt_mi(options, dictionary);

  TryFormVerbalMultiCue(
      nbt_mi, CreateVerbalMultiCueCurrentManeuver("Turn left onto North Plum Street.", true, 0.16f),
      CreateVerbalMultiCueNextManeuver("Turn right onto East Fulton Street."),
      "Turn left onto North Plum Street. Then, in 500 feet, Turn right onto East Fulton Street.");
}

void TryFormVerbalAlertApproachInstruction(NarrativeBuilderTest& nbt,
                                           float distance,
                                           const std::string& verbal_cue,
                                           const std::string& expected) {
  EXPECT_EQ(nbt.FormVerbalAlertApproachInstruction(distance, verbal_cue), expected);
}

TEST(NarrativeBuilder, TestFormVerbalAlertApproachInstruction) {
  Options options;
  options.set_units(Options::kilometers);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt_km(options, dictionary);

  TryFormVerbalAlertApproachInstruction(nbt_km, 0.125f, "Turn right onto Main Street.",
                                        "In 100 meters, Turn right onto Main Street.");

  TryFormVerbalAlertApproachInstruction(nbt_km, 0.4f, "Turn right onto Main Street.",
                                        "In 400 meters, Turn right onto Main Street.");

  TryFormVerbalAlertApproachInstruction(nbt_km, 0.8f, "Turn right onto Main Street.",
                                        "In 800 meters, Turn right onto Main Street.");

  TryFormVerbalAlertApproachInstruction(nbt_km, 1.f, "Take exit 1 30.",
                                        "In 1 kilometer, Take exit 1 30.");

  TryFormVerbalAlertApproachInstruction(nbt_km, 3.f, "Take exit 9 on the left.",
                                        "In 3 kilometers, Take exit 9 on the left.");

  /////////////////////////////////////////////////////////////////////////////

  options.set_units(Options::miles);
  NarrativeBuilderTest nbt_mi(options, dictionary);

  TryFormVerbalAlertApproachInstruction(nbt_mi, 0.125f, "Turn right onto Main Street",
                                        "In 700 feet, Turn right onto Main Street");

  TryFormVerbalAlertApproachInstruction(nbt_mi, 0.25f, "Turn right onto Main Street",
                                        "In a quarter mile, Turn right onto Main Street");

  TryFormVerbalAlertApproachInstruction(nbt_mi, 0.5f, "Turn right onto Main Street",
                                        "In a half mile, Turn right onto Main Street");

  TryFormVerbalAlertApproachInstruction(nbt_mi, 1.f, "Take exit 31C on the left.",
                                        "In 1 mile, Take exit 31C on the left.");

  TryFormVerbalAlertApproachInstruction(nbt_mi, 2.f, "Take exit 3 26.",
                                        "In 2 miles, Take exit 3 26.");
}

Maneuver CreateSignManeuver(DirectionsLeg_Maneuver_Type type,
                            Maneuver::RelativeDirection relative_direction,
                            bool drive_on_right,
                            const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_numbers,
                            const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_branches,
                            const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_towards,
                            const std::vector<std::tuple<std::string, bool, uint32_t>>& exit_names) {
  Maneuver maneuver;
  maneuver.set_type(type);
  maneuver.set_begin_relative_direction(relative_direction);
  maneuver.set_drive_on_right(drive_on_right);
  auto* signs = maneuver.mutable_signs();
  auto* exit_number_list = signs->mutable_exit_number_list();
  auto* exit_branch_list = signs->mutable_exit_branch_list();
  auto* exit_toward_list = signs->mutable_exit_toward_list();
  auto* exit_name_list = signs->mutable_exit_name_list();

  // Process exit numbers
  for (auto& exit_number : exit_numbers) {
    exit_number_list->emplace_back(std::get<TEXT>(exit_number),
                                   std::get<IS_ROUTE_NUMBER>(exit_number));
    exit_number_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(exit_number));
  }

  // Process exit branches
  for (auto& exit_branch : exit_branches) {
    exit_branch_list->emplace_back(std::get<TEXT>(exit_branch),
                                   std::get<IS_ROUTE_NUMBER>(exit_branch));
    exit_branch_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(exit_branch));
  }

  // Process exit numbers
  for (auto& exit_toward : exit_towards) {
    exit_toward_list->emplace_back(std::get<TEXT>(exit_toward),
                                   std::get<IS_ROUTE_NUMBER>(exit_toward));
    exit_toward_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(exit_toward));
  }

  // Process exit numbers
  for (auto& exit_name : exit_names) {
    exit_name_list->emplace_back(std::get<TEXT>(exit_name), std::get<IS_ROUTE_NUMBER>(exit_name));
    exit_name_list->back().set_consecutive_count(std::get<CONSECUTIVE_COUNT>(exit_name));
  }

  return maneuver;
}

void TryFormRampStraightInstruction(NarrativeBuilderTest& nbt,
                                    Maneuver maneuver,
                                    const std::string& expected) {
  EXPECT_EQ(nbt.FormRampStraightInstruction(maneuver), expected);
}

TEST(NarrativeBuilder, TestFormRampStraightInstruction) {
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt(options, dictionary);

  // phrase_id = 0
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {}, {}, {}),
                                 "Stay straight to take the ramp.");

  // phrase_id = 1
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {std::make_tuple("I 95 South", 1, 0)}, {},
                                                    {}),
                                 "Stay straight to take the I 95 South ramp.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {std::make_tuple("I 95 South", 1, 0)}, {},
                                                    {std::make_tuple("Gettysburg Pike", 0, 0)}),
                                 "Stay straight to take the I 95 South ramp.");

  // phrase_id = 2
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {}, {std::make_tuple("Baltimore", 0, 0)}, {}),
                                 "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {}, {std::make_tuple("Baltimore", 0, 0)},
                                                    {std::make_tuple("Gettysburg Pike", 0, 0)}),
                                 "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 3
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {std::make_tuple("I 95 South", 1, 0)},
                                                    {std::make_tuple("Baltimore", 0, 0)}, {}),
                                 "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward exists
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {std::make_tuple("I 95 South", 1, 0)},
                                                    {std::make_tuple("Baltimore", 0, 0)},
                                                    {std::make_tuple("Gettysburg Pike", 0, 0)}),
                                 "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 4
  TryFormRampStraightInstruction(nbt,
                                 CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampStraight,
                                                    Maneuver::RelativeDirection::kKeepStraight, true,
                                                    {}, {}, {},
                                                    {std::make_tuple("Gettysburg Pike", 0, 0)}),
                                 "Stay straight to take the Gettysburg Pike ramp.");
}

void TryFormRampRightInstruction(NarrativeBuilderTest& nbt,
                                 Maneuver maneuver,
                                 const std::string& expected) {
  EXPECT_EQ(nbt.FormRampInstruction(maneuver), expected);
}

TEST(NarrativeBuilder, TestFormRampRightInstruction) {
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt(options, dictionary);

  // phrase_id = 0
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {}, {}, {}),
                              "Take the ramp on the right.");

  // phrase_id = 1
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                              "Take the I 95 South ramp on the right.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the I 95 South ramp on the right.");

  // phrase_id = 2
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {}, {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the ramp on the right toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {}, {std::make_tuple("Baltimore", 0, 0)},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the ramp on the right toward Baltimore.");

  // phrase_id = 3
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 4
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the Gettysburg Pike ramp on the right.");

  // phrase_id = 5
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kRight, true, {}, {},
                                                 {}, {}),
                              "Turn right to take the ramp.");

  // phrase_id = 6
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                              "Turn right to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kRight, true, {}, {},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Turn right to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Turn right to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kRight, true, {}, {},
                                                 {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Turn right to take the Gettysburg Pike ramp.");

  // phrase_id = 10
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {}, {}, {}),
                              "Take the ramp.");

  // phrase_id = 11
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                              "Take the I 95 South ramp.");

  // phrase_id = 11; Test that exit name is not used when a branch exists
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the I 95 South ramp.");

  // phrase_id = 12
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {}, {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the ramp toward Baltimore.");

  // phrase_id = 12; Test that exit name is not used when a toward exists
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {}, {std::make_tuple("Baltimore", 0, 0)},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the ramp toward Baltimore.");

  // phrase_id = 13
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the I 95 South ramp toward Baltimore.");

  // phrase_id = 13; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the I 95 South ramp toward Baltimore.");

  // phrase_id = 14
  TryFormRampRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the Gettysburg Pike ramp.");
}

void TryFormRampLeftInstruction(NarrativeBuilderTest& nbt,
                                Maneuver maneuver,
                                const std::string& expected) {
  EXPECT_EQ(nbt.FormRampInstruction(maneuver), expected);
}

TEST(NarrativeBuilder, TestFormRampLeftInstruction) {
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt(options, dictionary);

  // phrase_id = 0
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                                                {}, {}),
                             "Take the ramp on the left.");

  // phrase_id = 1
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                             "Take the I 95 South ramp on the left.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)}, {},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the I 95 South ramp on the left.");

  // phrase_id = 2
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the ramp on the left toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                                                {std::make_tuple("Baltimore", 0, 0)},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the ramp on the left toward Baltimore.");

  // phrase_id = 3
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 4
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                                                {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the Gettysburg Pike ramp on the left.");

  // phrase_id = 5
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kLeft, true, {}, {}, {},
                                                {}),
                             "Turn left to take the ramp.");

  // phrase_id = 6
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                             "Turn left to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kLeft, true, {}, {},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Turn left to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Turn left to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kLeft, true, {}, {}, {},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Turn left to take the Gettysburg Pike ramp.");

  // phrase_id = 10
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {}, {}),
                             "Take the ramp.");

  // phrase_id = 11
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {},
                                                {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                             "Take the I 95 South ramp.");

  // phrase_id = 11; Test that exit name is not used when a branch exists
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {},
                                                {std::make_tuple("I 95 South", 1, 0)}, {},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the I 95 South ramp.");

  // phrase_id = 12
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the ramp toward Baltimore.");

  // phrase_id = 12; Test that exit name is not used when a toward exists
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {std::make_tuple("Baltimore", 0, 0)},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the ramp toward Baltimore.");

  // phrase_id = 13
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the I 95 South ramp toward Baltimore.");

  // phrase_id = 13; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the I 95 South ramp toward Baltimore.");

  // phrase_id = 14
  TryFormRampLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kRampLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the Gettysburg Pike ramp.");
}

void TryFormExitRightInstruction(NarrativeBuilderTest& nbt,
                                 Maneuver maneuver,
                                 const std::string& expected) {
  EXPECT_EQ(nbt.FormExitInstruction(maneuver), expected);
}

TEST(NarrativeBuilder, TestFormExitRightInstruction) {
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt(options, dictionary);

  // phrase_id = 0
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {}, {}, {}),
                              "Take the exit on the right.");

  // phrase_id = 1
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false,
                                                 {std::make_tuple("67A", 0, 0)}, {}, {}, {}),
                              "Take exit 67A on the right.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false,
                                                 {std::make_tuple("67A", 0, 0)}, {}, {},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take exit 67A on the right.");

  // phrase_id = 2
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                              "Take the I 95 South exit on the right.");

  // phrase_id = 3
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false,
                                                 {std::make_tuple("67A", 0, 0)},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                              "Take exit 67A on the right onto I 95 South.");

  // phrase_id = 4
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {}, {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the exit on the right toward Baltimore.");

  // phrase_id = 5
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false,
                                                 {std::make_tuple("67A", 0, 0)}, {},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take exit 67A on the right toward Baltimore.");

  // phrase_id = 6
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the I 95 South exit on the right toward Baltimore.");

  // phrase_id = 7
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false,
                                                 {std::make_tuple("67A", 0, 0)},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take exit 67A on the right onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the Gettysburg Pike exit on the right.");

  // phrase_id = 10
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, false, {},
                                                 {std::make_tuple("US 15", 1, 0)}, {},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the Gettysburg Pike exit on the right onto US 15.");

  // phrase_id = 12
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, false, {}, {},
                         {std::make_tuple("Harrisburg", 0, 0), std::make_tuple("Gettysburg", 0, 0)},
                         {std::make_tuple("Gettysburg Pike", 0, 0)}),
      "Take the Gettysburg Pike exit on the right toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, false, {},
                         {std::make_tuple("US 15", 1, 0)},
                         {std::make_tuple("Harrisburg", 0, 0), std::make_tuple("Gettysburg", 0, 0)},
                         {std::make_tuple("Gettysburg Pike", 0, 0)}),
      "Take the Gettysburg Pike exit on the right onto US 15 toward Harrisburg/Gettysburg.");

  // phrase_id = 15
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {}, {}, {}),
                              "Take the exit.");

  // phrase_id = 16
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true,
                                                 {std::make_tuple("67A", 0, 0)}, {}, {}, {}),
                              "Take exit 67A.");

  // phrase_id = 16; Test that name is ignored when number is present
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true,
                                                 {std::make_tuple("67A", 0, 0)}, {}, {},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take exit 67A.");

  // phrase_id = 17
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                              "Take the I 95 South exit.");

  // phrase_id = 18
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true,
                                                 {std::make_tuple("67A", 0, 0)},
                                                 {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                              "Take exit 67A onto I 95 South.");

  // phrase_id = 19
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {}, {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the exit toward Baltimore.");

  // phrase_id = 20
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true,
                                                 {std::make_tuple("67A", 0, 0)}, {},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take exit 67A toward Baltimore.");

  // phrase_id = 21
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take the I 95 South exit toward Baltimore.");

  // phrase_id = 22
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true,
                                                 {std::make_tuple("67A", 0, 0)},
                                                 {std::make_tuple("I 95 South", 1, 0)},
                                                 {std::make_tuple("Baltimore", 0, 0)}, {}),
                              "Take exit 67A onto I 95 South toward Baltimore.");

  // phrase_id = 23
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {}, {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the Gettysburg Pike exit.");

  // phrase_id = 25
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {std::make_tuple("US 15", 1, 0)}, {},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the Gettysburg Pike exit onto US 15.");

  // phrase_id = 27
  TryFormExitRightInstruction(nbt,
                              CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                                                 Maneuver::RelativeDirection::kKeepRight, true, {},
                                                 {},
                                                 {std::make_tuple("Harrisburg", 0, 0),
                                                  std::make_tuple("Gettysburg", 0, 0)},
                                                 {std::make_tuple("Gettysburg Pike", 0, 0)}),
                              "Take the Gettysburg Pike exit toward Harrisburg/Gettysburg.");

  // phrase_id = 29
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, true, {},
                         {std::make_tuple("US 15", 1, 0)},
                         {std::make_tuple("Harrisburg", 0, 0), std::make_tuple("Gettysburg", 0, 0)},
                         {std::make_tuple("Gettysburg Pike", 0, 0)}),
      "Take the Gettysburg Pike exit onto US 15 toward Harrisburg/Gettysburg.");
}

void TryFormExitLeftInstruction(NarrativeBuilderTest& nbt,
                                Maneuver maneuver,
                                const std::string& expected) {
  EXPECT_EQ(nbt.FormExitInstruction(maneuver), expected);
}

TEST(NarrativeBuilder, TestFormExitLeftInstruction) {
  Options options;
  options.set_units(Options::miles);
  options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(options);

  NarrativeBuilderTest nbt(options, dictionary);

  // phrase_id = 0
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                                                {}, {}),
                             "Take the exit on the left.");

  // phrase_id = 1
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true,
                                                {std::make_tuple("67A", 0, 0)}, {}, {}, {}),
                             "Take exit 67A on the left.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true,
                                                {std::make_tuple("67A", 0, 0)}, {}, {},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take exit 67A on the left.");

  // phrase_id = 2
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                             "Take the I 95 South exit on the left.");

  // phrase_id = 3
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true,
                                                {std::make_tuple("67A", 0, 0)},
                                                {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                             "Take exit 67A on the left onto I 95 South.");

  // phrase_id = 4
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the exit on the left toward Baltimore.");

  // phrase_id = 5
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true,
                                                {std::make_tuple("67A", 0, 0)}, {},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take exit 67A on the left toward Baltimore.");

  // phrase_id = 6
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the I 95 South exit on the left toward Baltimore.");

  // phrase_id = 7
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true,
                                                {std::make_tuple("67A", 0, 0)},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take exit 67A on the left onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                                                {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the Gettysburg Pike exit on the left.");

  // phrase_id = 10
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, true, {},
                                                {std::make_tuple("US 15", 1, 0)}, {},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the Gettysburg Pike exit on the left onto US 15.");

  // phrase_id = 12
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, true, {}, {},
                         {std::make_tuple("Harrisburg", 0, 0), std::make_tuple("Gettysburg", 0, 0)},
                         {std::make_tuple("Gettysburg Pike", 0, 0)}),
      "Take the Gettysburg Pike exit on the left toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, true, {},
                         {std::make_tuple("US 15", 1, 0)},
                         {std::make_tuple("Harrisburg", 0, 0), std::make_tuple("Gettysburg", 0, 0)},
                         {std::make_tuple("Gettysburg Pike", 0, 0)}),
      "Take the Gettysburg Pike exit on the left onto US 15 toward Harrisburg/Gettysburg.");

  // phrase_id = 15
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {}, {}),
                             "Take the exit.");

  // phrase_id = 16
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false,
                                                {std::make_tuple("67A", 0, 0)}, {}, {}, {}),
                             "Take exit 67A.");

  // phrase_id = 16; Test that name is ignored when number is present
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false,
                                                {std::make_tuple("67A", 0, 0)}, {}, {},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take exit 67A.");

  // phrase_id = 17
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {},
                                                {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                             "Take the I 95 South exit.");

  // phrase_id = 18
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false,
                                                {std::make_tuple("67A", 0, 0)},
                                                {std::make_tuple("I 95 South", 1, 0)}, {}, {}),
                             "Take exit 67A onto I 95 South.");

  // phrase_id = 19
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the exit toward Baltimore.");

  // phrase_id = 20
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false,
                                                {std::make_tuple("67A", 0, 0)}, {},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take exit 67A toward Baltimore.");

  // phrase_id = 21
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take the I 95 South exit toward Baltimore.");

  // phrase_id = 22
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false,
                                                {std::make_tuple("67A", 0, 0)},
                                                {std::make_tuple("I 95 South", 1, 0)},
                                                {std::make_tuple("Baltimore", 0, 0)}, {}),
                             "Take exit 67A onto I 95 South toward Baltimore.");

  // phrase_id = 23
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {}, {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the Gettysburg Pike exit.");

  // phrase_id = 25
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {},
                                                {std::make_tuple("US 15", 1, 0)}, {},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the Gettysburg Pike exit onto US 15.");

  // phrase_id = 27
  TryFormExitLeftInstruction(nbt,
                             CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                                                Maneuver::RelativeDirection::kKeepLeft, false, {}, {},
                                                {std::make_tuple("Harrisburg", 0, 0),
                                                 std::make_tuple("Gettysburg", 0, 0)},
                                                {std::make_tuple("Gettysburg Pike", 0, 0)}),
                             "Take the Gettysburg Pike exit toward Harrisburg/Gettysburg.");

  // phrase_id = 29
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(DirectionsLeg_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, false, {},
                         {std::make_tuple("US 15", 1, 0)},
                         {std::make_tuple("Harrisburg", 0, 0), std::make_tuple("Gettysburg", 0, 0)},
                         {std::make_tuple("Gettysburg Pike", 0, 0)}),
      "Take the Gettysburg Pike exit onto US 15 toward Harrisburg/Gettysburg.");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
