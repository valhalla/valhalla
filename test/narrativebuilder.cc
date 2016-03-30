#include <regex>

#include <valhalla/baldr/verbal_text_formatter_factory.h>

#include "proto/trippath.pb.h"
#include "odin/maneuver.h"
#include "odin/sign.h"
#include "odin/signs.h"
#include "odin/util.h"
#include "odin/narrative_builder_factory.h"
#include "odin/narrative_dictionary.h"
#include "odin/narrativebuilder.h"
#include "odin/enhancedtrippath.h"

#include "test.h"

using namespace std;
using namespace valhalla::odin;

namespace {

// Sub class to test protected methods
class NarrativeBuilderTest : public NarrativeBuilder {
 public:
  NarrativeBuilderTest(const DirectionsOptions& directions_options,
                       const NarrativeDictionary& dictionary,
                       const EnhancedTripPath* trip_path = nullptr)
      : NarrativeBuilder(directions_options, trip_path, dictionary) {
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

  std::string FormVerbalPostTransitionInstruction(
      Maneuver& maneuver, bool include_street_names = false,
      uint32_t element_max_count = kVerbalPostElementMaxCount,
      std::string delim = kVerbalDelim) {
    return NarrativeBuilder::FormVerbalPostTransitionInstruction(
        maneuver, include_street_names, element_max_count, delim);
  }

};

const NarrativeDictionary& GetNarrativeDictionary(
    const DirectionsOptions& directions_options) {
  // Get the locale dictionary
  const auto phrase_dictionary = get_locales().find(
      directions_options.language());

  // If language tag is not found then throw error
  if (phrase_dictionary == get_locales().end()) {
    throw std::runtime_error("Invalid language tag.");
  }

  return phrase_dictionary->second;
}

void PopulateManeuver(
    Maneuver& maneuver, const std::string& country_code,
    const std::string& state_code, TripDirections_Maneuver_Type type,
    std::vector<std::string> street_names,
    std::vector<std::string> begin_street_names,
    std::vector<std::string> cross_street_names, std::string instruction,
    float distance, uint32_t time, uint32_t turn_degree,
    Maneuver::RelativeDirection begin_relative_direction,
    TripDirections_Maneuver_CardinalDirection begin_cardinal_direction,
    uint32_t begin_heading, uint32_t end_heading, uint32_t begin_node_index,
    uint32_t end_node_index, uint32_t begin_shape_index,
    uint32_t end_shape_index, bool ramp, bool turn_channel, bool ferry,
    bool rail_ferry, bool roundabout, bool portions_toll, bool portions_unpaved,
    bool portions_highway, bool internal_intersection,
    std::vector<std::vector<std::string>> exit_numbers,
    std::vector<std::vector<std::string>> exit_branches,
    std::vector<std::vector<std::string>> exit_towards,
    std::vector<std::vector<std::string>> exit_names,
    uint32_t internal_right_turn_count = 0, uint32_t internal_left_turn_count =
        0,
    uint32_t roundabout_exit_count = 0, bool fork = false,
    bool begin_intersecting_edge_name_consistency = false,
    bool intersecting_forward_edge = false,
    std::string verbal_transition_alert_instruction = "",
    std::string verbal_pre_transition_instruction = "",
    std::string verbal_post_transition_instruction = "", bool tee = false,
    bool unnamed_walkway = false, bool unnamed_cycleway = false,
    bool unnamed_mountain_bike_trail = false, float basic_time = 0.0f,
    bool verbal_multi_cue = false) {

  maneuver.set_verbal_formatter(
      VerbalTextFormatterFactory::Create(country_code, state_code));

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

  maneuver.set_internal_right_turn_count(internal_right_turn_count);
  maneuver.set_internal_left_turn_count(internal_left_turn_count);
  maneuver.set_roundabout_exit_count(roundabout_exit_count);
  maneuver.set_fork(fork);
  maneuver.set_begin_intersecting_edge_name_consistency(
      begin_intersecting_edge_name_consistency);
  maneuver.set_intersecting_forward_edge(intersecting_forward_edge);
  maneuver.set_verbal_transition_alert_instruction(
      verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(
      verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(
      verbal_post_transition_instruction);
  maneuver.set_tee(tee);
  maneuver.set_unnamed_walkway(unnamed_walkway);
  maneuver.set_unnamed_cycleway(unnamed_cycleway);
  maneuver.set_unnamed_mountain_bike_trail(unnamed_mountain_bike_trail);
  maneuver.set_basic_time(basic_time);
  maneuver.set_verbal_multi_cue(verbal_multi_cue);
}

void TryBuild(const DirectionsOptions& directions_options,
              std::list<Maneuver>& maneuvers,
              std::list<Maneuver>& expected_maneuvers,
              const EnhancedTripPath* etp = nullptr) {
  std::unique_ptr<NarrativeBuilder> narrative_builder =
      NarrativeBuilderFactory::Create(directions_options, etp);
  narrative_builder->Build(directions_options, etp, maneuvers);

  // Check maneuver list sizes
  if (maneuvers.size() != expected_maneuvers.size())
    throw std::runtime_error("Incorrect maneuver count");
  for (auto man = maneuvers.begin(), expected_man = expected_maneuvers.begin();
      man != maneuvers.end(); ++man, ++expected_man) {

    // Check maneuver type
    if (man->type() != expected_man->type()) {
      throw std::runtime_error("Incorrect maneuver type");
    }

    // Check maneuver instruction
    if (man->instruction() != expected_man->instruction()) {
      throw std::runtime_error(
          "Incorrect maneuver instruction - expected: "
              + expected_man->instruction() + "  |  produced: "
              + man->instruction());
    }

    // Check maneuver verbal_transition_alert_instruction
    if (man->verbal_transition_alert_instruction()
        != expected_man->verbal_transition_alert_instruction()) {
      throw std::runtime_error(
          "Incorrect maneuver verbal_transition_alert_instruction - expected: "
              + expected_man->verbal_transition_alert_instruction()
              + "  |  produced: " + man->verbal_transition_alert_instruction());
    }

    // Check maneuver verbal_pre_transition_instruction
    if (man->verbal_pre_transition_instruction()
        != expected_man->verbal_pre_transition_instruction()) {
      throw std::runtime_error(
          "Incorrect maneuver verbal_pre_transition_instruction - expected: "
              + expected_man->verbal_pre_transition_instruction()
              + "  |  produced: " + man->verbal_pre_transition_instruction());
    }

    // Check maneuver verbal_post_transition_instruction
    if (man->verbal_post_transition_instruction()
        != expected_man->verbal_post_transition_instruction()) {
      throw std::runtime_error(
          "Incorrect maneuver verbal_post_transition_instruction - expected: "
              + expected_man->verbal_post_transition_instruction()
              + "  |  produced: " + man->verbal_post_transition_instruction());
    }
  }
}

void PopulateStartManeuverList_0(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStart, { }, { }, { }, "",
                   0.786592, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kEast, 88, 80, 0,
                   1, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 0,
                   0, 0, 1, 0, "", "", "", 0);

}

void PopulateStartManeuverList_1(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStart, { "5th Avenue" }, { },
                   { }, "", 0.224001, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 209,
                   209, 0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0);

}

void PopulateStartManeuverList_2(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStart, { "US 222", "PA 272" },
                   { "North Prince Street", "US 222", "PA 272" }, { }, "",
                   5.098166, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 173, 143,
                   0, 45, 0, 88, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { },
                   0, 0, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateDestinationManeuverList_0(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kDestination, { }, { }, { }, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 6, 6,
                   7, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 0, 0,
                   0, 0, 0, "", "", "", 0);
}

void PopulateDestinationManeuverList_1(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kDestination, { }, { }, { }, "",
                   0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 120,
                   120, 1756, 1756, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 0, 0, 0, 0, 0, "", "", "", 0);
}

void PopulateDestinationManeuverList_2(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kDestinationRight, { }, { },
                   { }, "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4,
                   6, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 0, 0,
                   0, 0, 0, "", "", "", 0);
}

void PopulateDestinationManeuverList_3(std::list<Maneuver>& maneuvers,
                                       const std::string& country_code,
                                       const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kDestinationLeft, { }, { }, { },
                   "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 0, 0, 4, 4,
                   6, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 0, 0,
                   0, 0, 0, "", "", "", 0);
}

void PopulateContinueManeuverList_0(std::list<Maneuver>& maneuvers,
                                    const std::string& country_code,
                                    const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kContinue, { }, { }, { }, "",
                   0.097000, 26, 0, Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 291, 323, 1,
                   3, 3, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   0, 0, 0, 0, 0, "", "", "", 0);
}

void PopulateContinueManeuverList_1(std::list<Maneuver>& maneuvers,
                                    const std::string& country_code,
                                    const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kContinue, { "10th Avenue" },
                   { }, { }, "", 0.481000, 34, 6,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 185, 218,
                   2, 5, 4, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { },
                   0, 0, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateTurnManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kLeft, { }, { }, { }, "",
                   0.824000, 60, 282, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kWest, 260, 268, 1,
                   2, 1, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 1,
                   0, 0, 1, 1, "", "", "", 0);
}

void PopulateTurnManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kSharpRight,
                   { "Flatbush Avenue" }, { }, { }, "", 0.192229, 44, 147,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 322,
                   322, 1, 4, 1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 1, 0, 0, 0, 1, 1, "", "", "", 0);
}

void PopulateTurnManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kSharpLeft, { "MD 924" }, {
                       "North Bond Street", "US 1 Business", "MD 924" },
                   { }, "", 0.840369, 111, 201,
                   Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 141,
                   144, 2, 16, 2, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 1, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateTurnManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kRight, { "Sunstone Drive" },
                   { }, { }, "", 0.077000, 49, 89,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kEast, 75, 77, 1,
                   3, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 1, 0,
                   0, 0, 1, 1, "", "", "", 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kRight, { "Sunstone Drive" },
                   { }, { }, "", 0.038000, 12, 90,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 167, 167,
                   3, 4, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 1,
                   0, 0, 0, 1, 0, "", "", "", 1);
}

void PopulateBearManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kSlightRight, { }, { }, { }, "",
                   0.018000, 9, 37, Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 303,
                   303, 3, 4, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 1, 0, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateBearManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kSlightLeft, { "Arlen Road" },
                   { }, { }, "", 0.118000, 22, 323,
                   Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kEast, 86, 126,
                   210, 212, 1566, 1576, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { },
                   { }, { }, 0, 1, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateBearManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kSlightRight,
                   { "US 1 Business" }, { "Belair Road", "US 1 Business" }, { },
                   "", 3.431836, 275, 30,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 36,
                   115, 82, 115, 257, 338, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateBearManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kRoundaboutExit, { "US 15" }, {
                       "Catoctin Mountain Highway", "US 15" },
                   { }, "", 18.278002, 928, 34,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 201, 204,
                   161, 187, 1461, 1805, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { },
                   { }, { }, 1, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 900, 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kSlightLeft, { "US 15 South" },
                   { }, { }, "", 4.137000, 232, 349,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 193, 197,
                   187, 197, 1805, 1878, 0, 0, 0, 0, 0, 0, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 0, 1, "", "", "", 0, 0, 0, 0, 200, 0);
}

void PopulateUturnManeuverList_0(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnLeft, { }, { }, { }, "",
                   0.592000, 28, 180, Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kEast, 76, 76, 3,
                   4, 24, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   0, 0, 0, 0, 0, "", "", "", 0);
}

void PopulateUturnManeuverList_1(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnRight, {
                       "Bunker Hill Road" },
                   { }, { }, "", 0.592000, 28, 180,
                   Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kEast, 76, 76, 3,
                   4, 24, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   0, 0, 0, 0, 0, "", "", "", 0);
}

void PopulateUturnManeuverList_2(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kRight, { "Bunker Hill Road" },
                   { }, { }, "", 0.287000, 28, 81,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kNorthWest, 335,
                   337, 2, 3, 36, 46, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 1, 0, 0, 0, 1, 1, "", "", "", 0, 0, 0, 0, 20, 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnLeft,
                   { "Bunker Hill Road" }, { }, { }, "", 0.287000, 25, 180,
                   Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 157, 155,
                   3, 4, 46, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { },
                   0, 0, 0, 0, 0, 0, "", "", "", 0, 0, 0, 0, 20, 0);
}

void PopulateUturnManeuverList_3(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnLeft, { }, { }, {
                       "Devonshire Road" },
                   "", 0.072697, 47, 180, Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 212,
                   221, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 1, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateUturnManeuverList_4(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnLeft, { "Jonestown Road",
                       "US 22" },
                   { }, { "Devonshire Road" }, "", 0.072697, 47, 180,
                   Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 212,
                   221, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 1, 0, 0, 1, 0, "", "", "", 0);
}

void PopulateUturnManeuverList_5(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kStart, { "Jonestown Road",
                       "US 22" },
                   { }, { }, "", 0.062923, 2, 0,
                   Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 36, 32,
                   0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 2, 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnLeft, { "Jonestown Road",
                       "US 22" },
                   { }, { "Devonshire Road" }, "", 0.072697, 47, 180,
                   Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 212,
                   221, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 1, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 40, 0);
}

void PopulateRampStraightManeuverList_0(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampStraight, { }, { }, { },
                   "", 2.4, 37, 340, Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 60, 57,
                   9, 10, 88, 92, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { },
                   0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampStraightManeuverList_1(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampStraight, { }, { }, { },
                   "", 0.374000, 37, 340,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 60, 57,
                   9, 10, 88, 92, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "US 322 East", "1" } },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampStraightManeuverList_2(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampStraight, { }, { }, { },
                   "", 0.374000, 37, 340,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 60, 57,
                   9, 10, 88, 92, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { {
                       "Hershey", "0" } },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampStraightManeuverList_3(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampStraight, { }, { }, { },
                   "", 0.374000, 37, 340,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 60, 57,
                   9, 10, 88, 92, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "US 322 East", "1" }, { "US 422 East", "1" }, {
                       "US 522 East", "1" }, { "US 622 East", "1" }, {
                       "US 722 East", "1" } },
                   { { "Hershey", "1" }, { "Palmdale", "1" },
                       { "Palmyra", "1" }, { "Campbelltown", "1" }, { "Eprata",
                           "1" } },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampStraightManeuverList_4(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampStraight, { }, { }, { },
                   "", 0.374000, 37, 340,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 60, 57,
                   9, 10, 88, 92, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { {
                       "Gettysburg Pike", "0" } },
                   0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 30, 0);
}

void PopulateRampManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampRight, { }, { }, { }, "",
                   0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kEast, 99, 137, 14,
                   15, 61, 71, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 9, 0);
}

void PopulateRampManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampRight, { }, { }, { }, "",
                   0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kEast, 99, 137, 14,
                   15, 61, 71, 1, 0, 0, 0, 0, 0, 0, 0, 0, { },
                   { { "I 95", "0" } }, { }, { }, 0, 0, 0, 0, 1, 0, "", "", "",
                   0, 0, 0, 0, 9, 0);
}

void PopulateRampManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampLeft, { "NY 27 East",
                       "South Conduit Avenue" },
                   { }, { }, "", 0.124000, 6, 353,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kEast, 105, 102,
                   24, 25, 204, 206, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { {
                       "JFK", "0" } },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 6, 0);
}

void PopulateRampManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampLeft, { "NY 27 East",
                       "South Conduit Avenue" },
                   { }, { }, "", 0.124000, 6, 353,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kEast, 105, 102,
                   24, 25, 204, 206, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "South Conduit Avenue", "0" } },
                   { { "JFK", "0" } }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0,
                   0, 0, 6, 0);
}

void PopulateRampManeuverList_4(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampRight, { }, { }, { }, "",
                   0.234000, 9, 10, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kEast, 99, 137, 14,
                   15, 61, 71, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { {
                       "Gettysburg Pike", "0" } },
                   0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 9, 0);
}

void PopulateRampManeuverList_5(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampRight, { }, { }, { }, "",
                   0.256000, 46, 92, Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kEast, 86, 129, 11,
                   13, 51, 62, 1, 0, 0, 0, 0, 1, 0, 0, 0, { }, { }, { }, { }, 1,
                   0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_6(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampLeft, { }, { }, { }, "",
                   0.539000, 31, 266, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kEast, 86, 277, 24,
                   26, 60, 79, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "PA 283 West", "1" } },
                   { }, { }, 0, 1, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_7(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampLeft, { }, { }, { }, "",
                   0.539000, 31, 266, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kEast, 86, 277, 24,
                   26, 60, 79, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { {
                       "Harrisburg", "0" }, {
                       "Harrisburg International Airport", "0" } },
                   { }, 0, 1, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_8(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampLeft, { }, { }, { }, "",
                   0.539000, 31, 266, Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kEast, 86, 277, 24,
                   26, 60, 79, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "PA 283 West", "1" } },
                   { { "Harrisburg", "0" }, {
                       "Harrisburg International Airport", "0" } },
                   { }, 0, 1, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateRampManeuverList_9(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRampRight, { }, { }, { }, "",
                   0.256000, 46, 92, Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kEast, 86, 129, 11,
                   13, 51, 62, 1, 0, 0, 0, 0, 1, 0, 0, 0, { }, { }, { }, { {
                       "Gettysburg Pike", "0" } },
                   1, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { { "67 B-A",
                       "0" } },
                   { }, { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23,
                   0);
}

void PopulateExitManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "US 322 West", "2" } },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { { "67 B-A",
                       "0" } },
                   { { "US 322 West", "2" } }, { }, { }, 0, 0, 0, 0, 1, 0, "",
                   "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_4(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { {
                       "Lewistown", "1" } },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_5(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { { "67 B-A",
                       "0" } },
                   { }, { { "Lewistown", "1" } }, { }, 0, 0, 0, 0, 1, 0, "", "",
                   "", 0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_6(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "US 322 West", "2" } },
                   { { "Lewistown", "1" } }, { }, 0, 0, 0, 0, 1, 0, "", "", "",
                   0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_7(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { "US 322 West" },
                   { }, { }, "", 0.561000, 23, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 272, 278,
                   42, 43, 260, 264, 1, 0, 0, 0, 0, 0, 0, 0, 0, { { "67 B-A",
                       "0" } },
                   { { "US 322 West", "2" }, { "US 22 West", "1" }, {
                       "US 22 East", "0" }, { "PA 230 East", "0" }, {
                       "Cameron Street", "0" } },
                   { { "Lewistown", "1" }, { "State College", "1" }, {
                       "Harrisburg", "0" } },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 23, 0);
}

void PopulateExitManeuverList_8(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitLeft, { "MD 43 East" },
                   { }, { }, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 135,
                   83, 158, 160, 1420, 1444, 1, 0, 0, 0, 0, 0, 0, 0, 0, { },
                   { }, { }, { { "White Marsh Boulevard", "0" } }, 0, 0, 0, 0,
                   1, 0, "", "", "", 0, 0, 0, 0, 46, 0);
}

void PopulateExitManeuverList_10(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitLeft, { "MD 43 East" },
                   { }, { }, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 135,
                   83, 158, 160, 1420, 1444, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "MD 43 East", "1" } },
                   { }, { { "White Marsh Boulevard", "0" } }, 0, 0, 0, 0, 1, 0,
                   "", "", "", 0, 0, 0, 0, 46, 0);
}

void PopulateExitManeuverList_12(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitLeft, { "MD 43 East" },
                   { }, { }, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 135,
                   83, 158, 160, 1420, 1444, 1, 0, 0, 0, 0, 0, 0, 0, 0, { },
                   { }, { { "White Marsh", "0" } }, { { "White Marsh Boulevard",
                       "0" } },
                   0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 46, 0);
}

void PopulateExitManeuverList_14(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitLeft, { "MD 43 East" },
                   { }, { }, "", 1.002000, 45, 356,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 135,
                   83, 158, 160, 1420, 1444, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { {
                       "MD 43 East", "1" } },
                   { { "White Marsh", "0" } }, {
                       { "White Marsh Boulevard", "0" } },
                   0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 46, 0);
}

void PopulateKeepManeuverList_0(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayStraight, { }, { }, { },
                   "", 0.068000, 4, 1,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kEast, 91, 97, 2,
                   3, 8, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   0, 0, 1, 1, 0, "", "", "", 0, 0, 0, 0, 4, 0);
}

void PopulateKeepManeuverList_1(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayRight, { }, { }, { }, "",
                   14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 221,
                   214, 21, 45, 148, 323, 0, 0, 0, 0, 0, 1, 0, 1, 0, { { "62",
                       "0" } },
                   { }, { }, { }, 0, 0, 0, 1, 1, 0, "", "", "", 0, 0, 0, 0, 581,
                   0);
}

void PopulateKeepManeuverList_2(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayRight, { "I 895 South" },
                   { }, { }, "", 14.464000, 634, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 221,
                   214, 21, 45, 148, 323, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 1, 1, 0, "", "", "", 0, 0, 0, 0, 581, 0);
}

void PopulateKeepManeuverList_3(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayRight, { "I 895 South" },
                   { }, { }, "", 14.464000, 634, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 221,
                   214, 21, 45, 148, 323, 0, 0, 0, 0, 0, 1, 0, 1, 0, { { "62",
                       "0" } },
                   { }, { }, { }, 0, 0, 0, 1, 1, 0, "", "", "", 0, 0, 0, 0, 581,
                   0);
}

void PopulateKeepManeuverList_4(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayRight, { }, { }, { }, "",
                   14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 221,
                   214, 21, 45, 148, 323, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { }, {
                       { "Annapolis", "0" } },
                   { }, 0, 0, 0, 1, 1, 0, "", "", "", 0, 0, 0, 0, 581, 0);
}

void PopulateKeepManeuverList_5(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayRight, { }, { }, { }, "",
                   14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 221,
                   214, 21, 45, 148, 323, 0, 0, 0, 0, 0, 1, 0, 1, 0, { { "62",
                       "0" } },
                   { }, { { "Annapolis", "0" } }, { }, 0, 0, 0, 1, 1, 0, "", "",
                   "", 0, 0, 0, 0, 581, 0);
}

void PopulateKeepManeuverList_6(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayRight, { }, { }, { }, "",
                   14.464000, 634, 2, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 221,
                   214, 21, 45, 148, 323, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { {
                       "I 895 South", "1" }, {
                       "Baltimore Harbor Tunnel Thruway", "0" } },
                   { { "Annapolis", "0" } }, { }, 0, 0, 0, 1, 1, 0, "", "", "",
                   0, 0, 0, 0, 581, 0);
}

void PopulateKeepManeuverList_7(std::list<Maneuver>& maneuvers,
                                const std::string& country_code,
                                const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayRight, { "I 895 South" },
                   { }, { }, "", 14.464000, 634, 2,
                   Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 221,
                   214, 21, 45, 148, 323, 0, 0, 0, 0, 0, 1, 0, 1, 0, { { "62",
                       "0" } },
                   { { "I 895 South", "1" }, {
                       "Baltimore Harbor Tunnel Thruway", "0" } },
                   { { "Annapolis", "0" } }, { }, 0, 0, 0, 1, 1, 0, "", "", "",
                   0, 0, 0, 0, 581, 0);
}

void PopulateKeepToStayOnManeuverList_0(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kMerge, { "I 95 South",
                       "John F. Kennedy Memorial Highway" },
                   { }, { }, "", 23.639002, 843, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 234,
                   219, 24, 34, 210, 380, 0, 0, 0, 0, 0, 0, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayLeft, { "I 95 South" },
                   { }, { }, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 219,
                   232, 34, 45, 380, 491, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { {
                       "I 95 South", "0" } },
                   { }, { }, 0, 0, 0, 1, 0, 0, "", "", "", 0, 0, 0, 0, 334, 0);
}

void PopulateKeepToStayOnManeuverList_1(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kMerge, { "I 95 South",
                       "John F. Kennedy Memorial Highway" },
                   { }, { }, "", 23.639002, 843, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 234,
                   219, 24, 34, 210, 380, 0, 0, 0, 0, 0, 0, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayLeft, { "I 95 South" },
                   { }, { }, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 219,
                   232, 34, 45, 380, 491, 0, 0, 0, 0, 0, 1, 0, 1, 0, { { "62",
                       "0" } },
                   { { "I 95 South", "0" } }, { }, { }, 0, 0, 0, 1, 0, 0, "",
                   "", "", 0, 0, 0, 0, 334, 0);
}

void PopulateKeepToStayOnManeuverList_2(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kMerge, { "I 95 South",
                       "John F. Kennedy Memorial Highway" },
                   { }, { }, "", 23.639002, 843, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 234,
                   219, 24, 34, 210, 380, 0, 0, 0, 0, 0, 0, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayLeft, { "I 95 South" },
                   { }, { }, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 219,
                   232, 34, 45, 380, 491, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { {
                       "I 95 South", "0" } },
                   { { "Baltimore", "0" } }, { }, 0, 0, 0, 1, 0, 0, "", "", "",
                   0, 0, 0, 0, 334, 0);
}

void PopulateKeepToStayOnManeuverList_3(std::list<Maneuver>& maneuvers,
                                        const std::string& country_code,
                                        const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kMerge, { "I 95 South",
                       "John F. Kennedy Memorial Highway" },
                   { }, { }, "", 23.639002, 843, 2,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 234,
                   219, 24, 34, 210, 380, 0, 0, 0, 0, 0, 0, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 832, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kStayLeft, { "I 95 South" },
                   { }, { }, "", 8.258000, 363, 0,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 219,
                   232, 34, 45, 380, 491, 0, 0, 0, 0, 0, 1, 0, 1, 0, { { "62",
                       "0" } },
                   { { "I 95 South", "0" } }, { { "Baltimore", "0" } }, { }, 0,
                   0, 0, 1, 0, 0, "", "", "", 0, 0, 0, 0, 334, 0);
}

void PopulateMergeManeuverList_0(std::list<Maneuver>& maneuvers,
                                 const std::string& country_code,
                                 const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { }, { }, { }, "",
                   0.864000, 34, 6, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 174, 241,
                   39, 41, 158, 180, 1, 0, 0, 0, 0, 1, 0, 0, 0, { }, { {
                       "I 76 West", "1" } },
                   { { "Pittsburgh", "0" } }, { }, 0, 0, 0, 0, 1, 0, "", "", "",
                   0, 0, 0, 0, 34, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kMerge, { }, { }, { }, "",
                   7.624001, 245, 1, Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 242,
                   293, 41, 42, 180, 236, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 243, 0);
}

void PopulateMergeManeuverList_1_1(std::list<Maneuver>& maneuvers,
                                   const std::string& country_code,
                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { }, { }, { }, "",
                   0.864000, 34, 6, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 174, 241,
                   39, 41, 158, 180, 1, 0, 0, 0, 0, 1, 0, 0, 0, { }, { {
                       "I 76 West", "1" } },
                   { { "Pittsburgh", "0" } }, { }, 0, 0, 0, 0, 1, 0, "", "", "",
                   0, 0, 0, 0, 34, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kMerge, { "I 76 West",
                       "Pennsylvania Turnpike" },
                   { }, { }, "", 7.624001, 245, 1,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 242,
                   293, 41, 42, 180, 236, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 243, 0);
}

void PopulateMergeManeuverList_1_2(std::list<Maneuver>& maneuvers,
                                   const std::string& country_code,
                                   const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kExitRight, { }, { }, { }, "",
                   2.1, 34, 6, Maneuver::RelativeDirection::kKeepRight,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 174, 241,
                   39, 41, 158, 180, 1, 0, 0, 0, 0, 1, 0, 0, 0, { }, { {
                       "I 76 West", "1" } },
                   { { "Pittsburgh", "0" } }, { }, 0, 0, 0, 0, 1, 0, "", "", "",
                   0, 0, 0, 0, 34, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kMerge, { "I 76 West",
                       "Pennsylvania Turnpike" },
                   { }, { }, "", 7.624001, 245, 1,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 242,
                   293, 41, 42, 180, 236, 0, 0, 0, 0, 0, 1, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 243, 0);
}

void PopulateEnterRoundaboutManeuverList_0(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRoundaboutEnter, { "US 15",
                       "MD 464" },
                   { }, { }, "", 0.043000, 2, 41,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 264, 167,
                   135, 139, 1457, 1464, 0, 0, 0, 0, 1, 0, 0, 0, 0, { }, { },
                   { }, { }, 1, 2, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 2, 0);
}

void PopulateEnterRoundaboutManeuverList_1(std::list<Maneuver>& maneuvers,
                                           const std::string& country_code,
                                           const std::string& state_code,
                                           uint32_t roundabout_exit_count) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRoundaboutEnter, { "US 15",
                       "MD 464" },
                   { }, { }, "", 0.043000, 2, 41,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 264, 167,
                   135, 139, 1457, 1464, 0, 0, 0, 0, 1, 0, 0, 0, 0, { }, { },
                   { }, { }, 1, 2, roundabout_exit_count, 0, 1, 0, "", "", "",
                   0, 0, 0, 0, 2, 0);
}

void PopulateExitRoundaboutManeuverList_0(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRoundaboutExit, { }, { }, { },
                   "", 1.041000, 69, 24,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 224,
                   262, 8, 11, 32, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 64, 0);
}

void PopulateExitRoundaboutManeuverList_1(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRoundaboutExit, {
                       "Philadelphia Road", "MD 7" },
                   { }, { }, "", 1.041000, 69, 24,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 224,
                   262, 8, 11, 32, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 64, 0);
}

void PopulateExitRoundaboutManeuverList_2(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kRoundaboutExit, { "US 15" }, {
                       "Catoctin Mountain Highway", "US 15" },
                   { }, "", 18.278002, 923, 34,
                   Maneuver::RelativeDirection::kRight,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 201, 204,
                   139, 154, 1464, 1808, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { },
                   { }, { }, 1, 0, 0, 0, 1, 0, "", "", "", 0, 0, 0, 0, 914, 0);
}

void PopulateEnterFerryManeuverList_0(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kFerryEnter, { }, { }, { }, "",
                   1.446000, 822, 4, Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 280, 280, 5,
                   6, 8, 9, 0, 0, 1, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 0,
                   0, 0, 0, 0, "", "", "", 0, 0, 0, 0, 521, 0);
}

void PopulateEnterFerryManeuverList_1(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kFerryEnter, {
                       "Millersburg Ferry" },
                   { }, { }, "", 1.446000, 822, 4,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 280, 280, 5,
                   6, 8, 9, 0, 0, 1, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 0,
                   0, 0, 0, 0, "", "", "", 0, 0, 0, 0, 521, 0);
}

void PopulateEnterFerryManeuverList_2(std::list<Maneuver>& maneuvers,
                                      const std::string& country_code,
                                      const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kFerryEnter, {
                       "Bridgeport - Port Jefferson" },
                   { }, { }, "", 27.731001, 3628, 24,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 151,
                   142, 3, 4, 15, 30, 0, 0, 1, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 0, 0, 0, 0, 0, "", "", "", 0, 0, 0, 0, 3328, 0);
}

void PopulateExitFerryManeuverList_0(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kFerryExit, { }, { }, { }, "",
                   0.065000, 11, 2, Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kSouthEast, 144,
                   94, 4, 5, 30, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 0, 0, 0, 0, 0, "", "", "", 0, 0, 0, 0, 12, 0);
}

void PopulateExitFerryManeuverList_1(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kFerryExit, { "Ferry Lane" },
                   { }, { }, "", 0.578000, 81, 7,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kWest, 287, 262, 6,
                   13, 9, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 1,
                   0, 0, 0, 0, 0, "", "", "", 0, 0, 0, 0, 70, 0);
}

void PopulateExitFerryManeuverList_2(std::list<Maneuver>& maneuvers,
                                     const std::string& country_code,
                                     const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver = maneuvers.back();
  PopulateManeuver(maneuver, country_code, state_code,
                   TripDirections_Maneuver_Type_kFerryExit, { "US 9" }, {
                       "Cape May-Lewes Ferry Entrance", "US 9" },
                   { }, "", 0.099000, 7, 356,
                   Maneuver::RelativeDirection::kKeepStraight,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 31, 62,
                   23, 25, 71, 75, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 1, 0, 0, 0, 0, 0, "", "", "", 0, 0, 0, 0, 5, 0);
}

void PopulateVerbalMultiCueManeuverList_0(std::list<Maneuver>& maneuvers,
                                          const std::string& country_code,
                                          const std::string& state_code) {
  maneuvers.emplace_back();
  Maneuver& maneuver1 = maneuvers.back();
  PopulateManeuver(maneuver1, country_code, state_code,
                   TripDirections_Maneuver_Type_kLeft, { "North Plum Street" },
                   { }, { }, "", 0.074000, 19, 270,
                   Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kNorth, 352, 352,
                   2, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   1, 0, 0, 1, 1, "", "", "", 0, 0, 0, 0, 4, 0);

  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kLeft, { "East Fulton Street" },
                   { }, { }, "", 0.120478, 29, 269,
                   Maneuver::RelativeDirection::kLeft,
                   TripDirections_Maneuver_CardinalDirection_kWest, 261, 263, 3,
                   5, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0, 1,
                   0, 0, 1, 1, "", "", "", 0, 0, 0, 0, 12, 0);
}

void SetExpectedManeuverInstructions(
    std::list<Maneuver>& expected_maneuvers, const string& instruction,
    const string& verbal_transition_alert_instruction,
    const string& verbal_pre_transition_instruction,
    const string& verbal_post_transition_instruction) {
  Maneuver& maneuver = expected_maneuvers.back();
  maneuver.set_instruction(instruction);
  maneuver.set_verbal_transition_alert_instruction(
      verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(
      verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(
      verbal_post_transition_instruction);
}

void SetExpectedPreviousManeuverInstructions(
    std::list<Maneuver>& expected_maneuvers, const string& instruction,
    const string& verbal_transition_alert_instruction,
    const string& verbal_pre_transition_instruction,
    const string& verbal_post_transition_instruction) {
  Maneuver& maneuver = expected_maneuvers.front();
  maneuver.set_instruction(instruction);
  maneuver.set_verbal_transition_alert_instruction(
      verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(
      verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(
      verbal_post_transition_instruction);
}

void TestBuildStartInstructions_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head east.", "",
                                  "Head east for a half mile.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

void TestBuildStartInstructions_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Head southwest on 5th Avenue.", "",
      "Head southwest on 5th Avenue for 1 tenth of a mile.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

void TestBuildStartInstructions_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Head south on North Prince Street/US 222/PA 272. Continue on US 222/PA 272.",
      "", "Head south on North Prince Street, U.S. 2 22.",
      "Continue on U.S. 2 22, Pennsylvania 2 72 for 3.2 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

void TestBuildStartInstructions_0_kilometers_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kKilometers);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Head east.", "",
                                  "Head east for 800 meters.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

void TestBuildStartInstructions_1_kilometers_en_US() {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kKilometers);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Head southwest on 5th Avenue.", "",
      "Head southwest on 5th Avenue for 200 meters.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

void TestBuildStartInstructions_2_kilometers_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kKilometers);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateStartManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateStartManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Head south on North Prince Street/US 222/PA 272. Continue on US 222/PA 272.",
      "", "Head south on North Prince Street, U.S. 2 22.",
      "Continue on U.S. 2 22, Pennsylvania 2 72 for 5.1 kilometers.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 0 "You have arrived at your destination."
// 0 "You will arrive at your destination."
// 0 "You have arrived at your destination."
void TestBuildDestinationInstructions_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_0(expected_maneuvers, country_code,
                                    state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "You have arrived at your destination.",
                                  "You will arrive at your destination.",
                                  "You have arrived at your destination.", "");

  // Add location info to trip path
  TripPath path;
  TripPath_Location* location;
  // origin
  location = path.add_location();
  // destination
  location = path.add_location();

  TryBuild(directions_options, maneuvers, expected_maneuvers,
           static_cast<EnhancedTripPath*>(&path));
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
// 1 "You will arrive at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
// 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
void TestBuildDestinationInstructions_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_1(expected_maneuvers, country_code,
                                    state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "You have arrived at 3206 Powelton Avenue.",
                                  "You will arrive at 32 o6 Powelton Avenue.",
                                  "You have arrived at 32 o6 Powelton Avenue.",
                                  "");

  // Add location info to trip path
  TripPath path;
  TripPath_Location* location;
  // origin
  location = path.add_location();
  // destination
  location = path.add_location();
  location->set_street("3206 Powelton Avenue");

  TryBuild(directions_options, maneuvers, expected_maneuvers,
           static_cast<EnhancedTripPath*>(&path));
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 2 "Your destination is on the <SOS>."
// 2 "Your destination will be on the <SOS>."
// 2 "Your destination is on the <SOS>."
void TestBuildDestinationInstructions_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_2(expected_maneuvers, country_code,
                                    state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Your destination is on the right.",
                                  "Your destination will be on the right.",
                                  "Your destination is on the right.", "");

  // Add location info to trip path
  TripPath path;
  TripPath_Location* location;
  // origin
  location = path.add_location();
  // destination
  location = path.add_location();
  location->set_side_of_street(TripPath_Location_SideOfStreet_kRight);

  TryBuild(directions_options, maneuvers, expected_maneuvers,
           static_cast<EnhancedTripPath*>(&path));
}

///////////////////////////////////////////////////////////////////////////////
// FormDestinationInstruction
// 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."
// 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> will be on the <SOS>"
// 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."
void TestBuildDestinationInstructions_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateDestinationManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateDestinationManeuverList_3(expected_maneuvers, country_code,
                                    state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Lancaster Brewing Company is on the left.",
      "Lancaster Brewing Company will be on the left.",
      "Lancaster Brewing Company is on the left.", "");

  // Add location info to trip path
  TripPath path;
  TripPath_Location* location;
  // origin
  location = path.add_location();
  // destination
  location = path.add_location();
  location->set_name("Lancaster Brewing Company");
  location->set_side_of_street(TripPath_Location_SideOfStreet_kLeft);

  TryBuild(directions_options, maneuvers, expected_maneuvers,
           static_cast<EnhancedTripPath*>(&path));
}

///////////////////////////////////////////////////////////////////////////////
// FormContinueInstruction
// 0 "Continue."
// 0 "Continue."
// 0 "Continue for <LENGTH>."
void TestBuildContinueInstructions_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateContinueManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateContinueManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Continue.",
                                  "Continue.",
                                  "Continue for 300 feet.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormContinueInstruction
// 1 "Continue on <STREET_NAMES>."
// 1 "Continue on <STREET_NAMES(1)>."
// 1 "Continue on <STREET_NAMES(2)> for <LENGTH>."
void TestBuildContinueInstructions_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateContinueManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateContinueManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Continue on 10th Avenue.",
      "Continue on 10th Avenue.",
      "Continue on 10th Avenue for 3 tenths of a mile.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 0 "Turn <RELATIVE_DIRECTION>."
// 0 "Turn <RELATIVE_DIRECTION>."
// 0 "Turn <RELATIVE_DIRECTION>."
void TestBuildTurnInstructions_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Turn left.",
                                  "Turn left.",
                                  "Turn left.",
                                  "Continue for a half mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
void TestBuildTurnInstructions_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "NY";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Turn sharp right onto Flatbush Avenue.",
                                  "Turn sharp right onto Flatbush Avenue.",
                                  "Turn sharp right onto Flatbush Avenue.",
                                  "Continue for 1 tenth of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 2 "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// 2 "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(1)>."
// 2 "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(2)>."
void TestBuildTurnInstructions_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn sharp left onto North Bond Street/US 1 Business/MD 924. Continue on MD 924.",
      "Turn sharp left onto North Bond Street.",
      "Turn sharp left onto North Bond Street, U.S. 1 Business.",
      "Continue on Maryland 9 24 for a half mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 3 "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
// 3 "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES(1)>."
// 3 "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES(2)>."
void TestBuildTurnInstructions_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "VA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateTurnManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateTurnManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Turn right onto Sunstone Drive.",
      "Turn right onto Sunstone Drive.",
      "Turn right onto Sunstone Drive. Then Turn right to stay on Sunstone Drive.",
      "Continue for 300 feet.");
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Turn right to stay on Sunstone Drive.",
                                  "Turn right to stay on Sunstone Drive.",
                                  "Turn right to stay on Sunstone Drive.",
                                  "Continue for 100 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 0 "Bear <RELATIVE_DIRECTION>."
// 0 "Bear <RELATIVE_DIRECTION>."
// 0 "Bear <RELATIVE_DIRECTION>."
void TestBuildBearInstructions_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Bear right.",
                                  "Bear right.",
                                  "Bear right.",
                                  "Continue for 60 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 1 "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES>."
// 1 "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES(1)>."
// 1 "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES(2)>."
void TestBuildBearInstructions_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Bear left onto Arlen Road.",
                                  "Bear left onto Arlen Road.",
                                  "Bear left onto Arlen Road.",
                                  "Continue for 400 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 2 "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// 2 "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(1)>."
// 2 "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES(2)>."
void TestBuildBearInstructions_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "MD";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Bear right onto Belair Road/US 1 Business. Continue on US 1 Business.",
      "Bear right onto Belair Road.",
      "Bear right onto Belair Road, U.S. 1 Business.",
      "Continue on U.S. 1 Business for 2.1 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 3 "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
// 3 "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES(1)>."
// 3 "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES(2)>."
void TestBuildBearInstructions_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "VA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateBearManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateBearManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Exit the roundabout onto Catoctin Mountain Highway/US 15. Continue on US 15.",
      "", "Exit the roundabout onto Catoctin Mountain Highway, U.S. 15.",
      "Continue on U.S. 15 for 11.4 miles.");
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Bear left to stay on US 15 South.",
                                  "Bear left to stay on U.S. 15 South.",
                                  "Bear left to stay on U.S. 15 South.",
                                  "Continue for 2.6 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 0 "Make a <RELATIVE_DIRECTION> U-turn."
// 0 "Make a <RELATIVE_DIRECTION> U-turn."
// 0 "Make a <RELATIVE_DIRECTION> U-turn."
void TestBuildUturnInstructions_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Make a left U-turn.",
                                  "Make a left U-turn.", "Make a left U-turn.",
                                  "Continue for 4 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 1 "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>."
// 1 "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES(1)>."
// 1 "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES(2)>."
void TestBuildUturnInstructions_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Make a right U-turn onto Bunker Hill Road.",
                                  "Make a right U-turn onto Bunker Hill Road.",
                                  "Make a right U-turn onto Bunker Hill Road.",
                                  "Continue for 4 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 2 "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>."
// 2 "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES(1)>."
// 2 "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES(2)>."
void TestBuildUturnInstructions_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(expected_maneuvers,
                                          "Turn right onto Bunker Hill Road.",
                                          "Turn right onto Bunker Hill Road.",
                                          "Turn right onto Bunker Hill Road.",
                                          "Continue for 2 tenths of a mile.");
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Make a left U-turn to stay on Bunker Hill Road.",
      "Make a left U-turn to stay on Bunker Hill Road.",
      "Make a left U-turn to stay on Bunker Hill Road.",
      "Continue for 2 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(1)>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(2)>."
void TestBuildUturnInstructions_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Make a left U-turn at Devonshire Road.",
                                  "Make a left U-turn at Devonshire Road.",
                                  "Make a left U-turn at Devonshire Road.",
                                  "Continue for 200 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 4 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(1)>."
// 4 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(2)> onto <STREET_NAMES(2)>."
void TestBuildUturnInstructions_4_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a left U-turn at Devonshire Road onto Jonestown Road/US 22.",
      "Make a left U-turn at Devonshire Road.",
      "Make a left U-turn at Devonshire Road onto Jonestown Road, U.S. 22.",
      "Continue for 200 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 5 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
// 3 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(1)>."
// 5 "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES(2)> to stay on <STREET_NAMES(2)>."
void TestBuildUturnInstructions_5_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateUturnManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateUturnManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Head northeast on Jonestown Road/US 22.",
      "",
      "Head northeast on Jonestown Road, U.S. 22 for 200 feet. Then Make a left U-turn at Devonshire Road.",
      "");
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a left U-turn at Devonshire Road to stay on Jonestown Road/US 22.",
      "Make a left U-turn at Devonshire Road.",
      "Make a left U-turn at Devonshire Road to stay on Jonestown Road, U.S. 22.",
      "Continue for 200 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 0 "Stay straight to take the ramp."
// 0 "Stay straight to take the ramp."
// 0 "Stay straight to take the ramp."
void TestBuildRampStraight_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_0(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Stay straight to take the ramp.",
                                  "Stay straight to take the ramp.",
                                  "Stay straight to take the ramp.",
                                  "Continue for 1.5 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 1 "Stay straight to take the <BRANCH_SIGN> ramp."
// 1 "Stay straight to take the <BRANCH_SIGN> ramp."
// 1 "Stay straight to take the <BRANCH_SIGN> ramp."
void TestBuildRampStraight_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_1(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Stay straight to take the US 322 East ramp.",
      "Stay straight to take the U.S. 3 22 East ramp.",
      "Stay straight to take the U.S. 3 22 East ramp.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
// 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
// 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
void TestBuildRampStraight_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_2(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Stay straight to take the ramp toward Hershey.",
      "Stay straight to take the ramp toward Hershey.",
      "Stay straight to take the ramp toward Hershey.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 3 "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
// 1 "Stay straight to take the <BRANCH_SIGN> ramp"
// 3 "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
void TestBuildRampStraight_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_3(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Stay straight to take the US 322 East/US 422 East/US 522 East/US 622 East ramp toward Hershey/Palmdale/Palmyra/Campbelltown.",
      "Stay straight to take the U.S. 3 22 East ramp.",
      "Stay straight to take the U.S. 3 22 East, U.S. 4 22 East ramp toward Hershey, Palmdale.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampStraightInstruction
// 4 "Stay straight to take the <NAME_SIGN> ramp."
// 3 "Stay straight to take the <NAME_SIGN> ramp."
// 4 "Stay straight to take the <NAME_SIGN> ramp."
void TestBuildRampStraight_4_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampStraightManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampStraightManeuverList_4(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Stay straight to take the Gettysburg Pike ramp.",
      "Stay straight to take the Gettysburg Pike ramp.",
      "Stay straight to take the Gettysburg Pike ramp.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
// "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
// "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
void TestBuildRamp_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the ramp on the right.",
                                  "Take the ramp on the right.",
                                  "Take the ramp on the right.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
void TestBuildRamp_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the I 95 ramp on the right.",
                                  "Take the Interstate 95 ramp on the right.",
                                  "Take the Interstate 95 ramp on the right.",
                                  "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
void TestBuildRamp_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the ramp on the left toward JFK.",
                                  "Take the ramp on the left toward JFK.",
                                  "Take the ramp on the left toward JFK.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>",
// "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
void TestBuildRamp_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take the South Conduit Avenue ramp on the left toward JFK.",
      "Take the South Conduit Avenue ramp on the left.",
      "Take the South Conduit Avenue ramp on the left toward JFK.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
// "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
void TestBuildRamp_4_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the Gettysburg Pike ramp on the right.",
                                  "Take the Gettysburg Pike ramp on the right.",
                                  "Take the Gettysburg Pike ramp on the right.",
                                  "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
// "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
// "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
void TestBuildRamp_5_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Turn right to take the ramp.",
                                  "Turn right to take the ramp.",
                                  "Turn right to take the ramp.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
void TestBuildRamp_6_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Turn left to take the PA 283 West ramp.",
      "Turn left to take the Pennsylvania 2 83 West ramp.",
      "Turn left to take the Pennsylvania 2 83 West ramp.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
// "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
// "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
void TestBuildRamp_7_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn left to take the ramp toward Harrisburg/Harrisburg International Airport.",
      "Turn left to take the ramp toward Harrisburg.",
      "Turn left to take the ramp toward Harrisburg, Harrisburg International Airport.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
// "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
// "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
void TestBuildRamp_8_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn left to take the PA 283 West ramp toward Harrisburg/Harrisburg International Airport.",
      "Turn left to take the Pennsylvania 2 83 West ramp.",
      "Turn left to take the Pennsylvania 2 83 West ramp toward Harrisburg, Harrisburg International Airport.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormRampInstruction
// "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
// "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
// "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
void TestBuildRamp_9_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateRampManeuverList_9(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateRampManeuverList_9(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Turn right to take the Gettysburg Pike ramp.",
      "Turn right to take the Gettysburg Pike ramp.",
      "Turn right to take the Gettysburg Pike ramp.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "0": "Take the exit on the <RELATIVE_DIRECTION>.",
// "0": "Take the exit on the <RELATIVE_DIRECTION>.",
// "0": "Take the exit on the <RELATIVE_DIRECTION>.",
void TestBuildExit_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the exit on the right.",
                                  "Take the exit on the right.",
                                  "Take the exit on the right.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
void TestBuildExit_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take exit 67 B-A on the right.",
                                  "Take exit 67 B-A on the right.",
                                  "Take exit 67 B-A on the right.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
void TestBuildExit_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the US 322 West exit on the right.",
                                  "Take the U.S. 3 22 West exit on the right.",
                                  "Take the U.S. 3 22 West exit on the right.",
                                  "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "3": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "3": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
void TestBuildExit_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take exit 67 B-A on the right onto US 322 West.",
      "Take exit 67 B-A on the right.",
      "Take exit 67 B-A on the right onto U.S. 3 22 West.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
void TestBuildExit_4_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take the exit on the right toward Lewistown.",
      "Take the exit on the right toward Lewistown.",
      "Take the exit on the right toward Lewistown.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "5": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "5": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
void TestBuildExit_5_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take exit 67 B-A on the right toward Lewistown.",
      "Take exit 67 B-A on the right.",
      "Take exit 67 B-A on the right toward Lewistown.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "6": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "6": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
void TestBuildExit_6_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take the US 322 West exit on the right toward Lewistown.",
      "Take the U.S. 3 22 West exit on the right.",
      "Take the U.S. 3 22 West exit on the right toward Lewistown.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "7": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
// "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
// "7": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
void TestBuildExit_7_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take exit 67 B-A on the right onto US 322 West toward Lewistown/State College.",
      "Take exit 67 B-A on the right.",
      "Take exit 67 B-A on the right onto U.S. 3 22 West toward Lewistown, State College.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.",
void TestBuildExit_8_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_8(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_8(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Take the White Marsh Boulevard exit on the left.",
      "Take the White Marsh Boulevard exit on the left.",
      "Take the White Marsh Boulevard exit on the left.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "10": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "10": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
void TestBuildExit_10_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_10(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_10(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take the White Marsh Boulevard exit on the left onto MD 43 East.",
      "Take the Maryland 43 East exit on the left.",
      "Take the White Marsh Boulevard exit on the left onto Maryland 43 East.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "12": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "12": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
void TestBuildExit_12_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_12(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_12(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take the White Marsh Boulevard exit on the left toward White Marsh.",
      "Take the exit on the left toward White Marsh.",
      "Take the White Marsh Boulevard exit on the left toward White Marsh.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitInstruction
// "14": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
// "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
// "14": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
void TestBuildExit_14_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitManeuverList_14(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitManeuverList_14(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Take the White Marsh Boulevard exit on the left onto MD 43 East toward White Marsh.",
      "Take the Maryland 43 East exit on the left.",
      "Take the White Marsh Boulevard exit on the left onto Maryland 43 East toward White Marsh.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "0": "Keep <RELATIVE_DIRECTION> at the fork.",
// "0": "Keep <RELATIVE_DIRECTION> at the fork.",
// "0": "Keep <RELATIVE_DIRECTION> at the fork.",
void TestBuildKeep_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep straight at the fork.",
                                  "Keep straight at the fork.",
                                  "Keep straight at the fork.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
void TestBuildKeep_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep right to take exit 62.",
                                  "Keep right to take exit 62.",
                                  "Keep right to take exit 62.",
                                  "Continue for 9 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
void TestBuildKeep_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep right to take I 895 South.",
                                  "Keep right to take Interstate 8 95 South.",
                                  "Keep right to take Interstate 8 95 South.",
                                  "Continue for 9 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>.",
void TestBuildKeep_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Keep right to take exit 62 onto I 895 South.",
      "Keep right to take exit 62.",
      "Keep right to take exit 62 onto Interstate 8 95 South.",
      "Continue for 9 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
void TestBuildKeep_4_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_4(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_4(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep right toward Annapolis.",
                                  "Keep right toward Annapolis.",
                                  "Keep right toward Annapolis.",
                                  "Continue for 9 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
void TestBuildKeep_5_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_5(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_5(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Keep right to take exit 62 toward Annapolis.",
      "Keep right to take exit 62.",
      "Keep right to take exit 62 toward Annapolis.", "Continue for 9 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "6": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>.",
// "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
// "6": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>.",
void TestBuildKeep_6_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_6(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_6(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Keep right to take I 895 South toward Annapolis.",
      "Keep right to take Interstate 8 95 South.",
      "Keep right to take Interstate 8 95 South toward Annapolis.",
      "Continue for 9 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepInstruction
// "7": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward <TOWARD_SIGN>."
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
// "7": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward <TOWARD_SIGN>."
void TestBuildKeep_7_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepManeuverList_7(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepManeuverList_7(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Keep right to take exit 62 onto I 895 South toward Annapolis.",
      "Keep right to take exit 62.",
      "Keep right to take exit 62 onto Interstate 8 95 South toward Annapolis.",
      "Continue for 9 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
void TestBuildKeepToStayOn_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_0(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.",
      "Continue for 14.7 miles.");
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Keep left to stay on I 95 South.",
                                  "Keep left to stay on Interstate 95 South.",
                                  "Keep left to stay on Interstate 95 South.",
                                  "Continue for 5.1 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>.",
void TestBuildKeepToStayOn_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_1(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.",
      "Continue for 14.7 miles.");
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Keep left to take exit 62 to stay on I 95 South.",
      "Keep left to stay on Interstate 95 South.",
      "Keep left to take exit 62 to stay on Interstate 95 South.",
      "Continue for 5.1 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "2": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>.",
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "2": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>.",
void TestBuildKeepToStayOn_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_2(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.",
      "Continue for 14.7 miles.");
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Keep left to stay on I 95 South toward Baltimore.",
      "Keep left to stay on Interstate 95 South.",
      "Keep left to stay on Interstate 95 South toward Baltimore.",
      "Continue for 5.1 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormKeepToStayOnInstruction
// "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
// "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
// "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
void TestBuildKeepToStayOn_3_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateKeepToStayOnManeuverList_3(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateKeepToStayOnManeuverList_3(expected_maneuvers, country_code,
                                     state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Merge onto I 95 South/John F. Kennedy Memorial Highway.", "",
      "Merge onto Interstate 95 South, John F. Kennedy Memorial Highway.",
      "Continue for 14.7 miles.");
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Keep left to take exit 62 to stay on I 95 South toward Baltimore.",
      "Keep left to stay on Interstate 95 South.",
      "Keep left to take exit 62 to stay on Interstate 95 South toward Baltimore.",
      "Continue for 5.1 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormMergeInstruction
// "0": "Merge.",
// No alert since prior maneuver is not > 2 km
// "0": "Merge.",
void TestBuildMerge_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateMergeManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateMergeManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Take the I 76 West exit on the right toward Pittsburgh.",
      "Take the Interstate 76 West exit on the right.",
      "Take the Interstate 76 West exit on the right toward Pittsburgh.", "");
  SetExpectedManeuverInstructions(expected_maneuvers, "Merge.", "", "Merge.",
                                  "Continue for 4.7 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormMergeInstruction
// "1": "Merge onto <STREET_NAMES>."
// No alert since prior maneuver is not > 2 km
// "1": "Merge onto <STREET_NAMES>."
void TestBuildMerge_1_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateMergeManeuverList_1_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateMergeManeuverList_1_1(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Take the I 76 West exit on the right toward Pittsburgh.",
      "Take the Interstate 76 West exit on the right.",
      "Take the Interstate 76 West exit on the right toward Pittsburgh.", "");
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Merge onto I 76 West/Pennsylvania Turnpike.", "",
      "Merge onto Interstate 76 West, Pennsylvania Turnpike.",
      "Continue for 4.7 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormMergeInstruction
// "1": "Merge onto <STREET_NAMES>."
// "1": "Merge onto <STREET_NAMES>."
// "1": "Merge onto <STREET_NAMES>."
void TestBuildMerge_1_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateMergeManeuverList_1_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateMergeManeuverList_1_2(expected_maneuvers, country_code, state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Take the I 76 West exit on the right toward Pittsburgh.",
      "Take the Interstate 76 West exit on the right.",
      "Take the Interstate 76 West exit on the right toward Pittsburgh.",
      "Continue for 1.3 miles.");
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Merge onto I 76 West/Pennsylvania Turnpike.",
      "Merge onto Interstate 76 West.",
      "Merge onto Interstate 76 West, Pennsylvania Turnpike.",
      "Continue for 4.7 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterRoundaboutInstruction
// "0": "Enter the roundabout.",
// "0": "Enter the roundabout.",
// "0": "Enter the roundabout.",
void TestBuildEnterRoundabout_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterRoundaboutManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterRoundaboutManeuverList_0(expected_maneuvers, country_code,
                                        state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Enter the roundabout.",
                                  "Enter the roundabout.",
                                  "Enter the roundabout.", "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterRoundaboutInstruction
// "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."
// "0": "Enter the roundabout.",
// "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."
void TestBuildEnterRoundabout_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  uint32_t roundabout_exit_count = 1;
  const std::vector<std::string> kExpectedOrdinalValues = { "1st", "2nd", "3rd",
      "4th", "5th", "6th", "7th", "8th", "9th", "10th" };

  for (auto& ordinal_value : kExpectedOrdinalValues) {
    // Configure maneuvers
    std::list<Maneuver> maneuvers;
    PopulateEnterRoundaboutManeuverList_1(maneuvers, country_code, state_code,
                                          roundabout_exit_count);

    // Configure expected maneuvers based on directions options
    std::list<Maneuver> expected_maneuvers;
    PopulateEnterRoundaboutManeuverList_1(expected_maneuvers, country_code,
                                          state_code, roundabout_exit_count++);
    SetExpectedManeuverInstructions(
        expected_maneuvers,
        "Enter the roundabout and take the " + ordinal_value + " exit.",
        "Enter the roundabout.",
        "Enter the roundabout and take the " + ordinal_value + " exit.", "");

    TryBuild(directions_options, maneuvers, expected_maneuvers);
  }
}

///////////////////////////////////////////////////////////////////////////////
// FormExitRoundaboutInstruction
// "0": "Exit the roundabout.",
// No verbal alert
// "0": "Exit the roundabout.",
void TestBuildExitRoundabout_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitRoundaboutManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitRoundaboutManeuverList_0(expected_maneuvers, country_code,
                                       state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Exit the roundabout.",
                                  "", "Exit the roundabout.",
                                  "Continue for 6 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitRoundaboutInstruction
// "1": "Exit the roundabout onto <STREET_NAMES>.",
// No verbal alert
// "1": "Exit the roundabout onto <STREET_NAMES>.",
void TestBuildExitRoundabout_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitRoundaboutManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitRoundaboutManeuverList_1(expected_maneuvers, country_code,
                                       state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers, "Exit the roundabout onto Philadelphia Road/MD 7.",
      "", "Exit the roundabout onto Philadelphia Road, Maryland 7.",
      "Continue for 6 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitRoundaboutInstruction
// "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// No verbal alert
// "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>.",
void TestBuildExitRoundabout_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitRoundaboutManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitRoundaboutManeuverList_2(expected_maneuvers, country_code,
                                       state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Exit the roundabout onto Catoctin Mountain Highway/US 15. Continue on US 15.",
      "", "Exit the roundabout onto Catoctin Mountain Highway, U.S. 15.",
      "Continue on U.S. 15 for 11.4 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterFerryInstruction
// "0": "Take the Ferry.",
// "0": "Take the Ferry.",
// "0": "Take the Ferry.",
void TestBuildEnterFerry_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterFerryManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterFerryManeuverList_0(expected_maneuvers, country_code,
                                   state_code);
  SetExpectedManeuverInstructions(expected_maneuvers, "Take the Ferry.",
                                  "Take the Ferry.", "Take the Ferry.",
                                  "Continue for 9 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterFerryInstruction
// "1": "Take the <STREET_NAMES>.",
// "1": "Take the <STREET_NAMES>.",
// "1": "Take the <STREET_NAMES>.",
void TestBuildEnterFerry_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterFerryManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterFerryManeuverList_1(expected_maneuvers, country_code,
                                   state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the Millersburg Ferry.",
                                  "Take the Millersburg Ferry.",
                                  "Take the Millersburg Ferry.",
                                  "Continue for 9 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormEnterFerryInstruction
// "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
// "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
// "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
void TestBuildEnterFerry_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateEnterFerryManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateEnterFerryManeuverList_2(expected_maneuvers, country_code,
                                   state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Take the Bridgeport - Port Jefferson Ferry.",
                                  "Take the Bridgeport - Port Jefferson Ferry.",
                                  "Take the Bridgeport - Port Jefferson Ferry.",
                                  "Continue for 17.2 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitFerryInstruction
// "0": "Head <CARDINAL_DIRECTION>.",
// "0": "Head <CARDINAL_DIRECTION>.",
// "0": "Head <CARDINAL_DIRECTION>.",
void TestBuildExitFerry_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Head southeast.",
                                  "Head southeast.",
                                  "Head southeast.",
                                  "Continue for 200 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitFerryInstruction
// "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
// "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
// "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
void TestBuildExitFerry_1_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_1(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Head west on Ferry Lane.",
                                  "Head west on Ferry Lane.",
                                  "Head west on Ferry Lane.",
                                  "Continue for 4 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormExitFerryInstruction
// "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."
// "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."
void TestBuildExitFerry_2_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateExitFerryManeuverList_2(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateExitFerryManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Head northeast on Cape May-Lewes Ferry Entrance/US 9. Continue on US 9.",
                                  "Head northeast on Cape May-Lewes Ferry Entrance.",
                                  "Head northeast on Cape May-Lewes Ferry Entrance, U.S. 9.",
                                  "Continue on U.S. 9 for 300 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTransitConnectionStartInstruction
// "0": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// "0": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."
// "0": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."
void TestBuildTransitConnectionStart_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
//  PopulateTransitConnectionStartManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
//  PopulateTransitConnectionStartManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Head northeast on Cape May-Lewes Ferry Entrance/US 9. Continue on US 9.",
                                  "Head northeast on Cape May-Lewes Ferry Entrance.",
                                  "Head northeast on Cape May-Lewes Ferry Entrance, U.S. 9.",
                                  "Continue on U.S. 9 for 300 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormVerbalMultiCue
// 0 "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>"
void TestBuildVerbalMultiCue_0_miles_en_US() {
  std::string country_code = "US";
  std::string state_code = "PA";

  // Configure directions options
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  // Configure maneuvers
  std::list<Maneuver> maneuvers;
  PopulateVerbalMultiCueManeuverList_0(maneuvers, country_code, state_code);

  // Configure expected maneuvers based on directions options
  std::list<Maneuver> expected_maneuvers;
  PopulateVerbalMultiCueManeuverList_0(expected_maneuvers, country_code,
                                       state_code);
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Turn left onto North Plum Street.",
      "Turn left onto North Plum Street.",
      "Turn left onto North Plum Street. Then Turn left onto East Fulton Street.",
      "Continue for 200 feet.");
  SetExpectedManeuverInstructions(expected_maneuvers,
                                  "Turn left onto East Fulton Street.",
                                  "Turn left onto East Fulton Street.",
                                  "Turn left onto East Fulton Street.",
                                  "Continue for 400 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

// FormDestinati onInstruction
Maneuver CreateVerbalPostManeuver(vector<std::string> street_names,
                                  float kilometers,
                                  TripDirections_Maneuver_Type type =
                                      TripDirections_Maneuver_Type_kRight) {

  Maneuver maneuver;
  maneuver.set_street_names(street_names);
  maneuver.set_length(kilometers);
  maneuver.set_type(type);

  return maneuver;
}

void TryFormVerbalPostTransitionInstruction(NarrativeBuilderTest& nbt,
                                            Maneuver maneuver,
                                            bool include_street_names,
                                            std::string expected) {
  std::string instruction = nbt.FormVerbalPostTransitionInstruction(
      maneuver, include_street_names);
  if (instruction != expected) {
    throw std::runtime_error(
        "Incorrect FormVerbalPostTransitionInstruction - EXPECTED: " + expected
            + "  |  FORMED: " + instruction);
  }
}

void TestFormVerbalPostTransitionInstruction() {
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kKilometers);
  directions_options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(
      directions_options);

  NarrativeBuilderTest nbt_km(directions_options, dictionary);

  // Verify kilometer whole number
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 4.0f), false,
      "Continue for 4 kilometers.");

  // Verify kilometers round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 3.54056f), false,
      "Continue for 3.5 kilometers.");

  // Verify kilometers round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 3.86243f), false,
      "Continue for 3.9 kilometers.");

  // Verify kilometers street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 3.86243f), true,
      "Continue on Main Street for 3.9 kilometers.");

  // Verify 1 kilometer round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 1.04f), false,
      "Continue for 1 kilometer.");

  // Verify 1 kilometer round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.95f), false,
      "Continue for 1 kilometer.");

  // Verify 1 kilometer street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 1.0f), true,
      "Continue on Main Street for 1 kilometer.");

  // Verify a half kilometer round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.54f), false,
      "Continue for a half kilometer.");

  // Verify a half kilometer round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.45f), false,
      "Continue for a half kilometer.");

  // Verify a half kilometer street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.5f), true,
      "Continue on Main Street for a half kilometer.");

  // Verify 900 meters round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.94f), false,
      "Continue for 900 meters.");

  // Verify 900 meters round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.85f), false,
      "Continue for 900 meters.");

  // Verify 900 meters street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.9f), true,
      "Continue on Main Street for 900 meters.");

  // Verify 400 meters round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.44f), false,
      "Continue for 400 meters.");

  // Verify 400 meters round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.35f), false,
      "Continue for 400 meters.");

  // Verify 400 meters street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.4f), true,
      "Continue on Main Street for 400 meters.");

  // Verify 100 meters round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.14f), false,
      "Continue for 100 meters.");

  // Verify 100 meters round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.095f), false,
      "Continue for 100 meters.");

  // Verify 100 meters street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.1f), true,
      "Continue on Main Street for 100 meters.");

  // Verify 90 meters round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.094f), false,
      "Continue for 90 meters.");

  // Verify 90 meters round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.085f), false,
      "Continue for 90 meters.");

  // Verify 90 meters street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.09f), true,
      "Continue on Main Street for 90 meters.");

  // Verify 30 meters round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.034f), false,
      "Continue for 30 meters.");

  // Verify 30 meters round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.025f), false,
      "Continue for 30 meters.");

  // Verify 30 meters street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.03f), true,
      "Continue on Main Street for 30 meters.");

  // Verify 10 meters round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.012f), false,
      "Continue for 10 meters.");

  // Verify 10 meters round up
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.0096f), false,
      "Continue for 10 meters.");

  // Verify 10 meters street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.01f), true,
      "Continue on Main Street for 10 meters.");

  // Verify less than 10 meters round down
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.0094f), false,
      "Continue for less than 10 meters.");

  // Verify less than 10 meters
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.0088f), false,
      "Continue for less than 10 meters.");

  // Verify less than 10 meters street name
  TryFormVerbalPostTransitionInstruction(
      nbt_km, CreateVerbalPostManeuver( { "Main Street" }, 0.005f), true,
      "Continue on Main Street for less than 10 meters.");

  /////////////////////////////////////////////////////////////////////////////

  directions_options.set_units(DirectionsOptions_Units_kMiles);

  NarrativeBuilderTest nbt_mi(directions_options, dictionary);

  // Verify mile whole number
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 4.828032f), false,
      "Continue for 3 miles.");

  // Verify miles round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 3.604931f), false,
      "Continue for 2.2 miles.");

  // Verify miles round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 3.637117f), false,
      "Continue for 2.3 miles.");

  // Verify miles street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 3.637117f), true,
      "Continue on Main Street for 2.3 miles.");

  // Verify 1 mile round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 1.657624f), false,
      "Continue for 1 mile.");

  // Verify 1 mile round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 1.561064f), false,
      "Continue for 1 mile.");

  // Verify 1 mile street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 1.60934f), true,
      "Continue on Main Street for 1 mile.");

  // Verify half mile round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.8368589f), false,
      "Continue for a half mile.");

  // Verify half mile round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.7724851f), false,
      "Continue for a half mile.");

  // Verify half mile street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.804672f), true,
      "Continue on Main Street for a half mile.");

  // Verify 9 tenths of a mile round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 1.480596f), false,
      "Continue for 9 tenths of a mile.");

  // Verify 9 tenths of a mile round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 1.416223f), false,
      "Continue for 9 tenths of a mile.");

  // Verify 9 tenths of a mile street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 1.44841f), true,
      "Continue on Main Street for 9 tenths of a mile.");

  // Verify 4 tenths of a mile round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.675924f), false,
      "Continue for 4 tenths of a mile.");

  // Verify 4 tenths of a mile round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.611551f), false,
      "Continue for 4 tenths of a mile.");

  // Verify 4 tenths of a mile street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.643738f), true,
      "Continue on Main Street for 4 tenths of a mile.");

  // Verify 1 tenth of a mile round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.193121f), false,
      "Continue for 1 tenth of a mile.");

  // Verify 1 tenth of a mile round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.158496f), false,
      "Continue for 1 tenth of a mile.");

  // Verify 1 tenth of a mile street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.160934f), true,
      "Continue on Main Street for 1 tenth of a mile.");

  // Verify 500 feet round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.155448f), false,
      "Continue for 500 feet.");

  // Verify 500 feet round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.149352f), false,
      "Continue for 500 feet.");

  // Verify 500 feet street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.1524f), true,
      "Continue on Main Street for 500 feet.");

  // Verify 100 feet round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.036576f), false,
      "Continue for 100 feet.");

  // Verify 100 feet round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.028956f), false,
      "Continue for 100 feet.");

  // Verify 100 feet street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.03048f), true,
      "Continue on Main Street for 100 feet.");

  // Verify 90 feet round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.0283464f), false,
      "Continue for 90 feet.");

  // Verify 90 feet round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.0268224f), false,
      "Continue for 90 feet.");

  // Verify 90 feet street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.027432f), true,
      "Continue on Main Street for 90 feet.");

  // Verify 10 feet round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.0036576f), false,
      "Continue for 10 feet.");

  // Verify 10 feet round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.00292608f), false,
      "Continue for 10 feet.");

  // Verify 10 feet street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.003048f), true,
      "Continue on Main Street for 10 feet.");

  // Verify less than 10 feet round down
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.00280416f), false,
      "Continue for less than 10 feet.");

  // Verify less than 10 feet round up
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.00268224f), false,
      "Continue for less than 10 feet.");

  // Verify less than 10 feet street name
  TryFormVerbalPostTransitionInstruction(
      nbt_mi, CreateVerbalPostManeuver( { "Main Street" }, 0.001524f), true,
      "Continue on Main Street for less than 10 feet.");

}

Maneuver CreateSignManeuver(TripDirections_Maneuver_Type type,
                            Maneuver::RelativeDirection relative_direction,
                            vector<std::string> exit_numbers,
                            vector<std::string> exit_branches,
                            vector<std::string> exit_towards,
                            vector<std::string> exit_names) {

  Maneuver maneuver;
  maneuver.set_type(type);
  maneuver.set_begin_relative_direction(relative_direction);
  auto* signs = maneuver.mutable_signs();
  auto* exit_number_list = signs->mutable_exit_number_list();
  auto* exit_branch_list = signs->mutable_exit_branch_list();
  auto* exit_toward_list = signs->mutable_exit_toward_list();
  auto* exit_name_list = signs->mutable_exit_name_list();

  // Process exit numbers
  for (auto& exit_number : exit_numbers) {
    exit_number_list->emplace_back(exit_number);
  }

  // Process exit branches
  for (auto& exit_branch : exit_branches) {
    exit_branch_list->emplace_back(exit_branch);
  }

  // Process exit numbers
  for (auto& exit_toward : exit_towards) {
    exit_toward_list->emplace_back(exit_toward);
  }

  // Process exit numbers
  for (auto& exit_name : exit_names) {
    exit_name_list->emplace_back(exit_name);
  }

  return maneuver;
}

void TryFormRampStraightInstruction(NarrativeBuilderTest& nbt,
                                    Maneuver maneuver, std::string expected) {
  std::string instruction = nbt.FormRampStraightInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormRampStraightInstruction");
}

void TestFormRampStraightInstruction() {
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(
      directions_options);

  NarrativeBuilderTest nbt(directions_options, dictionary);

  // phrase_id = 0
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, { },
                         { }, { }),
      "Stay straight to take the ramp.");

  // phrase_id = 1
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Stay straight to take the I 95 South ramp.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, {
                             "I 95 South" },
                         { }, { "Gettysburg Pike" }),
      "Stay straight to take the I 95 South ramp.");

  // phrase_id = 2
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, { }, {
                             "Baltimore" },
                         { "Gettysburg Pike" }),
      "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 3
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward exists
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { "Gettysburg Pike" }),
      "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 4
  TryFormRampStraightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, { },
                         { }, { "Gettysburg Pike" }),
      "Stay straight to take the Gettysburg Pike ramp.");

}

void TryFormRampRightInstruction(NarrativeBuilderTest& nbt, Maneuver maneuver,
                                 std::string expected) {
  std::string instruction = nbt.FormRampInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormRampRightInstruction");
}

void TestFormRampRightInstruction() {
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(
      directions_options);

  NarrativeBuilderTest nbt(directions_options, dictionary);

  // phrase_id = 0
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { }),
      "Take the ramp on the right.");

  // phrase_id = 1
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South ramp on the right.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the right.");

  // phrase_id = 2
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the ramp on the right toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { "Gettysburg Pike" }),
      "Take the ramp on the right toward Baltimore.");

  // phrase_id = 3
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 4
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike ramp on the right.");

  // phrase_id = 5
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, { }, { },
                         { }),
      "Turn right to take the ramp.");

  // phrase_id = 6
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Turn right to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Turn right to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Turn right to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, { }, { }, {
                             "Gettysburg Pike" }),
      "Turn right to take the Gettysburg Pike ramp.");

}

void TryFormRampLeftInstruction(NarrativeBuilderTest& nbt, Maneuver maneuver,
                                std::string expected) {
  std::string instruction = nbt.FormRampInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormRampLeftInstruction");
}

void TestFormRampLeftInstruction() {
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(
      directions_options);

  NarrativeBuilderTest nbt(directions_options, dictionary);

  // phrase_id = 0
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { }),
      "Take the ramp on the left.");

  // phrase_id = 1
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South ramp on the left.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the left.");

  // phrase_id = 2
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the ramp on the left toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { "Gettysburg Pike" }),
      "Take the ramp on the left toward Baltimore.");

  // phrase_id = 3
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 4
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike ramp on the left.");

  // phrase_id = 5
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { }, { }, { },
                         { }),
      "Turn left to take the ramp.");

  // phrase_id = 6
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { "I 95 South" }, { }, { }),
      "Turn left to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { }, { }, {
                             "Baltimore" },
                         { }),
      "Turn left to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Turn left to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { }, { }, { }, {
                             "Gettysburg Pike" }),
      "Turn left to take the Gettysburg Pike ramp.");

}

void TryFormExitRightInstruction(NarrativeBuilderTest& nbt, Maneuver maneuver,
                                 std::string expected) {
  std::string instruction = nbt.FormExitInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormExitRightInstruction");
}

void TestFormExitRightInstruction() {
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(
      directions_options);

  NarrativeBuilderTest nbt(directions_options, dictionary);

  // phrase_id = 0
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { }),
      "Take the exit on the right.");

  // phrase_id = 1
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { }, { }),
      "Take exit 67A on the right.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { }, { "Gettysburg Pike" }),
      "Take exit 67A on the right.");

  // phrase_id = 2
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South exit on the right.");

  // phrase_id = 3
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" }, {
                             "I 95 South" },
                         { }, { }),
      "Take exit 67A on the right onto I 95 South.");

  // phrase_id = 4
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the exit on the right toward Baltimore.");

  // phrase_id = 5
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { "Baltimore" }, { }),
      "Take exit 67A on the right toward Baltimore.");

  // phrase_id = 6
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South exit on the right toward Baltimore.");

  // phrase_id = 7
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take exit 67A on the right onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right.");

  // phrase_id = 10
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right onto US 15.");

  // phrase_id = 12
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitRightInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { },
                         { "US 15" }, { "Harrisburg", "Gettysburg" }, {
                             "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right onto US 15 toward Harrisburg/Gettysburg.");

}

void TryFormExitLeftInstruction(NarrativeBuilderTest& nbt, Maneuver maneuver,
                                std::string expected) {
  std::string instruction = nbt.FormExitInstruction(maneuver);
  if (instruction != expected)
    throw std::runtime_error("Incorrect FormExitLeftInstruction");
}

void TestFormExitLeftInstruction() {
  DirectionsOptions directions_options;
  directions_options.set_units(DirectionsOptions_Units_kMiles);
  directions_options.set_language("en-US");

  const NarrativeDictionary& dictionary = GetNarrativeDictionary(
      directions_options);

  NarrativeBuilderTest nbt(directions_options, dictionary);

  // phrase_id = 0
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { }),
      "Take the exit on the left.");

  // phrase_id = 1
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { }, { }),
      "Take exit 67A on the left.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { }, { "Gettysburg Pike" }),
      "Take exit 67A on the left.");

  // phrase_id = 2
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South exit on the left.");

  // phrase_id = 3
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, {
                             "I 95 South" },
                         { }, { }),
      "Take exit 67A on the left onto I 95 South.");

  // phrase_id = 4
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the exit on the left toward Baltimore.");

  // phrase_id = 5
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { "Baltimore" }, { }),
      "Take exit 67A on the left toward Baltimore.");

  // phrase_id = 6
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South exit on the left toward Baltimore.");

  // phrase_id = 7
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take exit 67A on the left onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left.");

  // phrase_id = 10
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15.");

  // phrase_id = 12
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitLeftInstruction(
      nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { },
                         { "US 15" }, { "Harrisburg", "Gettysburg" }, {
                             "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15 toward Harrisburg/Gettysburg.");

}

}

int main() {
  test::suite suite("narrativebuilder");

  // FormRampStraightInstruction
  suite.test(TEST_CASE(TestFormRampStraightInstruction));

  // FormRampRightInstruction
  suite.test(TEST_CASE(TestFormRampRightInstruction));

  // FormRampLeftInstruction
  suite.test(TEST_CASE(TestFormRampLeftInstruction));

  // FormExitRightInstruction
  suite.test(TEST_CASE(TestFormExitRightInstruction));

  // FormExitLeftInstruction
  suite.test(TEST_CASE(TestFormExitLeftInstruction));

  // FormVerbalPostTransitionInstruction
  suite.test(TEST_CASE(TestFormVerbalPostTransitionInstruction));

  // BuildStartInstructions_0_miles_en_US
  suite.test(TEST_CASE(TestBuildStartInstructions_0_miles_en_US));

  // BuildStartInstructions_1_miles_en_US
  suite.test(TEST_CASE(TestBuildStartInstructions_1_miles_en_US));

  // BuildStartInstructions_2_miles_en_US
  suite.test(TEST_CASE(TestBuildStartInstructions_2_miles_en_US));

  // BuildStartInstructions_0_kilometers_en_US
  suite.test(TEST_CASE(TestBuildStartInstructions_0_kilometers_en_US));

  // BuildStartInstructions_1_kilometers_en_US
  suite.test(TEST_CASE(TestBuildStartInstructions_1_kilometers_en_US));

  // BuildStartInstructions_2_kilometers_en_US
  suite.test(TEST_CASE(TestBuildStartInstructions_2_kilometers_en_US));

  // BuildDestinationInstructions_0_miles_en_US
  suite.test(TEST_CASE(TestBuildDestinationInstructions_0_miles_en_US));

  // BuildDestinationInstructions_1_miles_en_US
  suite.test(TEST_CASE(TestBuildDestinationInstructions_1_miles_en_US));

  // BuildDestinationInstructions_2_miles_en_US
  suite.test(TEST_CASE(TestBuildDestinationInstructions_2_miles_en_US));

  // BuildDestinationInstructions_3_miles_en_US
  suite.test(TEST_CASE(TestBuildDestinationInstructions_3_miles_en_US));

  // BuildContinueInstructions_0_miles_en_US
  suite.test(TEST_CASE(TestBuildContinueInstructions_0_miles_en_US));

  // BuildContinueInstructions_1_miles_en_US
  suite.test(TEST_CASE(TestBuildContinueInstructions_1_miles_en_US));

  // BuildTurnInstructions_0_miles_en_US
  suite.test(TEST_CASE(TestBuildTurnInstructions_0_miles_en_US));

  // BuildTurnInstructions_1_miles_en_US
  suite.test(TEST_CASE(TestBuildTurnInstructions_1_miles_en_US));

  // BuildTurnInstructions_2_miles_en_US
  suite.test(TEST_CASE(TestBuildTurnInstructions_2_miles_en_US));

  // BuildTurnInstructions_3_miles_en_US
  suite.test(TEST_CASE(TestBuildTurnInstructions_3_miles_en_US));

  // BuildBearInstructions_0_miles_en_US
  suite.test(TEST_CASE(TestBuildBearInstructions_0_miles_en_US));

  // BuildBearInstructions_1_miles_en_US
  suite.test(TEST_CASE(TestBuildBearInstructions_1_miles_en_US));

  // BuildBearInstructions_2_miles_en_US
  suite.test(TEST_CASE(TestBuildBearInstructions_2_miles_en_US));

  // BuildBearInstructions_3_miles_en_US
  suite.test(TEST_CASE(TestBuildBearInstructions_3_miles_en_US));

  // BuildUturnInstructions_0_miles_en_US
  suite.test(TEST_CASE(TestBuildUturnInstructions_0_miles_en_US));

  // BuildUturnInstructions_1_miles_en_US
  suite.test(TEST_CASE(TestBuildUturnInstructions_1_miles_en_US));

  // BuildUturnInstructions_2_miles_en_US
  suite.test(TEST_CASE(TestBuildUturnInstructions_2_miles_en_US));

  // BuildUturnInstructions_3_miles_en_US
  suite.test(TEST_CASE(TestBuildUturnInstructions_3_miles_en_US));

  // BuildUturnInstructions_4_miles_en_US
  suite.test(TEST_CASE(TestBuildUturnInstructions_4_miles_en_US));

  // BuildUturnInstructions_5_miles_en_US
  suite.test(TEST_CASE(TestBuildUturnInstructions_5_miles_en_US));

  // BuildRampStraight_0_miles_en_US
  suite.test(TEST_CASE(TestBuildRampStraight_0_miles_en_US));

  // BuildRampStraight_1_miles_en_US
  suite.test(TEST_CASE(TestBuildRampStraight_1_miles_en_US));

  // BuildRampStraight_2_miles_en_US
  suite.test(TEST_CASE(TestBuildRampStraight_2_miles_en_US));

  // BuildRampStraight_3_miles_en_US
  suite.test(TEST_CASE(TestBuildRampStraight_3_miles_en_US));

  // BuildRampStraight_4_miles_en_US
  suite.test(TEST_CASE(TestBuildRampStraight_4_miles_en_US));

  // BuildRamp_0_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_0_miles_en_US));

  // BuildRamp_1_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_1_miles_en_US));

  // BuildRamp_2_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_2_miles_en_US));

  // BuildRamp_3_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_3_miles_en_US));

  // BuildRamp_4_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_4_miles_en_US));

  // BuildRamp_5_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_5_miles_en_US));

  // BuildRamp_6_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_6_miles_en_US));

  // BuildRamp_7_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_7_miles_en_US));

  // BuildRamp_8_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_8_miles_en_US));

  // BuildRamp_9_miles_en_US
  suite.test(TEST_CASE(TestBuildRamp_9_miles_en_US));

  // BuildExit_0_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_0_miles_en_US));

  // BuildExit_1_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_1_miles_en_US));

  // BuildExit_2_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_2_miles_en_US));

  // BuildExit_3_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_3_miles_en_US));

  // BuildExit_4_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_4_miles_en_US));

  // BuildExit_5_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_5_miles_en_US));

  // BuildExit_6_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_6_miles_en_US));

  // BuildExit_7_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_7_miles_en_US));

  // BuildExit_8_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_8_miles_en_US));

  // BuildExit_10_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_10_miles_en_US));

  // BuildExit_12_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_12_miles_en_US));

  // BuildExit_14_miles_en_US
  suite.test(TEST_CASE(TestBuildExit_14_miles_en_US));

  // BuildKeep_0_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_0_miles_en_US));

  // BuildKeep_1_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_1_miles_en_US));

  // BuildKeep_2_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_2_miles_en_US));

  // BuildKeep_3_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_3_miles_en_US));

  // BuildKeep_4_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_4_miles_en_US));

  // BuildKeep_5_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_5_miles_en_US));

  // BuildKeep_6_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_6_miles_en_US));

  // BuildKeep_7_miles_en_US
  suite.test(TEST_CASE(TestBuildKeep_7_miles_en_US));

  // BuildKeepToStayOn_0_miles_en_US
  suite.test(TEST_CASE(TestBuildKeepToStayOn_0_miles_en_US));

  // BuildKeepToStayOn_1_miles_en_US
  suite.test(TEST_CASE(TestBuildKeepToStayOn_1_miles_en_US));

  // BuildKeepToStayOn_2_miles_en_US
  suite.test(TEST_CASE(TestBuildKeepToStayOn_2_miles_en_US));

  // BuildKeepToStayOn_3_miles_en_US
  suite.test(TEST_CASE(TestBuildKeepToStayOn_3_miles_en_US));

  // BuildMerge_0_miles_en_US
  suite.test(TEST_CASE(TestBuildMerge_0_miles_en_US));

  // BuildMerge_1_1_miles_en_US
  suite.test(TEST_CASE(TestBuildMerge_1_1_miles_en_US));

  // BuildMerge_1_2_miles_en_US
  suite.test(TEST_CASE(TestBuildMerge_1_2_miles_en_US));

  // BuildEnterRoundabout_0_miles_en_US
  suite.test(TEST_CASE(TestBuildEnterRoundabout_0_miles_en_US));

  // BuildEnterRoundabout_1_miles_en_US
  suite.test(TEST_CASE(TestBuildEnterRoundabout_1_miles_en_US));

  // BuildExitRoundabout_0_miles_en_US
  suite.test(TEST_CASE(TestBuildExitRoundabout_0_miles_en_US));

  // BuildExitRoundabout_1_miles_en_US
  suite.test(TEST_CASE(TestBuildExitRoundabout_1_miles_en_US));

  // BuildExitRoundabout_2_miles_en_US
  suite.test(TEST_CASE(TestBuildExitRoundabout_2_miles_en_US));

  // BuildEnterFerry_0_miles_en_US
  suite.test(TEST_CASE(TestBuildEnterFerry_0_miles_en_US));

  // BuildEnterFerry_1_miles_en_US
  suite.test(TEST_CASE(TestBuildEnterFerry_1_miles_en_US));

  // BuildEnterFerry_2_miles_en_US
  suite.test(TEST_CASE(TestBuildEnterFerry_2_miles_en_US));

  // BuildExitFerry_0_miles_en_US
  suite.test(TEST_CASE(TestBuildExitFerry_0_miles_en_US));

  // BuildExitFerry_1_miles_en_US
  suite.test(TEST_CASE(TestBuildExitFerry_1_miles_en_US));

  // BuildExitFerry_2_miles_en_US
  suite.test(TEST_CASE(TestBuildExitFerry_2_miles_en_US));

  // BuildTransitConnectionStart_0_miles_en_US
//  suite.test(TEST_CASE(TestBuildTransitConnectionStart_0_miles_en_US));

  // BuildVerbalMultiCue_0_miles_en_US
  suite.test(TEST_CASE(TestBuildVerbalMultiCue_0_miles_en_US));

  return suite.tear_down();
}

