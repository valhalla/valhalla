#include <regex>

#include "test.h"
#include "valhalla/odin/maneuver.h"
#include "valhalla/odin/sign.h"
#include "valhalla/odin/signs.h"
#include "valhalla/odin/util.h"
#include "valhalla/odin/narrative_builder_factory.h"
#include "valhalla/odin/narrative_dictionary.h"
#include "valhalla/odin/narrativebuilder.h"
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/baldr/verbal_text_formatter_factory.h>

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
      Maneuver& maneuver, DirectionsOptions_Units units,
      bool include_street_names = false,
      uint32_t element_max_count = kVerbalPostElementMaxCount,
      std::string delim = kVerbalDelim) {
    return NarrativeBuilder::FormVerbalPostTransitionInstruction(
        maneuver, units, include_street_names, element_max_count, delim);
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
    uint32_t internal_right_turn_count = 0,
    uint32_t internal_left_turn_count = 0,
    uint32_t roundabout_exit_count = 0, bool fork = false,
    bool begin_intersecting_edge_name_consistency = false,
    bool intersecting_forward_edge = false,
    std::string verbal_transition_alert_instruction = "",
    std::string verbal_pre_transition_instruction = "",
    std::string verbal_post_transition_instruction = "", bool tee = false) {

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
  maneuver.set_begin_intersecting_edge_name_consistency(begin_intersecting_edge_name_consistency);
  maneuver.set_intersecting_forward_edge(intersecting_forward_edge);
  maneuver.set_verbal_transition_alert_instruction(verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(verbal_post_transition_instruction);
  maneuver.set_tee(tee);
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
                   TripDirections_Maneuver_Type_kDestination, { }, { }, { },
                   "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
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
                   TripDirections_Maneuver_Type_kDestinationLeft, { }, { },
                   { }, "", 0.000000, 0, 0, Maneuver::RelativeDirection::kNone,
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
                   { }, { }, 1, 0, 0, 0, 1, 0, "", "", "", 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kSlightLeft, { "US 15 South" },
                   { }, { }, "", 4.137000, 232, 349,
                   Maneuver::RelativeDirection::kKeepLeft,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 193, 197,
                   187, 197, 1805, 1878, 0, 0, 0, 0, 0, 0, 0, 1, 0, { }, { },
                   { }, { }, 0, 0, 0, 0, 0, 1, "", "", "", 0);
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
                   { }, 1, 0, 0, 0, 1, 1, "", "", "", 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnLeft,
                   { "Bunker Hill Road" }, { }, { }, "", 0.287000, 25, 180,
                   Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kSouth, 157, 155,
                   3, 4, 46, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { },
                   0, 0, 0, 0, 0, 0, "", "", "", 0);
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
                   { }, { }, "", 0.062923, 0, 0,
                   Maneuver::RelativeDirection::kNone,
                   TripDirections_Maneuver_CardinalDirection_kNorthEast, 36, 32,
                   0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { }, { }, 0,
                   0, 0, 0, 1, 0, "", "", "", 0);
  maneuvers.emplace_back();
  Maneuver& maneuver2 = maneuvers.back();
  PopulateManeuver(maneuver2, country_code, state_code,
                   TripDirections_Maneuver_Type_kUturnLeft, { "Jonestown Road",
                       "US 22" },
                   { }, { "Devonshire Road" }, "", 0.072697, 47, 180,
                   Maneuver::RelativeDirection::KReverse,
                   TripDirections_Maneuver_CardinalDirection_kSouthWest, 212,
                   221, 1, 3, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, { }, { }, { },
                   { }, 0, 1, 0, 0, 1, 0, "", "", "", 0);
}

void SetExpectedManeuverInstructions(
    std::list<Maneuver>& expected_maneuvers,
    const string& instruction,
    const string& verbal_transition_alert_instruction,
    const string& verbal_pre_transition_instruction,
    const string& verbal_post_transition_instruction) {
  Maneuver& maneuver = expected_maneuvers.back();
  maneuver.set_instruction(instruction);
  maneuver.set_verbal_transition_alert_instruction(verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(verbal_post_transition_instruction);
}

void SetExpectedPreviousManeuverInstructions(
    std::list<Maneuver>& expected_maneuvers,
    const string& instruction,
    const string& verbal_transition_alert_instruction,
    const string& verbal_pre_transition_instruction,
    const string& verbal_post_transition_instruction) {
  Maneuver& maneuver = expected_maneuvers.front();
  maneuver.set_instruction(instruction);
  maneuver.set_verbal_transition_alert_instruction(verbal_transition_alert_instruction);
  maneuver.set_verbal_pre_transition_instruction(verbal_pre_transition_instruction);
  maneuver.set_verbal_post_transition_instruction(verbal_post_transition_instruction);
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Head east.",
      "",
      "Head east for a half mile.",
      "");

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
      expected_maneuvers,
      "Head southwest on 5th Avenue.",
      "",
      "Head southwest on 5th Avenue for 1 tenth of a mile.",
      "");

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
      "",
      "Head south on North Prince Street, U.S. 2 22.",
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Head east.",
      "",
      "Head east for 800 meters.",
      "");

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
      expected_maneuvers,
      "Head southwest on 5th Avenue.",
      "",
      "Head southwest on 5th Avenue for 200 meters.",
      "");

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
      "",
      "Head south on North Prince Street, U.S. 2 22.",
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
  PopulateDestinationManeuverList_0(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "You have arrived at your destination.",
      "You will arrive at your destination.",
      "You have arrived at your destination.",
      "");

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
  PopulateDestinationManeuverList_1(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
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
  PopulateDestinationManeuverList_2(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Your destination is on the right.",
      "Your destination will be on the right.",
      "Your destination is on the right.",
      "");

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
  PopulateDestinationManeuverList_3(expected_maneuvers, country_code, state_code);
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Lancaster Brewing Company is on the left.",
      "Lancaster Brewing Company will be on the left.",
      "Lancaster Brewing Company is on the left.",
      "");

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
// 0 "Continue for <DISTANCE>."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Continue.",
      "Continue.",
      "Continue for 300 feet.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormContinueInstruction
// 1 "Continue on <STREET_NAMES>."
// 1 "Continue on <STREET_NAMES(1)>."
// 1 "Continue on <STREET_NAMES(2)> for <DISTANCE>."
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
      "Continue on 10th Avenue for 3 tenths of a mile.",
      "");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 0 "Turn <FormTurnTypeInstruction>."
// 0 "Turn <FormTurnTypeInstruction>."
// 0 "Turn <FormTurnTypeInstruction>."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn left.",
      "Turn left.",
      "Turn left.",
      "Continue for a half mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES>."
// 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES(1)>."
// 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES(2)>."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn sharp right onto Flatbush Avenue.",
      "Turn sharp right onto Flatbush Avenue.",
      "Turn sharp right onto Flatbush Avenue.",
      "Continue for 1 tenth of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormTurnInstruction
// 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(1)>."
// 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(2)>."
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
// 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
// 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES(1)>."
// 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."
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
      "Turn right onto Sunstone Drive.",
      "Continue for 300 feet.");
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Turn right to stay on Sunstone Drive.",
      "Turn right to stay on Sunstone Drive.",
      "Turn right to stay on Sunstone Drive.",
      "Continue for 100 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 0 "Bear <FormTurnTypeInstruction>."
// 0 "Bear <FormTurnTypeInstruction>."
// 0 "Bear <FormTurnTypeInstruction>."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Bear right.",
      "Bear right.",
      "Bear right.",
      "Continue for 60 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES>."
// 1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES(1)>."
// 1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES(2)>."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Bear left onto Arlen Road.",
      "Bear left onto Arlen Road.",
      "Bear left onto Arlen Road.",
      "Continue for 400 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormBearInstruction
// 2 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
// 2 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(1)>."
// 2 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(2)>."
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
// 3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
// 3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES(1)>."
// 3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."
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
      "",
      "Exit the roundabout onto Catoctin Mountain Highway, U.S. 15.",
      "Continue on U.S. 15 for 11.4 miles.");
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Bear left to stay on US 15 South.",
      "Bear left to stay on U.S. 15 South.",
      "Bear left to stay on U.S. 15 South.",
      "Continue for 2.6 miles.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 0 "Make a <FormTurnTypeInstruction> U-turn."
// 0 "Make a <FormTurnTypeInstruction> U-turn."
// 0 "Make a <FormTurnTypeInstruction> U-turn."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a left U-turn.",
      "Make a left U-turn.",
      "Make a left U-turn.",
      "Continue for 4 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 1 "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES>."
// 1 "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES(1)>."
// 1 "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES(2)>."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a right U-turn onto Bunker Hill Road.",
      "Make a right U-turn onto Bunker Hill Road.",
      "Make a right U-turn onto Bunker Hill Road.",
      "Continue for 4 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 2 "Make a <FormTurnTypeInstruction> U-turn to stay on <STREET_NAMES>."
// 2 "Make a <FormTurnTypeInstruction> U-turn to stay on <STREET_NAMES(1)>."
// 2 "Make a <FormTurnTypeInstruction> U-turn to stay on <STREET_NAMES(2)>."
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
  SetExpectedPreviousManeuverInstructions(
      expected_maneuvers,
      "Turn right onto Bunker Hill Road.",
      "Turn right onto Bunker Hill Road.",
      "Turn right onto Bunker Hill Road.",
      "Continue for 2 tenths of a mile.");
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a left U-turn to stay on Bunker Hill Road.",
      "Make a left U-turn to stay on Bunker Hill Road.",
      "Make a left U-turn to stay on Bunker Hill Road.",
      "Continue for 2 tenths of a mile.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES>."
// 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(1)>."
// 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)>."
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
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a left U-turn at Devonshire Road.",
      "Make a left U-turn at Devonshire Road.",
      "Make a left U-turn at Devonshire Road.",
      "Continue for 200 feet.");

  TryBuild(directions_options, maneuvers, expected_maneuvers);
}

///////////////////////////////////////////////////////////////////////////////
// FormUturnInstruction
// 4 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>."
// 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(1)>."
// 4 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)> onto <STREET_NAMES(2)>."
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
// 5 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
// 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(1)>."
// 5 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)> to stay on <STREET_NAMES(2)>."
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
      "Head northeast on Jonestown Road, U.S. 22 for 200 feet.",
      "");
  SetExpectedManeuverInstructions(
      expected_maneuvers,
      "Make a left U-turn at Devonshire Road to stay on Jonestown Road/US 22.",
      "Make a left U-turn at Devonshire Road.",
      "Make a left U-turn at Devonshire Road to stay on Jonestown Road, U.S. 22.",
      "Continue for 200 feet.");

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
                                            DirectionsOptions_Units units,
                                            bool include_street_names,
                                            std::string expected) {
  std::string instruction = nbt.FormVerbalPostTransitionInstruction(
      maneuver, units, include_street_names);
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

  // Verify kilometers round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 3.54056f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 3.5 kilometers.");

  // Verify kilometers round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 3.86243f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 3.9 kilometers.");

  // Verify kilometers street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 3.86243f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 3.9 kilometers.");

  // Verify 1 kilometer round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 1.04f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 1 kilometer.");

  // Verify 1 kilometer round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.95f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 1 kilometer.");

  // Verify 1 kilometer street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 1.0f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 1 kilometer.");

  // Verify a half kilometer round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.54f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for a half kilometer.");

  // Verify a half kilometer round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.45f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for a half kilometer.");

  // Verify a half kilometer street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.5f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for a half kilometer.");

  // Verify 900 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.94f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 900 meters.");

  // Verify 900 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.85f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 900 meters.");

  // Verify 900 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.9f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 900 meters.");

  // Verify 400 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.44f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 400 meters.");

  // Verify 400 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.35f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 400 meters.");

  // Verify 400 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.4f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 400 meters.");

  // Verify 100 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.14f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 100 meters.");

  // Verify 100 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.095f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 100 meters.");

  // Verify 100 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.1f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 100 meters.");

  // Verify 90 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.094f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 90 meters.");

  // Verify 90 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.085f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 90 meters.");

  // Verify 90 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.09f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 90 meters.");

  // Verify 30 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.034f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 30 meters.");

  // Verify 30 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.025f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 30 meters.");

  // Verify 30 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.03f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 30 meters.");

  // Verify 10 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.012f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 10 meters.");

  // Verify 10 meters round up
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.0096f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for 10 meters.");

  // Verify 10 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.01f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for 10 meters.");

  // Verify less than 10 meters round down
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" },  0.0094f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for less than 10 meters.");

  // Verify less than 10 meters
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.0088f),
      DirectionsOptions_Units_kKilometers, false,
      "Continue for less than 10 meters.");

  // Verify less than 10 meters street name
  TryFormVerbalPostTransitionInstruction(nbt_km,
      CreateVerbalPostManeuver( { "Main Street" }, 0.005f),
      DirectionsOptions_Units_kKilometers, true,
      "Continue on Main Street for less than 10 meters.");

  /////////////////////////////////////////////////////////////////////////////

  directions_options.set_units(DirectionsOptions_Units_kMiles);

  NarrativeBuilderTest nbt_mi(directions_options, dictionary);

  // Verify miles round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 3.604931f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 2.2 miles.");

  // Verify miles round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 3.637117f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 2.3 miles.");

  // Verify miles street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 3.637117f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 2.3 miles.");

  // Verify 1 mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 1.657624f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 mile.");

  // Verify 1 mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 1.561064f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 mile.");

  // Verify 1 mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 1.60934f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 1 mile.");

  // Verify half mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.8368589f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for a half mile.");

  // Verify half mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.7724851f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for a half mile.");

  // Verify half mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.804672f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for a half mile.");

  // Verify 9 tenths of a mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 1.480596f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 9 tenths of a mile.");

  // Verify 9 tenths of a mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 1.416223f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 9 tenths of a mile.");

  // Verify 9 tenths of a mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 1.44841f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 9 tenths of a mile.");

  // Verify 4 tenths of a mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.675924f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 4 tenths of a mile.");

  // Verify 4 tenths of a mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.611551f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 4 tenths of a mile.");

  // Verify 4 tenths of a mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.643738f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 4 tenths of a mile.");

  // Verify 1 tenth of a mile round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.193121f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 tenth of a mile.");

  // Verify 1 tenth of a mile round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.158496f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 1 tenth of a mile.");

  // Verify 1 tenth of a mile street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.160934f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 1 tenth of a mile.");

  // Verify 500 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.155448f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 500 feet.");

  // Verify 500 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.149352f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 500 feet.");

  // Verify 500 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.1524f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 500 feet.");

  // Verify 100 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.036576f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 100 feet.");

  // Verify 100 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.028956f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 100 feet.");

  // Verify 100 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.03048f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 100 feet.");

  // Verify 90 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.0283464f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 90 feet.");

  // Verify 90 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.0268224f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 90 feet.");

  // Verify 90 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.027432f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 90 feet.");

  // Verify 10 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.0036576f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 10 feet.");

  // Verify 10 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.00292608f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for 10 feet.");

  // Verify 10 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.003048f),
      DirectionsOptions_Units_kMiles, true,
      "Continue on Main Street for 10 feet.");

  // Verify less than 10 feet round down
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.00280416f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for less than 10 feet.");

  // Verify less than 10 feet round up
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.00268224f),
      DirectionsOptions_Units_kMiles, false,
      "Continue for less than 10 feet.");

  // Verify less than 10 feet street name
  TryFormVerbalPostTransitionInstruction(nbt_mi,
      CreateVerbalPostManeuver( { "Main Street" }, 0.001524f),
      DirectionsOptions_Units_kMiles, true,
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
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { }, { },
                         { }, { }),
      "Stay straight to take the ramp.");

  // phrase_id = 1
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { }, { }),
      "Stay straight to take the I 95 South ramp.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { }, { "Gettysburg Pike" }),
      "Stay straight to take the I 95 South ramp.");

  // phrase_id = 2
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { }, { "Baltimore" }, { }),
      "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { }, { "Baltimore" }, { "Gettysburg Pike" }),
      "Stay straight to take the ramp toward Baltimore.");

  // phrase_id = 3
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward exists
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { "I 95 South" }, { "Baltimore" }, { "Gettysburg Pike" }),
      "Stay straight to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 4
  TryFormRampStraightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampStraight,
                         Maneuver::RelativeDirection::kKeepStraight, { },
                         { }, { }, { "Gettysburg Pike" }),
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
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { }),
      "Take the ramp on the right.");

  // phrase_id = 1
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South ramp on the right.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the right.");

  // phrase_id = 2
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the ramp on the right toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { "Gettysburg Pike" }),
      "Take the ramp on the right toward Baltimore.");

  // phrase_id = 3
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the right toward Baltimore.");

  // phrase_id = 4
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike ramp on the right.");

  // phrase_id = 5
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, { }, { },
                         { }),
      "Turn right to take the ramp.");

  // phrase_id = 6
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { },
                         { "I 95 South" }, { }, { }),
      "Turn right to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { },
                         { }, { "Baltimore" }, { }),
      "Turn right to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Turn right to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampRight,
                         Maneuver::RelativeDirection::kRight, { }, { }, { },
                         { "Gettysburg Pike" }),
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
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { }),
      "Take the ramp on the left.");

  // phrase_id = 1
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South ramp on the left.");

  // phrase_id = 1; Test that exit name is not used when a branch exists
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the left.");

  // phrase_id = 2
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the ramp on the left toward Baltimore.");

  // phrase_id = 2; Test that exit name is not used when a toward exists
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { "Gettysburg Pike" }),
      "Take the ramp on the left toward Baltimore.");

  // phrase_id = 3
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 3; Test that exit name is not used when a branch or toward
  // exists
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { "Gettysburg Pike" }),
      "Take the I 95 South ramp on the left toward Baltimore.");

  // phrase_id = 4
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike ramp on the left.");

  // phrase_id = 5
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { }, { }, { },
                         { }),
      "Turn left to take the ramp.");

  // phrase_id = 6
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { "I 95 South" }, { }, { }),
      "Turn left to take the I 95 South ramp.");

  // phrase_id = 7
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { }, { "Baltimore" }, { }),
      "Turn left to take the ramp toward Baltimore.");

  // phrase_id = 8
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { },
                         { "I 95 South" }, { "Baltimore" }, { }),
      "Turn left to take the I 95 South ramp toward Baltimore.");

  // phrase_id = 9
  TryFormRampLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kRampLeft,
                         Maneuver::RelativeDirection::kLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
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
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { }),
      "Take the exit on the right.");

  // phrase_id = 1
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { }, { }),
      "Take exit 67A on the right.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { }, { "Gettysburg Pike" }),
      "Take exit 67A on the right.");

  // phrase_id = 2
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South exit on the right.");

  // phrase_id = 3
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" }, {
                             "I 95 South" },
                         { }, { }),
      "Take exit 67A on the right onto I 95 South.");

  // phrase_id = 4
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the exit on the right toward Baltimore.");

  // phrase_id = 5
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" },
                         { }, { "Baltimore" }, { }),
      "Take exit 67A on the right toward Baltimore.");

  // phrase_id = 6
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South exit on the right toward Baltimore.");

  // phrase_id = 7
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { "67A" }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take exit 67A on the right onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right.");

  // phrase_id = 10
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right onto US 15.");

  // phrase_id = 12
  TryFormExitRightInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitRight,
                         Maneuver::RelativeDirection::kKeepRight, { }, { }, {
                             "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the right toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitRightInstruction(nbt,
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
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { }),
      "Take the exit on the left.");

  // phrase_id = 1
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { }, { }),
      "Take exit 67A on the left.");

  // phrase_id = 1; Test that name is ignored when number is present
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { }, { "Gettysburg Pike" }),
      "Take exit 67A on the left.");

  // phrase_id = 2
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { }, { }),
      "Take the I 95 South exit on the left.");

  // phrase_id = 3
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, {
                             "I 95 South" },
                         { }, { }),
      "Take exit 67A on the left onto I 95 South.");

  // phrase_id = 4
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Baltimore" },
                         { }),
      "Take the exit on the left toward Baltimore.");

  // phrase_id = 5
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, { },
                         { "Baltimore" }, { }),
      "Take exit 67A on the left toward Baltimore.");

  // phrase_id = 6
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take the I 95 South exit on the left toward Baltimore.");

  // phrase_id = 7
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { "67A" }, {
                             "I 95 South" },
                         { "Baltimore" }, { }),
      "Take exit 67A on the left onto I 95 South toward Baltimore.");

  // phrase_id = 8
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, { },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left.");

  // phrase_id = 10
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { },
                         { "US 15" }, { }, { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15.");

  // phrase_id = 12
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { }, { }, {
                             "Harrisburg", "Gettysburg" },
                         { "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left toward Harrisburg/Gettysburg.");

  // phrase_id = 14
  TryFormExitLeftInstruction(nbt,
      CreateSignManeuver(TripDirections_Maneuver_Type_kExitLeft,
                         Maneuver::RelativeDirection::kKeepLeft, { },
                         { "US 15" }, { "Harrisburg", "Gettysburg" }, {
                             "Gettysburg Pike" }),
      "Take the Gettysburg Pike exit on the left onto US 15 toward Harrisburg/Gettysburg.");

}

}

int main() {
  test::suite suite("narrativebuilder");

  valhalla::odin::get_locales("./conf/locales");

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

  return suite.tear_down();
}

