#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "baldr/verbal_text_formatter.h"
#include "midgard/constants.h"

#include "odin/enhancedtrippath.h"
#include "odin/maneuver.h"
#include "odin/markup_formatter.h"
#include "odin/narrative_dictionary.h"
#include "odin/narrativebuilder.h"
#include "odin/util.h"
#include "worker.h"

namespace {
// Text instruction initial capacity
constexpr auto kInstructionInitialCapacity = 128;

// Text length initial capacity
constexpr auto kLengthStringInitialCapacity = 32;

// Basic time threshold in seconds for creating a verbal multi-cue
constexpr auto kVerbalMultiCueTimeThreshold = 13;
constexpr auto kVerbalMultiCueTimeStartManeuverThreshold = kVerbalMultiCueTimeThreshold * 3;

constexpr float kVerbalPostMinimumRampLength = 2.0f; // Kilometers
constexpr float kVerbalAlertMergePriorManeuverMinimumLength = kVerbalPostMinimumRampLength;

// Lower and upper bounds for roundabout_exit_count
constexpr uint32_t kRoundaboutExitCountLowerBound = 1;
constexpr uint32_t kRoundaboutExitCountUpperBound = 10;

} // namespace

namespace valhalla {
namespace odin {

NarrativeBuilder::NarrativeBuilder(const Options& options,
                                   const EnhancedTripLeg* trip_path,
                                   const NarrativeDictionary& dictionary,
                                   const MarkupFormatter& markup_formatter)
    : options_(options), trip_path_(trip_path), dictionary_(dictionary),
      markup_formatter_(markup_formatter), articulated_preposition_enabled_(false) {
}

void NarrativeBuilder::Build(std::list<Maneuver>& maneuvers) {
  Maneuver* prev_maneuver = nullptr;
  for (auto& maneuver : maneuvers) {
    switch (maneuver.type()) {
      case DirectionsLeg_Maneuver_Type_kStartRight:
      case DirectionsLeg_Maneuver_Type_kStart:
      case DirectionsLeg_Maneuver_Type_kStartLeft:
      case DirectionsLeg_Maneuver_Type_kFerryExit:
      case DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination: {
        // Set instruction
        maneuver.set_instruction(FormStartInstruction(maneuver));

        // Set verbal succinct transition instruction
        maneuver.set_verbal_succinct_transition_instruction(
            FormVerbalSuccinctStartTransitionInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalStartInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver, maneuver.HasBeginStreetNames()));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kDestinationRight:
      case DirectionsLeg_Maneuver_Type_kDestination:
      case DirectionsLeg_Maneuver_Type_kDestinationLeft: {
        // Set instruction
        maneuver.set_instruction(FormDestinationInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            FormVerbalAlertDestinationInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalDestinationInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kBecomes: {
        if (prev_maneuver) {
          // Set instruction
          maneuver.set_instruction(FormBecomesInstruction(maneuver, prev_maneuver));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(
              FormVerbalBecomesInstruction(maneuver, prev_maneuver));
        }

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver,
                                                maneuver
                                                    .HasBeginStreetNames())); // Set verbal succinct
                                                                              // transition
                                                                              // instruction
        break;
      }
      case DirectionsLeg_Maneuver_Type_kSlightRight:
      case DirectionsLeg_Maneuver_Type_kSlightLeft:
      case DirectionsLeg_Maneuver_Type_kRight:
      case DirectionsLeg_Maneuver_Type_kSharpRight:
      case DirectionsLeg_Maneuver_Type_kSharpLeft:
      case DirectionsLeg_Maneuver_Type_kLeft: {
        // Set instruction
        maneuver.set_instruction(FormTurnInstruction(maneuver));

        // Set verbal succinct transition instruction
        maneuver.set_verbal_succinct_transition_instruction(
            FormVerbalSuccinctTurnTransitionInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(FormVerbalAlertTurnInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalTurnInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver, maneuver.HasBeginStreetNames()));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kUturnRight:
      case DirectionsLeg_Maneuver_Type_kUturnLeft: {
        // Set instruction
        maneuver.set_instruction(FormUturnInstruction(maneuver));

        // Set verbal succinct transition instruction
        maneuver.set_verbal_succinct_transition_instruction(
            FormVerbalSuccinctUturnTransitionInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(FormVerbalAlertUturnInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalUturnInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kRampStraight: {
        // Set instruction
        maneuver.set_instruction(FormRampStraightInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            FormVerbalAlertRampStraightInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalRampStraightInstruction(maneuver));

        // Only set verbal post if > min ramp length
        // or contains obvious maneuver
        // or has collapsed merge maneuver
        if ((maneuver.length() > kVerbalPostMinimumRampLength) ||
            maneuver.contains_obvious_maneuver() || maneuver.has_collapsed_merge_maneuver()) {
          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              FormVerbalPostTransitionInstruction(maneuver));
        }
        break;
      }
      case DirectionsLeg_Maneuver_Type_kRampRight:
      case DirectionsLeg_Maneuver_Type_kRampLeft: {
        // Set instruction
        maneuver.set_instruction(FormRampInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(FormVerbalAlertRampInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalRampInstruction(maneuver));

        // Only set verbal post if > min ramp length
        // or contains obvious maneuver
        // or has collapsed merge maneuver
        if ((maneuver.length() > kVerbalPostMinimumRampLength) ||
            maneuver.contains_obvious_maneuver() || maneuver.has_collapsed_merge_maneuver()) {
          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              FormVerbalPostTransitionInstruction(maneuver));
        }
        break;
      }
      case DirectionsLeg_Maneuver_Type_kExitRight:
      case DirectionsLeg_Maneuver_Type_kExitLeft: {
        // Set instruction
        maneuver.set_instruction(FormExitInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(FormVerbalAlertExitInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalExitInstruction(maneuver));

        // Only set verbal post if > min ramp length
        // or contains obvious maneuver
        // or has collapsed merge maneuver
        if ((maneuver.length() > kVerbalPostMinimumRampLength) ||
            maneuver.contains_obvious_maneuver() || maneuver.has_collapsed_merge_maneuver()) {
          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              FormVerbalPostTransitionInstruction(maneuver));
        }
        break;
      }
      case DirectionsLeg_Maneuver_Type_kStayStraight:
      case DirectionsLeg_Maneuver_Type_kStayRight:
      case DirectionsLeg_Maneuver_Type_kStayLeft: {
        if (maneuver.to_stay_on()) {
          // Set stay on instruction
          maneuver.set_instruction(FormKeepToStayOnInstruction(maneuver));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              FormVerbalAlertKeepToStayOnInstruction(maneuver));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(FormVerbalKeepToStayOnInstruction(maneuver));

          // For a ramp - only set verbal post if > min ramp length
          if (maneuver.ramp() && !maneuver.has_collapsed_merge_maneuver()) {
            if (maneuver.length() > kVerbalPostMinimumRampLength) {
              // Set verbal post transition instruction
              maneuver.set_verbal_post_transition_instruction(
                  FormVerbalPostTransitionInstruction(maneuver));
            }
          } else {
            // Set verbal post transition instruction
            maneuver.set_verbal_post_transition_instruction(
                FormVerbalPostTransitionInstruction(maneuver));
          }
        } else {
          // Set instruction
          maneuver.set_instruction(FormKeepInstruction(maneuver));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(FormVerbalAlertKeepInstruction(maneuver));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(FormVerbalKeepInstruction(maneuver));

          // For a ramp - only set verbal post if > min ramp length
          if (maneuver.ramp() && !maneuver.has_collapsed_merge_maneuver()) {
            if (maneuver.length() > kVerbalPostMinimumRampLength) {
              // Set verbal post transition instruction
              maneuver.set_verbal_post_transition_instruction(
                  FormVerbalPostTransitionInstruction(maneuver));
            }
          } else {
            // Set verbal post transition instruction
            maneuver.set_verbal_post_transition_instruction(
                FormVerbalPostTransitionInstruction(maneuver));
          }
        }
        break;
      }
      case DirectionsLeg_Maneuver_Type_kMerge:
      case DirectionsLeg_Maneuver_Type_kMergeRight:
      case DirectionsLeg_Maneuver_Type_kMergeLeft: {
        // Set instruction
        maneuver.set_instruction(FormMergeInstruction(maneuver));

        // Set verbal succinct transition instruction
        maneuver.set_verbal_succinct_transition_instruction(
            FormVerbalSuccinctMergeTransitionInstruction(maneuver));

        // Set verbal transition alert instruction if previous maneuver
        // is greater than 2 km
        if (prev_maneuver && (prev_maneuver->length(Options::kilometers) >
                              kVerbalAlertMergePriorManeuverMinimumLength)) {
          maneuver.set_verbal_transition_alert_instruction(FormVerbalAlertMergeInstruction(maneuver));
        }

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalMergeInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kRoundaboutEnter: {
        // Set instruction
        maneuver.set_instruction(FormEnterRoundaboutInstruction(maneuver));

        // Set verbal succinct transition instruction
        maneuver.set_verbal_succinct_transition_instruction(
            FormVerbalSuccinctEnterRoundaboutTransitionInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            FormVerbalAlertEnterRoundaboutInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalEnterRoundaboutInstruction(maneuver));

        // If the maneuver has a combined enter exit roundabout instruction
        // then set verbal post transition instruction
        if (maneuver.has_combined_enter_exit_roundabout()) {
          maneuver.set_verbal_post_transition_instruction(
              FormVerbalPostTransitionInstruction(maneuver,
                                                  maneuver.HasRoundaboutExitBeginStreetNames()));
        }
        break;
      }
      case DirectionsLeg_Maneuver_Type_kRoundaboutExit: {
        // Set instruction
        maneuver.set_instruction(FormExitRoundaboutInstruction(maneuver));

        // Set verbal succinct transition instruction
        maneuver.set_verbal_succinct_transition_instruction(
            FormVerbalSuccinctExitRoundaboutTransitionInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalExitRoundaboutInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver, maneuver.HasBeginStreetNames()));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kFerryEnter: {
        // Set instruction
        maneuver.set_instruction(FormEnterFerryInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            FormVerbalAlertEnterFerryInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalEnterFerryInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kTransitConnectionStart: {
        // Set instruction
        maneuver.set_instruction(FormTransitConnectionStartInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalTransitConnectionStartInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kTransitConnectionTransfer: {
        // Set instruction
        maneuver.set_instruction(FormTransitConnectionTransferInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalTransitConnectionTransferInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kTransitConnectionDestination: {
        // Set instruction
        maneuver.set_instruction(FormTransitConnectionDestinationInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalTransitConnectionDestinationInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kTransit: {
        // Set depart instruction
        maneuver.set_depart_instruction(FormDepartInstruction(maneuver));

        // Set verbal depart instruction
        maneuver.set_verbal_depart_instruction(FormVerbalDepartInstruction(maneuver));

        // Set instruction
        maneuver.set_instruction(FormTransitInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalTransitInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionTransitInstruction(maneuver));

        // Set arrive instruction
        maneuver.set_arrive_instruction(FormArriveInstruction(maneuver));

        // Set verbal arrive instruction
        maneuver.set_verbal_arrive_instruction(FormVerbalArriveInstruction(maneuver));

        break;
      }
      case DirectionsLeg_Maneuver_Type_kTransitRemainOn: {
        // Set depart instruction
        maneuver.set_depart_instruction(FormDepartInstruction(maneuver));

        // Set verbal depart instruction
        maneuver.set_verbal_depart_instruction(FormVerbalDepartInstruction(maneuver));

        // Set instruction
        maneuver.set_instruction(FormTransitRemainOnInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalTransitRemainOnInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionTransitInstruction(maneuver));

        // Set arrive instruction
        maneuver.set_arrive_instruction(FormArriveInstruction(maneuver));

        // Set verbal arrive instruction
        maneuver.set_verbal_arrive_instruction(FormVerbalArriveInstruction(maneuver));

        break;
      }
      case DirectionsLeg_Maneuver_Type_kTransitTransfer: {
        // Set depart instruction
        maneuver.set_depart_instruction(FormDepartInstruction(maneuver));

        // Set verbal depart instruction
        maneuver.set_verbal_depart_instruction(FormVerbalDepartInstruction(maneuver));

        // Set instruction
        maneuver.set_instruction(FormTransitTransferInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalTransitTransferInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionTransitInstruction(maneuver));

        // Set arrive instruction
        maneuver.set_arrive_instruction(FormArriveInstruction(maneuver));

        // Set verbal arrive instruction
        maneuver.set_verbal_arrive_instruction(FormVerbalArriveInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kElevatorEnter: {
        // Set instruction
        maneuver.set_instruction(FormElevatorInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kStepsEnter: {
        // Set instruction
        maneuver.set_instruction(FormStepsInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kEscalatorEnter: {
        // Set instruction
        maneuver.set_instruction(FormEscalatorInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kBuildingEnter: {
        // Set instruction
        maneuver.set_instruction(FormEnterBuildingInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kBuildingExit: {
        // Set instruction
        maneuver.set_instruction(FormExitBuildingInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kContinue:
      default: {
        if (maneuver.has_node_type()) {
          std::string instr = FormPassInstruction(maneuver);
          // Set instruction
          maneuver.set_instruction(instr);
          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(instr);
        } else {
          // Set instruction
          maneuver.set_instruction(FormContinueInstruction(maneuver));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              FormVerbalAlertContinueInstruction(maneuver));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(FormVerbalContinueInstruction(maneuver));

          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              FormVerbalPostTransitionInstruction(maneuver));
        }
        break;
      }
    }
    maneuver.set_instruction(FormBssManeuverType(maneuver.bss_maneuver_type()) +
                             maneuver.instruction());

    // Update previous maneuver
    prev_maneuver = &maneuver;
  }

  // Iterate over maneuvers to form verbal multi-cue instructions
  FormVerbalMultiCue(maneuvers);
}

std::string NarrativeBuilder::FormVerbalAlertApproachInstruction(float distance,
                                                                 const std::string& verbal_cue) {
  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.approach_verbal_alert_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kLengthTag,
                     FormLength(distance, dictionary_.approach_verbal_alert_subset.metric_lengths,
                                dictionary_.approach_verbal_alert_subset.us_customary_lengths));
  boost::replace_all(instruction, kCurrentVerbalCueTag, verbal_cue);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormStartInstruction(Maneuver& maneuver) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",
  // "4": "Drive <CARDINAL_DIRECTION>.",
  // "5": "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "6": "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",
  // "8": "Walk <CARDINAL_DIRECTION>.",
  // "9": "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "10": "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",
  // "16": "Bike <CARDINAL_DIRECTION>.",
  // "17": "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "18": "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set cardinal_direction value
  std::string cardinal_direction =
      dictionary_.start_subset.cardinal_directions.at(maneuver.begin_cardinal_direction());

  // Set street_names value
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.start_subset.empty_street_name_labels, true);

  // Set begin_street_names value
  std::string begin_street_names = FormStreetNames(maneuver, maneuver.begin_street_names());

  // Update street names for maneuvers that contain obvious maneuvers
  UpdateObviousManeuverStreetNames(maneuver, begin_street_names, street_names);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id += 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id += 1;
  }
  if (maneuver.travel_mode() == TravelMode::kDrive) {
    phrase_id += 4;
  } else if (maneuver.travel_mode() == TravelMode::kPedestrian) {
    phrase_id += 8;
  } else if (maneuver.travel_mode() == TravelMode::kBicycle) {
    phrase_id += 16;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.start_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kCardinalDirectionTag, cardinal_direction);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalStartInstruction(Maneuver& maneuver,
                                                         uint32_t element_max_count,
                                                         const std::string& delim) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> for <LENGTH>.",
  // "2": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "3": "Head <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>.",
  // "4": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "5": "Drive <CARDINAL_DIRECTION>.",
  // "6": "Drive <CARDINAL_DIRECTION> for <LENGTH>.",
  // "7": "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "8": "Drive <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>.",
  // "9": "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "10": "Walk <CARDINAL_DIRECTION>.",
  // "11": "Walk <CARDINAL_DIRECTION> for <LENGTH>.",
  // "12": "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "13": "Walk <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>.",
  // "14": "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "15": "Bike <CARDINAL_DIRECTION>.",
  // "16": "Bike <CARDINAL_DIRECTION> for <LENGTH>.",
  // "17": "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "18": "Bike <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>.",
  // "19": "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set cardinal_direction value
  std::string cardinal_direction =
      dictionary_.start_verbal_subset.cardinal_directions.at(maneuver.begin_cardinal_direction());

  // Set street_names value
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.start_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Set begin_street_names value
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(),
                      &dictionary_.start_verbal_subset.empty_street_name_labels, false,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Update street names for maneuvers that contain obvious maneuvers
  UpdateObviousManeuverStreetNames(maneuver, begin_street_names, street_names);

  // Determine which phrase to use
  uint8_t phrase_id = 0;

  // Set base phrase id per mode
  if (maneuver.travel_mode() == TravelMode::kDrive) {
    phrase_id += 5;
  } else if (maneuver.travel_mode() == TravelMode::kPedestrian) {
    phrase_id += 10;
  } else if (maneuver.travel_mode() == TravelMode::kBicycle) {
    phrase_id += 15;
  }

  if (!street_names.empty()) {
    // Increment phrase id for street names
    phrase_id += 2;
  }

  if (!begin_street_names.empty()) {
    // Increment phrase id for begin street names
    phrase_id += 2;
  } else if (maneuver.include_verbal_pre_transition_length()) {
    // For non begin street names, increment phrase id for length
    phrase_id += 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.start_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kCardinalDirectionTag, cardinal_direction);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);
  boost::replace_all(instruction, kLengthTag,
                     FormLength(maneuver, dictionary_.start_verbal_subset.metric_lengths,
                                dictionary_.start_verbal_subset.us_customary_lengths));

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormDestinationInstruction(Maneuver& maneuver) {
  // "0": "You have arrived at your destination."
  // "1": "You have arrived at <DESTINATION>."
  // "2": "Your destination is on the <RELATIVE_DIRECTION>."
  // "3": "<DESTINATION> is on the <RELATIVE_DIRECTION>."

  uint8_t phrase_id = 0;
  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine if location (name or street) exists
  std::string destination;
  const auto& dest = trip_path_->GetDestination();
  // Check for destination name
  if (!dest.name().empty()) {
    phrase_id += 1;
    destination = dest.name();
  }
  // Check for destination street
  else if (!dest.street().empty()) {
    phrase_id += 1;
    destination = dest.street();
  }

  // Check for side of street relative direction
  std::string relative_direction;
  if (maneuver.type() == DirectionsLeg_Maneuver_Type_kDestinationLeft) {
    phrase_id += 2;
    relative_direction = dictionary_.destination_subset.relative_directions.at(0);
  } else if (maneuver.type() == DirectionsLeg_Maneuver_Type_kDestinationRight) {
    phrase_id += 2;
    relative_direction = dictionary_.destination_subset.relative_directions.at(1);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.destination_subset.phrases.at(std::to_string(phrase_id));

  if (phrase_id > 0) {
    // Replace phrase tags with values
    boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
    boost::replace_all(instruction, kDestinationTag, destination);
  }

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertDestinationInstruction(Maneuver& maneuver) {
  // "0": "You will arrive at your destination.",
  // "1": "You will arrive at <DESTINATION>.",
  // "2": "Your destination will be on the <RELATIVE_DIRECTION>.",
  // "3": "<DESTINATION> will be on the <RELATIVE_DIRECTION>."

  uint8_t phrase_id = 0;
  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine if destination (name or street) exists
  std::string destination;
  auto& dest = trip_path_->GetDestination();
  // Check for destination name
  if (!dest.name().empty()) {
    phrase_id += 1;
    destination = dest.name();
  }
  // Check for destination street
  else if (!dest.street().empty()) {
    phrase_id += 1;
    auto* verbal_formatter = maneuver.verbal_formatter();
    if (verbal_formatter) {
      destination = verbal_formatter->Format(dest.street());
    } else {
      destination = dest.street();
    }
  }

  // Check for side of street relative direction
  std::string relative_direction;
  if (maneuver.type() == DirectionsLeg_Maneuver_Type_kDestinationLeft) {
    phrase_id += 2;
    relative_direction = dictionary_.destination_subset.relative_directions.at(0);
  } else if (maneuver.type() == DirectionsLeg_Maneuver_Type_kDestinationRight) {
    phrase_id += 2;
    relative_direction = dictionary_.destination_subset.relative_directions.at(1);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.destination_verbal_alert_subset.phrases.at(std::to_string(phrase_id));

  if (phrase_id > 0) {
    // Replace phrase tags with values
    boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
    boost::replace_all(instruction, kDestinationTag, destination);
  }

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalDestinationInstruction(Maneuver& maneuver) {
  // "0": "You have arrived at your destination.",
  // "1": "You have arrived at <DESTINATION>.",
  // "2": "Your destination is on the <RELATIVE_DIRECTION>.",
  // "3": "<DESTINATION> is on the <RELATIVE_DIRECTION>."

  uint8_t phrase_id = 0;
  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine if destination (name or street) exists
  std::string destination;
  auto& dest = trip_path_->GetDestination();
  // Check for destination name
  if (!dest.name().empty()) {
    phrase_id += 1;
    destination = dest.name();
  }
  // Check for destination street
  else if (!dest.street().empty()) {
    phrase_id += 1;
    auto* verbal_formatter = maneuver.verbal_formatter();
    if (verbal_formatter) {
      destination = verbal_formatter->Format(dest.street());
    } else {
      destination = dest.street();
    }
  }

  // Check for side of street relative direction
  std::string relative_direction;
  if (maneuver.type() == DirectionsLeg_Maneuver_Type_kDestinationLeft) {
    phrase_id += 2;
    relative_direction = dictionary_.destination_subset.relative_directions.at(0);
  } else if (maneuver.type() == DirectionsLeg_Maneuver_Type_kDestinationRight) {
    phrase_id += 2;
    relative_direction = dictionary_.destination_subset.relative_directions.at(1);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.destination_verbal_subset.phrases.at(std::to_string(phrase_id));

  if (phrase_id > 0) {
    // Replace phrase tags with values
    boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
    boost::replace_all(instruction, kDestinationTag, destination);
  }

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormBecomesInstruction(Maneuver& maneuver, Maneuver* prev_maneuver) {
  // "0": "<PREVIOUS_STREET_NAMES> becomes <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names and the previous maneuver street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names());
  std::string prev_street_names = FormStreetNames(*prev_maneuver, prev_maneuver->street_names());

  // Determine which phrase to use
  uint8_t phrase_id = 0;

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.becomes_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kPreviousStreetNamesTag, prev_street_names);
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalBecomesInstruction(Maneuver& maneuver,
                                                           Maneuver* prev_maneuver,
                                                           uint32_t element_max_count,
                                                           const std::string& delim) {
  // "0": "<PREVIOUS_STREET_NAMES> becomes <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names and the previous maneuver street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(), nullptr, false, element_max_count, delim,
                      prev_maneuver->verbal_formatter());
  std::string prev_street_names =
      FormStreetNames(*prev_maneuver, prev_maneuver->street_names(), nullptr, false,
                      element_max_count, delim, prev_maneuver->verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.becomes_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kPreviousStreetNamesTag, prev_street_names);
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormContinueInstruction(Maneuver& maneuver,
                                                      bool limit_by_consecutive_count,
                                                      uint32_t element_max_count) {
  // "0": "Continue.",
  // "1": "Continue on <STREET_NAMES>."
  // "2": "Continue at <JUNCTION_NAME>."
  // "3": "Continue toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.continue_subset.empty_street_name_labels, true);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 3;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 2;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count);
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.continue_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertContinueInstruction(Maneuver& maneuver,
                                                                 bool limit_by_consecutive_count,
                                                                 uint32_t element_max_count,
                                                                 const std::string& delim) {
  // "0": "Continue.",
  // "1": "Continue on <STREET_NAMES>."
  // "2": "Continue at <JUNCTION_NAME>."
  // "3": "Continue toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.continue_verbal_alert_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 3;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 2;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.continue_verbal_alert_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalContinueInstruction(Maneuver& maneuver,
                                                            bool limit_by_consecutive_count,
                                                            uint32_t element_max_count,
                                                            const std::string& delim) {
  // "0": "Continue.",
  // "1": "Continue for <LENGTH>.",
  // "2": "Continue on <STREET_NAMES>.",
  // "3": "Continue on <STREET_NAMES> for <LENGTH>."
  // "4": "Continue at <JUNCTION_NAME>."
  // "5": "Continue at <JUNCTION_NAME> for <LENGTH>."
  // "6": "Continue toward <TOWARD_SIGN>."
  // "7": "Continue toward <TOWARD_SIGN> for <LENGTH>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.continue_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine base phrase
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 6;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 4;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
  } else if (!street_names.empty()) {
    phrase_id = 2;
  }

  if (maneuver.include_verbal_pre_transition_length()) {
    // Increment phrase id for length
    phrase_id += 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.continue_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kLengthTag,
                     FormLength(maneuver, dictionary_.continue_verbal_subset.metric_lengths,
                                dictionary_.continue_verbal_subset.us_customary_lengths));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTurnInstruction(Maneuver& maneuver,
                                                  bool limit_by_consecutive_count,
                                                  uint32_t element_max_count) {
  // "0": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION>.",
  // "1": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  // "2": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on
  // <STREET_NAMES>.",
  // "3": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  // "4": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> at <JUNCTION_NAME>."
  // "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  const TurnSubset* subset = nullptr;
  switch (maneuver.type()) {
    case DirectionsLeg_Maneuver_Type_kSlightRight:
    case DirectionsLeg_Maneuver_Type_kSlightLeft:
      subset = &dictionary_.bear_subset;
      break;
    case DirectionsLeg_Maneuver_Type_kRight:
    case DirectionsLeg_Maneuver_Type_kLeft:
      subset = &dictionary_.turn_subset;
      break;
    case DirectionsLeg_Maneuver_Type_kSharpRight:
    case DirectionsLeg_Maneuver_Type_kSharpLeft:
      subset = &dictionary_.sharp_subset;
      break;
    default:
      throw valhalla_exception_t{230};
  }

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(), &subset->empty_street_name_labels, true);

  // Assign the begin street names
  std::string begin_street_names = FormStreetNames(maneuver, maneuver.begin_street_names());

  // Update street names for maneuvers that contain obvious maneuvers
  UpdateObviousManeuverStreetNames(maneuver, begin_street_names, street_names);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 5;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 4;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count);
  } else if (maneuver.to_stay_on()) {
    phrase_id = 3;
  } else if (!begin_street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = subset->phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(), subset->relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertTurnInstruction(Maneuver& maneuver,
                                                             bool limit_by_consecutive_count,
                                                             uint32_t element_max_count,
                                                             const std::string& delim) {
  // "0": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION>.",
  // "1": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  // "2": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.",
  // "3": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  // "4": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> at <JUNCTION_NAME>."
  // "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  return FormVerbalTurnInstruction(maneuver, limit_by_consecutive_count, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalTurnInstruction(Maneuver& maneuver,
                                                        bool limit_by_consecutive_count,
                                                        uint32_t element_max_count,
                                                        const std::string& delim) {
  // "0": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION>.",
  // "1": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  // "2": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.",
  // "3": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  // "4": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> at <JUNCTION_NAME>."
  // "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  const TurnSubset* subset = nullptr;
  switch (maneuver.type()) {
    case DirectionsLeg_Maneuver_Type_kSlightRight:
    case DirectionsLeg_Maneuver_Type_kSlightLeft:
      subset = &dictionary_.bear_verbal_subset;
      break;
    case DirectionsLeg_Maneuver_Type_kRight:
    case DirectionsLeg_Maneuver_Type_kLeft:
      subset = &dictionary_.turn_verbal_subset;
      break;
    case DirectionsLeg_Maneuver_Type_kSharpRight:
    case DirectionsLeg_Maneuver_Type_kSharpLeft:
      subset = &dictionary_.sharp_verbal_subset;
      break;
    default:
      throw valhalla_exception_t{230};
  }

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(), &subset->empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Assign the begin street names
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(), &subset->empty_street_name_labels,
                      false, element_max_count, delim, maneuver.verbal_formatter());

  // Update street names for maneuvers that contain obvious maneuvers
  UpdateObviousManeuverStreetNames(maneuver, begin_street_names, street_names);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 5;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 4;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
  } else {
    if (!street_names.empty()) {
      phrase_id = 1;
    }
    if (!begin_street_names.empty()) {
      phrase_id = 2;
    }
    if (maneuver.to_stay_on()) {
      phrase_id = 3;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = subset->phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(), subset->relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormUturnInstruction(Maneuver& maneuver,
                                                   bool limit_by_consecutive_count,
                                                   uint32_t element_max_count) {
  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.",
  // "4": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.",
  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
  // "6": "Make a <RELATIVE_DIRECTION> U-turn at <JUNCTION_NAME>."
  // "7": "Make a <RELATIVE_DIRECTION> U-turn toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.uturn_subset.empty_street_name_labels, true);

  // Assign the cross street names
  std::string cross_street_names = FormStreetNames(maneuver, maneuver.cross_street_names());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 7;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 6;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count);
  } else {
    if (!street_names.empty()) {
      phrase_id += 1;
      if (maneuver.to_stay_on()) {
        phrase_id += 1;
      }
    }
    if (!cross_street_names.empty()) {
      phrase_id += 3;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.uturn_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(),
                                              dictionary_.uturn_subset.relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kCrossStreetNamesTag, cross_street_names);
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertUturnInstruction(Maneuver& maneuver,
                                                              bool limit_by_consecutive_count,
                                                              uint32_t element_max_count,
                                                              const std::string& delim) {
  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>."
  // "6": "Make a <RELATIVE_DIRECTION> U-turn at <JUNCTION_NAME>."
  // "7": "Make a <RELATIVE_DIRECTION> U-turn toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.uturn_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Assign the cross street names
  std::string cross_street_names =
      FormStreetNames(maneuver, maneuver.cross_street_names(),
                      &dictionary_.uturn_verbal_subset.empty_street_name_labels, false,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 7;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 6;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
  } else {
    if (!street_names.empty()) {
      phrase_id = 1;
      if (maneuver.to_stay_on()) {
        phrase_id = 2;
      }
    }
    if (!cross_street_names.empty()) {
      phrase_id = 3;
    }
  }

  return FormVerbalUturnInstruction(phrase_id,
                                    FormRelativeTwoDirection(maneuver.type(),
                                                             dictionary_.uturn_verbal_subset
                                                                 .relative_directions),
                                    street_names, cross_street_names, junction_name, guide_sign);
}

std::string NarrativeBuilder::FormVerbalUturnInstruction(Maneuver& maneuver,
                                                         bool limit_by_consecutive_count,
                                                         uint32_t element_max_count,
                                                         const std::string& delim) {
  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.",
  // "4": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.",
  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
  // "6": "Make a <RELATIVE_DIRECTION> U-turn at <JUNCTION_NAME>."
  // "7": "Make a <RELATIVE_DIRECTION> U-turn toward <TOWARD_SIGN>."

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.uturn_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Assign the cross street names
  std::string cross_street_names =
      FormStreetNames(maneuver, maneuver.cross_street_names(),
                      &dictionary_.uturn_verbal_subset.empty_street_name_labels, false,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 7;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 6;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
  } else {
    if (!street_names.empty()) {
      phrase_id += 1;
      if (maneuver.to_stay_on()) {
        phrase_id += 1;
      }
    }
    if (!cross_street_names.empty()) {
      phrase_id += 3;
    }
  }

  return FormVerbalUturnInstruction(phrase_id,
                                    FormRelativeTwoDirection(maneuver.type(),
                                                             dictionary_.uturn_verbal_subset
                                                                 .relative_directions),
                                    street_names, cross_street_names, junction_name, guide_sign);
}

std::string NarrativeBuilder::FormVerbalUturnInstruction(uint8_t phrase_id,
                                                         const std::string& relative_dir,
                                                         const std::string& street_names,
                                                         const std::string& cross_street_names,
                                                         const std::string& junction_name,
                                                         const std::string& guide_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.uturn_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kCrossStreetNamesTag, cross_street_names);
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormRampStraightInstruction(Maneuver& maneuver,
                                                          bool limit_by_consecutive_count,
                                                          uint32_t element_max_count) {
  // "0": "Stay straight to take the ramp.",
  // "1": "Stay straight to take the <BRANCH_SIGN> ramp.",
  // "2": "Stay straight to take the ramp toward <TOWARD_SIGN>.",
  // "3": "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "4": "Stay straight to take the <NAME_SIGN> ramp."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign() && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.ramp_straight_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kBranchSignTag, exit_branch_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);
  boost::replace_all(instruction, kNameSignTag, exit_name_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertRampStraightInstruction(Maneuver& maneuver,
                                                                     bool limit_by_consecutive_count,
                                                                     uint32_t element_max_count,
                                                                     const std::string& delim) {
  // "0": "Stay straight to take the ramp.",
  // "1": "Stay straight to take the <BRANCH_SIGN> ramp.",
  // "2": "Stay straight to take the ramp toward <TOWARD_SIGN>.",
  // "4": "Stay straight to take the <NAME_SIGN> ramp."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitBranchSign()) {
    phrase_id = 1;
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id = 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasExitNameSign()) {
    phrase_id = 4;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count, delim,
                                           maneuver.verbal_formatter(), &markup_formatter_);
  }

  return FormVerbalRampStraightInstruction(phrase_id, exit_branch_sign, exit_toward_sign,
                                           exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalRampStraightInstruction(Maneuver& maneuver,
                                                                bool limit_by_consecutive_count,
                                                                uint32_t element_max_count,
                                                                const std::string& delim) {
  // "0": "Stay straight to take the ramp.",
  // "1": "Stay straight to take the <BRANCH_SIGN> ramp.",
  // "2": "Stay straight to take the ramp toward <TOWARD_SIGN>.",
  // "3": "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "4": "Stay straight to take the <NAME_SIGN> ramp."

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign() && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count, delim,
                                           maneuver.verbal_formatter(), &markup_formatter_);
  }

  return FormVerbalRampStraightInstruction(phrase_id, exit_branch_sign, exit_toward_sign,
                                           exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalRampStraightInstruction(uint8_t phrase_id,
                                                                const std::string& exit_branch_sign,
                                                                const std::string& exit_toward_sign,
                                                                const std::string& exit_name_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.ramp_straight_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kBranchSignTag, exit_branch_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);
  boost::replace_all(instruction, kNameSignTag, exit_name_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormRampInstruction(Maneuver& maneuver,
                                                  bool limit_by_consecutive_count,
                                                  uint32_t element_max_count) {
  // "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
  // "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  // "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  // "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
  // "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
  // "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
  // "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
  // "10": "Take the ramp.",
  // "11": "Take the <BRANCH_SIGN> ramp.",
  // "12": "Take the ramp toward <TOWARD_SIGN>.",
  // "13": "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "14": "Take the <NAME_SIGN> ramp."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;

  // Determine if turn, else it's a "Take" instruction
  if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight) ||
      (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft)) {
    phrase_id = 5;
    // Determine if driving side matches relative direction
  } else if (((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepRight) &&
              maneuver.drive_on_right()) ||
             ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepLeft) &&
              !maneuver.drive_on_right())) {
    phrase_id = 10;
  }
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign() && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.ramp_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(),
                                              dictionary_.ramp_subset.relative_directions));
  boost::replace_all(instruction, kBranchSignTag, exit_branch_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);
  boost::replace_all(instruction, kNameSignTag, exit_name_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertRampInstruction(Maneuver& maneuver,
                                                             bool limit_by_consecutive_count,
                                                             uint32_t element_max_count,
                                                             const std::string& delim) {
  // "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
  // "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  // "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  // "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
  // "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
  // "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
  // "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
  // "10": "Take the ramp.",
  // "11": "Take the <BRANCH_SIGN> ramp.",
  // "12": "Take the ramp toward <TOWARD_SIGN>.",
  // "13": "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "14": "Take the <NAME_SIGN> ramp."

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;

  // Determine if turn, else it's a "Take" instruction
  if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight) ||
      (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft)) {
    phrase_id = 5;
    // Determine if driving side matches relative direction
  } else if (((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepRight) &&
              maneuver.drive_on_right()) ||
             ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepLeft) &&
              !maneuver.drive_on_right())) {
    phrase_id = 10;
  }

  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasExitNameSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count, delim,
                                           maneuver.verbal_formatter(), &markup_formatter_);
  }

  return FormVerbalRampInstruction(phrase_id,
                                   FormRelativeTwoDirection(maneuver.type(),
                                                            dictionary_.ramp_verbal_subset
                                                                .relative_directions),
                                   exit_branch_sign, exit_toward_sign, exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalRampInstruction(Maneuver& maneuver,
                                                        bool limit_by_consecutive_count,
                                                        uint32_t element_max_count,
                                                        const std::string& delim) {
  // "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
  // "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  // "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  // "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
  // "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
  // "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
  // "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
  // "10": "Take the ramp.",
  // "11": "Take the <BRANCH_SIGN> ramp.",
  // "12": "Take the ramp toward <TOWARD_SIGN>.",
  // "13": "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  // "14": "Take the <NAME_SIGN> ramp."

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;

  // Determine if turn, else it's a "Take" instruction
  if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight) ||
      (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft)) {
    phrase_id = 5;
    // Determine if driving side matches relative direction
  } else if (((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepRight) &&
              maneuver.drive_on_right()) ||
             ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepLeft) &&
              !maneuver.drive_on_right())) {
    phrase_id = 10;
  }

  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign() && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count, delim,
                                           maneuver.verbal_formatter(), &markup_formatter_);
  }

  return FormVerbalRampInstruction(phrase_id,
                                   FormRelativeTwoDirection(maneuver.type(),
                                                            dictionary_.ramp_verbal_subset
                                                                .relative_directions),
                                   exit_branch_sign, exit_toward_sign, exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalRampInstruction(uint8_t phrase_id,
                                                        const std::string& relative_dir,
                                                        const std::string& exit_branch_sign,
                                                        const std::string& exit_toward_sign,
                                                        const std::string& exit_name_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.ramp_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kBranchSignTag, exit_branch_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);
  boost::replace_all(instruction, kNameSignTag, exit_name_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormExitInstruction(Maneuver& maneuver,
                                                  bool limit_by_consecutive_count,
                                                  uint32_t element_max_count) {
  // "0": "Take the exit on the <RELATIVE_DIRECTION>.",
  // "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
  // "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
  // "3": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
  // "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "5": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "6": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "7": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward
  // <TOWARD_SIGN>.", "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.", "10": "Take the
  // <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.", "12": "Take the <NAME_SIGN>
  // exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.", "14": "Take the <NAME_SIGN> exit on the
  // <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
  // "15": "Take the exit.",
  // "16": "Take exit <NUMBER_SIGN>.",
  // "17": "Take the <BRANCH_SIGN> exit.",
  // "18": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN>.",
  // "19": "Take the exit toward <TOWARD_SIGN>.",
  // "20": "Take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
  // "21": "Take the <BRANCH_SIGN> exit toward <TOWARD_SIGN>.",
  // "22": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
  // "23": "Take the <NAME_SIGN> exit.",
  // "25": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN>.",
  // "27": "Take the <NAME_SIGN> exit toward <TOWARD_SIGN>.",
  // "29": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN> toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_number_sign;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;

  // Determine if driving side matches relative direction
  if (((maneuver.type() == DirectionsLeg_Maneuver_Type_kExitRight) && maneuver.drive_on_right()) ||
      ((maneuver.type() == DirectionsLeg_Maneuver_Type_kExitLeft) && !maneuver.drive_on_right())) {
    phrase_id = 15;
  }

  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitNumberSign()) {
    phrase_id += 8;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(),
                                              dictionary_.exit_subset.relative_directions));
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kBranchSignTag, exit_branch_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);
  boost::replace_all(instruction, kNameSignTag, exit_name_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertExitInstruction(Maneuver& maneuver,
                                                             bool limit_by_consecutive_count,
                                                             uint32_t element_max_count,
                                                             const std::string& delim) {
  // "0": "Take the exit on the <RELATIVE_DIRECTION>.",
  // "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
  // "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
  // "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.",
  // "15": "Take the exit.",
  // "16": "Take exit <NUMBER_SIGN>.",
  // "17": "Take the <BRANCH_SIGN> exit.",
  // "19": "Take the exit toward <TOWARD_SIGN>.",
  // "23": "Take the <NAME_SIGN> exit.",

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_number_sign;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;

  // Determine if driving side matches relative direction
  if (((maneuver.type() == DirectionsLeg_Maneuver_Type_kExitRight) && maneuver.drive_on_right()) ||
      ((maneuver.type() == DirectionsLeg_Maneuver_Type_kExitLeft) && !maneuver.drive_on_right())) {
    phrase_id = 15;
  }

  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter(),
                                             &markup_formatter_);
  } else if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasExitNameSign()) {
    phrase_id += 8;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count, delim,
                                           maneuver.verbal_formatter(), &markup_formatter_);
  }

  return FormVerbalExitInstruction(phrase_id,
                                   FormRelativeTwoDirection(maneuver.type(),
                                                            dictionary_.exit_verbal_subset
                                                                .relative_directions),
                                   exit_number_sign, exit_branch_sign, exit_toward_sign,
                                   exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalExitInstruction(Maneuver& maneuver,
                                                        bool limit_by_consecutive_count,
                                                        uint32_t element_max_count,
                                                        const std::string& delim) {
  // "0": "Take the exit on the <RELATIVE_DIRECTION>.",
  // "1": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>.",
  // "2": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>.",
  // "3": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.",
  // "4": "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "5": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "6": "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "7": "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward
  // <TOWARD_SIGN>.", "8": "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>.", "10": "Take the
  // <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>.", "12": "Take the <NAME_SIGN>
  // exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.", "14": "Take the <NAME_SIGN> exit on
  // the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
  // "15": "Take the exit.",
  // "16": "Take exit <NUMBER_SIGN>.",
  // "17": "Take the <BRANCH_SIGN> exit.",
  // "18": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN>.",
  // "19": "Take the exit toward <TOWARD_SIGN>.",
  // "20": "Take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
  // "21": "Take the <BRANCH_SIGN> exit toward <TOWARD_SIGN>.",
  // "22": "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN> toward <TOWARD_SIGN>.",
  // "23": "Take the <NAME_SIGN> exit.",
  // "25": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN>.",
  // "27": "Take the <NAME_SIGN> exit toward <TOWARD_SIGN>.",
  // "29": "Take the <NAME_SIGN> exit onto <BRANCH_SIGN> toward <TOWARD_SIGN>."

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string exit_number_sign;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;

  // Determine if driving side matches relative direction
  if (((maneuver.type() == DirectionsLeg_Maneuver_Type_kExitRight) && maneuver.drive_on_right()) ||
      ((maneuver.type() == DirectionsLeg_Maneuver_Type_kExitLeft) && !maneuver.drive_on_right())) {
    phrase_id = 15;
  }

  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter(),
                                             &markup_formatter_);
  }
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitNumberSign()) {
    phrase_id += 8;
    // Assign name sign
    exit_name_sign =
        maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count, delim,
                                           maneuver.verbal_formatter(), &markup_formatter_);
  }

  return FormVerbalExitInstruction(phrase_id,
                                   FormRelativeTwoDirection(maneuver.type(),
                                                            dictionary_.exit_verbal_subset
                                                                .relative_directions),
                                   exit_number_sign, exit_branch_sign, exit_toward_sign,
                                   exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalExitInstruction(uint8_t phrase_id,
                                                        const std::string& relative_dir,
                                                        const std::string& exit_number_sign,
                                                        const std::string& exit_branch_sign,
                                                        const std::string& exit_toward_sign,
                                                        const std::string& exit_name_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kBranchSignTag, exit_branch_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);
  boost::replace_all(instruction, kNameSignTag, exit_name_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormKeepInstruction(Maneuver& maneuver,
                                                  bool limit_by_consecutive_count,
                                                  uint32_t element_max_count) {
  // "0": "Keep <RELATIVE_DIRECTION> at the fork.",
  // "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
  // "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
  // "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>.",
  // "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
  // "6": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>.",
  // "7": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward
  // <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  std::string street_names;
  std::string exit_number_sign;
  std::string toward_sign;

  // If they exist, process guide signs
  if (maneuver.HasGuideSign()) {
    if (maneuver.HasGuideBranchSign()) {
      street_names =
          maneuver.signs().GetGuideBranchString(element_max_count, limit_by_consecutive_count);
    }
    if (maneuver.HasGuideTowardSign()) {
      // Assign guide sign
      toward_sign =
          maneuver.signs().GetGuideTowardString(element_max_count, limit_by_consecutive_count);
    }
  } else {
    // For ramps with branch sign info - we use the sign info to match what users are seeing
    if (maneuver.ramp() && maneuver.HasExitBranchSign()) {
      street_names =
          maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count);
    } else {
      street_names =
          FormStreetNames(maneuver, maneuver.street_names(),
                          &dictionary_.keep_subset.empty_street_name_labels, true, element_max_count);

      // If street names string is empty and the maneuver has sign branch info
      // then assign the sign branch name to the street names string
      if (street_names.empty() && maneuver.HasExitBranchSign()) {
        street_names =
            maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count);
      }
    }

    // If it exists, process the exit toward sign
    if (maneuver.HasExitTowardSign()) {
      // Assign toward sign
      toward_sign =
          maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count);
    }
  }

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (!street_names.empty()) {
    phrase_id += 2;
  }
  if (!toward_sign.empty()) {
    phrase_id += 4;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeThreeDirection(maneuver.type(),
                                                dictionary_.keep_subset.relative_directions));
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, toward_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertKeepInstruction(Maneuver& maneuver,
                                                             bool limit_by_consecutive_count,
                                                             uint32_t element_max_count,
                                                             const std::string& delim) {

  // "0": "Keep <RELATIVE_DIRECTION> at the fork.",
  // "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
  // "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
  // "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",

  std::string street_names;
  std::string exit_number_sign;
  std::string toward_sign;

  // If they exist, process guide signs
  if (maneuver.HasGuideSign()) {
    if (maneuver.HasGuideBranchSign()) {
      street_names =
          maneuver.signs().GetGuideBranchString(element_max_count, limit_by_consecutive_count, delim,
                                                maneuver.verbal_formatter(), &markup_formatter_);
    }
    if (maneuver.HasGuideTowardSign()) {
      // Assign guide sign
      toward_sign =
          maneuver.signs().GetGuideTowardString(element_max_count, limit_by_consecutive_count, delim,
                                                maneuver.verbal_formatter(), &markup_formatter_);
    }
  } else {
    // For ramps with branch sign info - we use the sign info to match what users are seeing
    if (maneuver.ramp() && maneuver.HasExitBranchSign()) {
      street_names =
          maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
    } else {
      // Assign the street names
      street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                     &dictionary_.keep_verbal_subset.empty_street_name_labels, true,
                                     element_max_count, delim, maneuver.verbal_formatter());

      // If street names string is empty and the maneuver has sign branch info
      // then assign the sign branch name to the street names string
      if (street_names.empty() && maneuver.HasExitBranchSign()) {
        street_names =
            maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
      }
    }

    // If it exists, process exit toward sign
    if (maneuver.HasExitTowardSign()) {
      // Assign toward sign
      toward_sign =
          maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
    }
  }

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter(),
                                             &markup_formatter_);
  } else if (!street_names.empty()) {
    phrase_id += 2;
  } else if (!toward_sign.empty()) {
    phrase_id += 4;
  }

  return FormVerbalKeepInstruction(phrase_id,
                                   FormRelativeThreeDirection(maneuver.type(),
                                                              dictionary_.keep_verbal_subset
                                                                  .relative_directions),
                                   street_names, exit_number_sign, toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepInstruction(Maneuver& maneuver,
                                                        bool limit_by_consecutive_count,
                                                        uint32_t element_max_count,
                                                        const std::string& delim) {

  // "0": "Keep <RELATIVE_DIRECTION> at the fork.",
  // "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>.",
  // "2": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>.",
  // "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>.",
  // "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  // "5": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>.",
  // "6": "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>.",
  // "7": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward
  // <TOWARD_SIGN>."

  std::string exit_number_sign;
  std::string toward_sign;
  std::string street_names;

  // If they exist, process guide signs
  if (maneuver.HasGuideSign()) {
    if (maneuver.HasGuideBranchSign()) {
      street_names =
          maneuver.signs().GetGuideBranchString(element_max_count, limit_by_consecutive_count, delim,
                                                maneuver.verbal_formatter(), &markup_formatter_);
    }
    if (maneuver.HasGuideTowardSign()) {
      // Assign guide sign
      toward_sign =
          maneuver.signs().GetGuideTowardString(element_max_count, limit_by_consecutive_count, delim,
                                                maneuver.verbal_formatter(), &markup_formatter_);
    }
  } else {
    // For ramps with branch sign info - we use the sign info to match what users are seeing
    if (maneuver.ramp() && maneuver.HasExitBranchSign()) {
      street_names =
          maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
    } else {
      // Assign the street names
      street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                     &dictionary_.keep_verbal_subset.empty_street_name_labels, true,
                                     element_max_count, delim, maneuver.verbal_formatter());

      // If street names string is empty and the maneuver has sign branch info
      // then assign the sign branch name to the street names string
      if (street_names.empty() && maneuver.HasExitBranchSign()) {
        street_names =
            maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
      }
    }

    // If it exists, process exit toward sign
    if (maneuver.HasExitTowardSign()) {
      // Assign toward sign
      toward_sign =
          maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
    }
  }

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter(),
                                             &markup_formatter_);
  }
  if (!street_names.empty()) {
    phrase_id += 2;
  }
  if (!toward_sign.empty()) {
    phrase_id += 4;
  }

  return FormVerbalKeepInstruction(phrase_id,
                                   FormRelativeThreeDirection(maneuver.type(),
                                                              dictionary_.keep_verbal_subset
                                                                  .relative_directions),
                                   street_names, exit_number_sign, toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepInstruction(uint8_t phrase_id,
                                                        const std::string& relative_dir,
                                                        const std::string& street_names,
                                                        const std::string& exit_number_sign,
                                                        const std::string& toward_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, toward_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormKeepToStayOnInstruction(Maneuver& maneuver,
                                                          bool limit_by_consecutive_count,
                                                          uint32_t element_max_count) {

  // "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
  // "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>.",
  // "2": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>.",
  // "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward
  //      <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.keep_to_stay_on_subset.empty_street_name_labels, true,
                      element_max_count);

  // If they exist, process guide toward signs
  // else use exit toward signs
  std::string toward_sign;
  if (maneuver.HasGuideTowardSign()) {
    // Assign guide sign
    toward_sign =
        maneuver.signs().GetGuideTowardString(element_max_count, limit_by_consecutive_count);
  } else if (maneuver.HasExitTowardSign()) {
    toward_sign = maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count);
  }

  // Determine which phrase to use
  std::string exit_number_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (!toward_sign.empty()) {
    phrase_id += 2;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_to_stay_on_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeThreeDirection(maneuver.type(), dictionary_.keep_to_stay_on_subset
                                                                     .relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kTowardSignTag, toward_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertKeepToStayOnInstruction(Maneuver& maneuver,
                                                                     uint32_t element_max_count,
                                                                     const std::string& delim) {

  // "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.keep_to_stay_on_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  return FormVerbalKeepToStayOnInstruction(
      0,
      FormRelativeThreeDirection(maneuver.type(),
                                 dictionary_.keep_to_stay_on_verbal_subset.relative_directions),
      street_names);
}

std::string NarrativeBuilder::FormVerbalKeepToStayOnInstruction(Maneuver& maneuver,
                                                                bool limit_by_consecutive_count,
                                                                uint32_t element_max_count,
                                                                const std::string& delim) {

  // "0": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.",
  // "1": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>.",
  // "2": "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>.",
  // "3": "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward
  // <TOWARD_SIGN>."

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.keep_to_stay_on_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // If they exist, process guide toward signs
  // else use exit toward signs
  std::string toward_sign;
  if (maneuver.HasGuideTowardSign()) {
    // Assign guide sign
    toward_sign =
        maneuver.signs().GetGuideTowardString(element_max_count, limit_by_consecutive_count, delim,
                                              maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasExitTowardSign()) {
    toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter(), &markup_formatter_);
  }

  // Determine which phrase to use
  std::string exit_number_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter(),
                                             &markup_formatter_);
  }
  if (!toward_sign.empty()) {
    phrase_id += 2;
  }

  return FormVerbalKeepToStayOnInstruction(
      phrase_id,
      FormRelativeThreeDirection(maneuver.type(),
                                 dictionary_.keep_to_stay_on_verbal_subset.relative_directions),
      street_names, exit_number_sign, toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepToStayOnInstruction(uint8_t phrase_id,
                                                                const std::string& relative_dir,
                                                                const std::string& street_names,
                                                                const std::string& exit_number_sign,
                                                                const std::string& toward_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_to_stay_on_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kTowardSignTag, toward_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormMergeInstruction(Maneuver& maneuver,
                                                   bool limit_by_consecutive_count,
                                                   uint32_t element_max_count) {
  // "0": "Merge."
  // "1": "Merge <RELATIVE_DIRECTION>."
  // "2": "Merge onto <STREET_NAMES>."
  // "3": "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."
  // "4": "Merge toward <TOWARD_SIGN>."
  // "5": "Merge <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.merge_subset.empty_street_name_labels, true);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string guide_sign;

  if (!street_names.empty()) {
    // Street names take priority over toward phrase
    phrase_id = 2;
  } else if (maneuver.HasGuideSign()) {
    // Use toward phrase if street names is empty
    phrase_id = 4;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count);
  }

  // Check for merge relative direction
  std::string relative_direction;
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeLeft) ||
      (maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeRight)) {
    phrase_id += 1;
    relative_direction =
        FormRelativeTwoDirection(maneuver.type(), dictionary_.merge_subset.relative_directions);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.merge_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertMergeInstruction(Maneuver& maneuver,
                                                              bool limit_by_consecutive_count,
                                                              uint32_t element_max_count,
                                                              const std::string& delim) {
  // "0": "Merge."
  // "1": "Merge <RELATIVE_DIRECTION>."
  // "2": "Merge onto <STREET_NAMES>."
  // "3": "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."
  // "4": "Merge toward <TOWARD_SIGN>."
  // "5": "Merge <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  return FormVerbalMergeInstruction(maneuver, limit_by_consecutive_count, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalMergeInstruction(Maneuver& maneuver,
                                                         bool limit_by_consecutive_count,
                                                         uint32_t element_max_count,
                                                         const std::string& delim) {
  // "0": "Merge."
  // "1": "Merge <RELATIVE_DIRECTION>."
  // "2": "Merge onto <STREET_NAMES>."
  // "3": "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."
  // "4": "Merge toward <TOWARD_SIGN>."
  // "5": "Merge <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.merge_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string guide_sign;

  if (!street_names.empty()) {
    // Street names take priority over toward phrase
    phrase_id = 2;
  } else if (maneuver.HasGuideSign()) {
    // Use toward phrase if street names is empty
    phrase_id = 4;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  }

  // Check for merge relative direction
  std::string relative_direction;
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeLeft) ||
      (maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeRight)) {
    phrase_id += 1;
    relative_direction =
        FormRelativeTwoDirection(maneuver.type(),
                                 dictionary_.merge_verbal_subset.relative_directions);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.merge_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormEnterRoundaboutInstruction(Maneuver& maneuver,
                                                             bool limit_by_consecutive_count,
                                                             uint32_t element_max_count) {
  // "0": "Enter the roundabout.",
  // "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit.",
  // "2",
  // "Enter the roundabout and take the <ORDINAL_VALUE> exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "3",
  // "Enter the roundabout and take the <ORDINAL_VALUE> exit onto
  // <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>. Continue on <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "4": "Enter the roundabout and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>.",
  // "5": "Enter the roundabout and take the exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "6",
  // "Enter the roundabout and take the exit onto <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>. Continue on
  // <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "7": "Enter the roundabout and take the exit toward <TOWARD_SIGN>.",
  // "8": "Enter <STREET_NAMES>",
  // "9": "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit.",
  // "10",
  // "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "11",
  // "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit onto
  // <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>. Continue on <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "12": "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>.",
  // "13": "Enter <STREET_NAMES> and take the exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "14",
  // "Enter <STREET_NAMES> and take the exit onto <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>. Continue on
  // <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "15": "Enter <STREET_NAMES> and take the exit toward <TOWARD_SIGN>.";

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names());

  std::string roundabout_exit_street_names;
  std::string roundabout_exit_begin_street_names;

  // TODO - in the future this value will come from the options_ member variable
  bool option_roundabout_exits = true;
  // If we are creating roundabout exit maneuvers
  // then only assign the roundabout exit street names
  if (option_roundabout_exits) {
    if (maneuver.roundabout_exit_begin_street_names().empty()) {
      // Use the street names
      roundabout_exit_street_names =
          FormStreetNames(maneuver, maneuver.roundabout_exit_street_names());
    } else {
      // Use the begin street names
      roundabout_exit_street_names =
          FormStreetNames(maneuver, maneuver.roundabout_exit_begin_street_names());
    }
  } else {
    // Assign the roundabout exit street names
    // We can use empty_street_name_labels when enter/exit maneuvers are combined
    roundabout_exit_street_names =
        FormStreetNames(maneuver, maneuver.roundabout_exit_street_names(),
                        &dictionary_.enter_roundabout_subset.empty_street_name_labels, true);

    // Assign the roundabout exit begin street names
    roundabout_exit_begin_street_names =
        FormStreetNames(maneuver, maneuver.roundabout_exit_begin_street_names());
  }

  // Determine which phrase to use - start with unnamed roundabout base phrase
  uint8_t phrase_id = 0;
  std::string guide_sign;

  // Determine between unnamed roundabout vs named roundabout
  if (!street_names.empty()) {
    // Assign named roundabout base phrase
    phrase_id = 8;
  }

  // Determine if we are using an ordinal value
  std::string ordinal_value;
  if ((maneuver.roundabout_exit_count() >= kRoundaboutExitCountLowerBound) &&
      (maneuver.roundabout_exit_count() <= kRoundaboutExitCountUpperBound)) {
    // Increment for ordinal phrase
    phrase_id += 1;
    // Set ordinal_value
    ordinal_value =
        dictionary_.enter_roundabout_subset.ordinal_values.at(maneuver.roundabout_exit_count() - 1);
  } else if (!roundabout_exit_street_names.empty() || !roundabout_exit_begin_street_names.empty() ||
             maneuver.roundabout_exit_signs().HasGuide()) {
    // Skip to the non-ordinal phrase with additional info
    phrase_id += 4;
  }

  if (maneuver.roundabout_exit_signs().HasGuide()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id += 3;
    // Assign guide sign
    guide_sign = maneuver.roundabout_exit_signs().GetGuideString(element_max_count,
                                                                 limit_by_consecutive_count);
  } else {
    if (!roundabout_exit_street_names.empty()) {
      // Increment for roundabout exit street name phrase
      phrase_id += 1;
    }
    if (!roundabout_exit_begin_street_names.empty()) {
      // Increment for roundabout exit begin street name phrase
      phrase_id += 1;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_roundabout_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kOrdinalValueTag, ordinal_value);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);
  boost::replace_all(instruction, kRoundaboutExitStreetNamesTag, roundabout_exit_street_names);
  boost::replace_all(instruction, kRoundaboutExitBeginStreetNamesTag,
                     roundabout_exit_begin_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string
NarrativeBuilder::FormVerbalAlertEnterRoundaboutInstruction(Maneuver& maneuver,
                                                            bool limit_by_consecutive_count,
                                                            uint32_t element_max_count,
                                                            const std::string& delim) {
  // "0": "Enter the roundabout.",
  // "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit.",
  // "2",
  // "Enter the roundabout and take the <ORDINAL_VALUE> exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "3",
  // "Enter the roundabout and take the <ORDINAL_VALUE> exit onto
  // <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "4": "Enter the roundabout and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>.",
  // "5": "Enter the roundabout and take the exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "6": "Enter the roundabout and take the exit onto <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "7": "Enter the roundabout and take the exit toward <TOWARD_SIGN>.",
  // "8": "Enter <STREET_NAMES>",
  // "9": "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit.",
  // "10",
  // "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "11",
  // "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit onto
  // <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "12": "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>.",
  // "13": "Enter <STREET_NAMES> and take the exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "14": "Enter <STREET_NAMES> and take the exit onto <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "15": "Enter <STREET_NAMES> and take the exit toward <TOWARD_SIGN>.";

  return FormVerbalEnterRoundaboutInstruction(maneuver, limit_by_consecutive_count, element_max_count,
                                              delim);
}

std::string NarrativeBuilder::FormVerbalEnterRoundaboutInstruction(Maneuver& maneuver,
                                                                   bool limit_by_consecutive_count,
                                                                   uint32_t element_max_count,
                                                                   const std::string& delim) {
  // "0": "Enter the roundabout.",
  // "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit.",
  // "2",
  // "Enter the roundabout and take the <ORDINAL_VALUE> exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "3",
  // "Enter the roundabout and take the <ORDINAL_VALUE> exit onto
  // <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "4": "Enter the roundabout and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>.",
  // "5": "Enter the roundabout and take the exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "6": "Enter the roundabout and take the exit onto <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "7": "Enter the roundabout and take the exit toward <TOWARD_SIGN>.",
  // "8": "Enter <STREET_NAMES>",
  // "9": "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit.",
  // "10",
  // "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "11",
  // "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit onto
  // <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "12": "Enter <STREET_NAMES> and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>.",
  // "13": "Enter <STREET_NAMES> and take the exit onto <ROUNDABOUT_EXIT_STREET_NAMES>.",
  // "14": "Enter <STREET_NAMES> and take the exit onto <ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>.",
  // "15": "Enter <STREET_NAMES> and take the exit toward <TOWARD_SIGN>.";

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.enter_roundabout_verbal_subset.empty_street_name_labels, false,
                      element_max_count, delim, maneuver.verbal_formatter());

  // TODO - in the future this value will come from the options_ member variable
  bool option_roundabout_exits = true;
  bool enhance_empty_street_names = false;
  // Only enhance empty street names if enter/exit are combined
  if (!option_roundabout_exits) {
    enhance_empty_street_names = true;
  }

  // Assign the roundabout exit street names
  std::string roundabout_exit_street_names =
      FormStreetNames(maneuver, maneuver.roundabout_exit_street_names(),
                      &dictionary_.enter_roundabout_verbal_subset.empty_street_name_labels,
                      enhance_empty_street_names, element_max_count, delim,
                      maneuver.verbal_formatter());

  // Assign the roundabout exit begin street names
  std::string roundabout_exit_begin_street_names =
      FormStreetNames(maneuver, maneuver.roundabout_exit_begin_street_names(),
                      &dictionary_.enter_roundabout_verbal_subset.empty_street_name_labels, false,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use - start with unnamed roundabout base phrase
  uint8_t phrase_id = 0;
  std::string guide_sign;

  // Determine between unnamed roundabout vs named roundabout
  if (!street_names.empty()) {
    // Assign named roundabout base phrase
    phrase_id = 8;
  }

  // Determine if we are using an ordinal value
  std::string ordinal_value;
  if ((maneuver.roundabout_exit_count() >= kRoundaboutExitCountLowerBound) &&
      (maneuver.roundabout_exit_count() <= kRoundaboutExitCountUpperBound)) {
    // Increment for ordinal phrase
    phrase_id += 1;
    // Set ordinal_value
    ordinal_value = dictionary_.enter_roundabout_verbal_subset.ordinal_values.at(
        maneuver.roundabout_exit_count() - 1);
  } else if (!roundabout_exit_street_names.empty() || !roundabout_exit_begin_street_names.empty() ||
             maneuver.roundabout_exit_signs().HasGuide()) {
    // Skip to the non-ordinal phrase with additional info
    phrase_id += 4;
  }

  if (maneuver.roundabout_exit_signs().HasGuide()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id += 3;
    // Assign guide sign
    guide_sign =
        maneuver.roundabout_exit_signs().GetGuideString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter(),
                                                        &markup_formatter_);
  } else {
    if (!roundabout_exit_street_names.empty()) {
      // Increment for roundabout exit street name phrase
      phrase_id += 1;
    }
    if (!roundabout_exit_begin_street_names.empty()) {
      // Increment for roundabout exit begin street name phrase
      phrase_id += 1;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_roundabout_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kOrdinalValueTag, ordinal_value);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);
  boost::replace_all(instruction, kRoundaboutExitStreetNamesTag, roundabout_exit_street_names);
  boost::replace_all(instruction, kRoundaboutExitBeginStreetNamesTag,
                     roundabout_exit_begin_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormExitRoundaboutInstruction(Maneuver& maneuver,
                                                            bool limit_by_consecutive_count,
                                                            uint32_t element_max_count) {
  // "0": "Exit the roundabout.",
  // "1": "Exit the roundabout onto <STREET_NAMES>.",
  // "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",
  // "3": "Exit the roundabout toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.exit_roundabout_subset.empty_street_name_labels, true);

  // Assign the begin street names
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(),
                      &dictionary_.exit_roundabout_subset.empty_street_name_labels);

  // Update street names for maneuvers that contain obvious maneuvers
  UpdateObviousManeuverStreetNames(maneuver, begin_street_names, street_names);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id = 3;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count);
  } else {
    if (!street_names.empty()) {
      // Increment for street name phrase
      phrase_id += 1;
    }
    if (!begin_street_names.empty()) {
      // Increment for begin street name phrase
      phrase_id += 1;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_roundabout_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalExitRoundaboutInstruction(Maneuver& maneuver,
                                                                  bool limit_by_consecutive_count,
                                                                  uint32_t element_max_count,
                                                                  const std::string& delim) {
  // "0": "Exit the roundabout.",
  // "1": "Exit the roundabout onto <STREET_NAMES>.",
  // "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>.",
  // "3": "Exit the roundabout toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.exit_roundabout_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Assign the begin street names
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(),
                      &dictionary_.exit_roundabout_verbal_subset.empty_street_name_labels, false,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Update street names for maneuvers that contain obvious maneuvers
  UpdateObviousManeuverStreetNames(maneuver, begin_street_names, street_names);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id = 3;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else {
    if (!street_names.empty()) {
      // Increment for street name phrase
      phrase_id += 1;
    }
    if (!begin_street_names.empty()) {
      // Increment for begin street name phrase
      phrase_id += 1;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_roundabout_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormEnterFerryInstruction(Maneuver& maneuver,
                                                        bool limit_by_consecutive_count,
                                                        uint32_t element_max_count) {
  // "0": "Take the Ferry."
  // "1": "Take the <STREET_NAMES>."
  // "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
  // "3": "Take the ferry toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.enter_ferry_subset.empty_street_name_labels, true);

  std::string ferry_label = dictionary_.enter_ferry_subset.ferry_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id = 3;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count);
  } else if (!street_names.empty()) {
    phrase_id = 1;
    if (!HasLabel(street_names, ferry_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_ferry_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kFerryLabelTag, ferry_label);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertEnterFerryInstruction(Maneuver& maneuver,
                                                                   bool limit_by_consecutive_count,
                                                                   uint32_t element_max_count,
                                                                   const std::string& delim) {
  // "0": "Take the Ferry."
  // "1": "Take the <STREET_NAMES>."
  // "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
  // "3": "Take the ferry toward <TOWARD_SIGN>."

  return FormVerbalEnterFerryInstruction(maneuver, limit_by_consecutive_count, element_max_count,
                                         delim);
}

std::string NarrativeBuilder::FormVerbalEnterFerryInstruction(Maneuver& maneuver,
                                                              bool limit_by_consecutive_count,
                                                              uint32_t element_max_count,
                                                              const std::string& delim) {
  // "0": "Take the Ferry."
  // "1": "Take the <STREET_NAMES>."
  // "2": "Take the <STREET_NAMES> <FERRY_LABEL>."
  // "3": "Take the ferry toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.enter_ferry_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  std::string ferry_label = dictionary_.enter_ferry_verbal_subset.ferry_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;

  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id = 3;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (!street_names.empty()) {
    phrase_id = 1;
    if (!HasLabel(street_names, ferry_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_ferry_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kFerryLabelTag, ferry_label);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitConnectionStartInstruction(Maneuver& maneuver) {
  // "0": "Enter the station.",
  // "1": "Enter the <TRANSIT_STOP>.",
  // "2": "Enter the <TRANSIT_STOP> <STATION_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign transit stop
  std::string transit_stop = maneuver.transit_connection_platform_info().name();

  // Assign station label
  std::string station_label = dictionary_.transit_connection_start_subset.station_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!transit_stop.empty()) {
    phrase_id = 1;
    if (!HasLabel(transit_stop, station_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_connection_start_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop);
  boost::replace_all(instruction, kStationLabelTag, station_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitConnectionStartInstruction(Maneuver& maneuver) {
  // "0": "Enter the station.",
  // "1": "Enter the <TRANSIT_STOP>.",
  // "2": "Enter the <TRANSIT_STOP> <STATION_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign transit stop
  std::string transit_stop = maneuver.transit_connection_platform_info().name();

  // Assign station label
  std::string station_label = dictionary_.transit_connection_start_verbal_subset.station_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!transit_stop.empty()) {
    phrase_id = 1;
    if (!HasLabel(transit_stop, station_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction =
      dictionary_.transit_connection_start_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop);
  boost::replace_all(instruction, kStationLabelTag, station_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitConnectionTransferInstruction(Maneuver& maneuver) {
  // "0": "Transfer at the station.",
  // "1": "Transfer at the <TRANSIT_STOP>.",
  // "2": "Transfer at the <TRANSIT_STOP> <STATION_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign transit stop
  std::string transit_stop = maneuver.transit_connection_platform_info().name();

  // Assign station label
  std::string station_label = dictionary_.transit_connection_transfer_subset.station_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!transit_stop.empty()) {
    phrase_id = 1;
    if (!HasLabel(transit_stop, station_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_connection_transfer_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop);
  boost::replace_all(instruction, kStationLabelTag, station_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitConnectionTransferInstruction(Maneuver& maneuver) {
  // "0": "Transfer at the station.",
  // "1": "Transfer at the <TRANSIT_STOP>.",
  // "2": "Transfer at the <TRANSIT_STOP> <STATION_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign transit stop
  std::string transit_stop = maneuver.transit_connection_platform_info().name();

  // Assign station label
  std::string station_label = dictionary_.transit_connection_transfer_verbal_subset.station_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!transit_stop.empty()) {
    phrase_id = 1;
    if (!HasLabel(transit_stop, station_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction =
      dictionary_.transit_connection_transfer_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop);
  boost::replace_all(instruction, kStationLabelTag, station_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitConnectionDestinationInstruction(Maneuver& maneuver) {
  // "0": "Exit the station.",
  // "1": "Exit the <TRANSIT_STOP>.",
  // "2": "Exit the <TRANSIT_STOP> <STATION_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign transit stop
  std::string transit_stop = maneuver.transit_connection_platform_info().name();

  // Assign station label
  std::string station_label = dictionary_.transit_connection_destination_subset.station_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!transit_stop.empty()) {
    phrase_id = 1;
    if (!HasLabel(transit_stop, station_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction =
      dictionary_.transit_connection_destination_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop);
  boost::replace_all(instruction, kStationLabelTag, station_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitConnectionDestinationInstruction(Maneuver& maneuver) {
  // "0": "Exit the station.",
  // "1": "Exit the <TRANSIT_STOP>.",
  // "2": "Exit the <TRANSIT_STOP> <STATION_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign transit stop
  std::string transit_stop = maneuver.transit_connection_platform_info().name();

  // Assign station label
  std::string station_label = dictionary_.transit_connection_destination_verbal_subset.station_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!transit_stop.empty()) {
    phrase_id = 1;
    if (!HasLabel(transit_stop, station_label)) {
      phrase_id = 2;
    }
  }

  // Set instruction to the determined tagged phrase
  instruction =
      dictionary_.transit_connection_destination_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop);
  boost::replace_all(instruction, kStationLabelTag, station_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormDepartInstruction(Maneuver& maneuver) {
  // "0": "Depart: <TIME>",
  // "1": "Depart: <TIME> from <TRANSIT_STOP>"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().front().name();

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.depart_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop_name);
  boost::replace_all(instruction, kTimeTag,
                     get_localized_time(maneuver.GetTransitDepartureTime(), dictionary_.GetLocale()));

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalDepartInstruction(Maneuver& maneuver) {
  // "0": "Depart at <TIME>",
  // "1": "Depart at <TIME> from <TRANSIT_STOP>"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().front().name();

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.depart_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop_name);
  boost::replace_all(instruction, kTimeTag,
                     get_localized_time(maneuver.GetTransitDepartureTime(), dictionary_.GetLocale()));

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormArriveInstruction(Maneuver& maneuver) {
  // "0": "Arrive: <TIME>",
  // "1": "Arrive: <TIME> at <TRANSIT_STOP>"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().back().name();

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.arrive_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop_name);
  boost::replace_all(instruction, kTimeTag,
                     get_localized_time(maneuver.GetTransitArrivalTime(), dictionary_.GetLocale()));

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalArriveInstruction(Maneuver& maneuver) {
  // "0": "Arrive at <TIME>",
  // "1": "Arrive at <TIME> at <TRANSIT_STOP>"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().back().name();

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.arrive_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformTag, transit_stop_name);
  boost::replace_all(instruction, kTimeTag,
                     get_localized_time(maneuver.GetTransitArrivalTime(), dictionary_.GetLocale()));

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitInstruction(Maneuver& maneuver) {
  // "0": "Take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
  // "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
  // <TRANSIT_STOP_COUNT_LABEL>)"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_info().headsign;
  auto stop_count = maneuver.GetTransitStopCount();
  auto stop_count_label =
      FormTransitPlatformCountLabel(stop_count, dictionary_.transit_subset.transit_stop_count_labels);

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitNameTag,
                     FormTransitName(maneuver, dictionary_.transit_subset.empty_transit_name_labels));
  boost::replace_all(instruction, kTransitHeadSignTag, transit_headsign);
  boost::replace_all(instruction, kTransitPlatformCountTag,
                     std::to_string(stop_count)); // TODO: locale specific numerals
  boost::replace_all(instruction, kTransitPlatformCountLabelTag, stop_count_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitInstruction(Maneuver& maneuver) {
  // "0": "Take the <TRANSIT_NAME>.",
  // "1": "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitNameTag,
                     FormTransitName(maneuver,
                                     dictionary_.transit_verbal_subset.empty_transit_name_labels));
  boost::replace_all(instruction, kTransitHeadSignTag, transit_headsign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitRemainOnInstruction(Maneuver& maneuver) {
  // "0": "Remain on the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)",
  // "1": "Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT>
  // <TRANSIT_STOP_COUNT_LABEL>)"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_info().headsign;
  auto stop_count = maneuver.GetTransitStopCount();
  auto stop_count_label =
      FormTransitPlatformCountLabel(stop_count,
                                    dictionary_.transit_remain_on_subset.transit_stop_count_labels);

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_remain_on_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitNameTag,
                     FormTransitName(maneuver,
                                     dictionary_.transit_remain_on_subset.empty_transit_name_labels));
  boost::replace_all(instruction, kTransitHeadSignTag, transit_headsign);
  boost::replace_all(instruction, kTransitPlatformCountTag,
                     std::to_string(stop_count)); // TODO: locale specific numerals
  boost::replace_all(instruction, kTransitPlatformCountLabelTag, stop_count_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitRemainOnInstruction(Maneuver& maneuver) {
  // "0": "Remain on the <TRANSIT_NAME>.",
  // "1": "Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_remain_on_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitNameTag,
                     FormTransitName(maneuver, dictionary_.transit_remain_on_verbal_subset
                                                   .empty_transit_name_labels));
  boost::replace_all(instruction, kTransitHeadSignTag, transit_headsign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitTransferInstruction(Maneuver& maneuver) {
  // "0": "Transfer to take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT>
  // <TRANSIT_STOP_COUNT_LABEL>)", "1": "Transfer to take the <TRANSIT_NAME> toward
  // <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_info().headsign;
  auto stop_count = maneuver.GetTransitStopCount();
  auto stop_count_label =
      FormTransitPlatformCountLabel(stop_count,
                                    dictionary_.transit_transfer_subset.transit_stop_count_labels);

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_transfer_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitNameTag,
                     FormTransitName(maneuver,
                                     dictionary_.transit_transfer_subset.empty_transit_name_labels));
  boost::replace_all(instruction, kTransitHeadSignTag, transit_headsign);
  boost::replace_all(instruction, kTransitPlatformCountTag,
                     std::to_string(stop_count)); // TODO: locale specific numerals
  boost::replace_all(instruction, kTransitPlatformCountLabelTag, stop_count_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitTransferInstruction(Maneuver& maneuver) {
  // "0": "Transfer to take the <TRANSIT_NAME>.",
  // "1": "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.transit_transfer_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitNameTag,
                     FormTransitName(maneuver, dictionary_.transit_transfer_verbal_subset
                                                   .empty_transit_name_labels));
  boost::replace_all(instruction, kTransitHeadSignTag, transit_headsign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalPostTransitionInstruction(Maneuver& maneuver,
                                                                  bool include_street_names,
                                                                  uint32_t element_max_count,
                                                                  const std::string& delim) {
  // "0": "Continue for <LENGTH>.",
  // "1": "Continue on <STREET_NAMES> for <LENGTH>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names if maneuver does not contain an obvious maneuver
  std::string street_names;
  if (!maneuver.contains_obvious_maneuver() && !maneuver.has_long_street_name()) {
    // Use the maneuver roundabout_exit_street_names
    // if the maneuver has a combined enter/exit roundabout instruction
    // otherwise use the maneuver street names
    const StreetNames& street_name_list =
        (maneuver.has_combined_enter_exit_roundabout() ? maneuver.roundabout_exit_street_names()
                                                       : maneuver.street_names());
    street_names =
        FormStreetNames(maneuver, street_name_list,
                        &dictionary_.post_transition_verbal_subset.empty_street_name_labels, true,
                        element_max_count, delim, maneuver.verbal_formatter());
  }

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (include_street_names && !street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.post_transition_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kLengthTag,
                     FormLength(maneuver, dictionary_.post_transition_verbal_subset.metric_lengths,
                                dictionary_.post_transition_verbal_subset.us_customary_lengths));
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalPostTransitionTransitInstruction(Maneuver& maneuver) {
  // "0": "Travel <TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  auto stop_count = maneuver.GetTransitStopCount();
  auto stop_count_label =
      FormTransitPlatformCountLabel(stop_count, dictionary_.post_transition_transit_verbal_subset
                                                    .transit_stop_count_labels);

  // Set instruction to the determined tagged phrase
  instruction =
      dictionary_.post_transition_transit_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kTransitPlatformCountTag,
                     std::to_string(stop_count)); // TODO: locale specific numerals
  boost::replace_all(instruction, kTransitPlatformCountLabelTag, stop_count_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalSuccinctStartTransitionInstruction(Maneuver& maneuver) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> for <LENGTH>.",
  // "5": "Drive <CARDINAL_DIRECTION>.",
  // "6": "Drive <CARDINAL_DIRECTION> for <LENGTH>.",
  // "10": "Walk <CARDINAL_DIRECTION>.",
  // "11": "Walk <CARDINAL_DIRECTION> for <LENGTH>.",
  // "15": "Bike <CARDINAL_DIRECTION>."
  // "16": "Bike <CARDINAL_DIRECTION> for <LENGTH>.",

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set cardinal_direction value
  std::string cardinal_direction =
      dictionary_.start_verbal_subset.cardinal_directions.at(maneuver.begin_cardinal_direction());

  // Determine which phrase to use
  uint8_t phrase_id = 0;

  // Set base phrase id per mode
  if (maneuver.travel_mode() == TravelMode::kDrive) {
    phrase_id += 5;
  } else if (maneuver.travel_mode() == TravelMode::kPedestrian) {
    phrase_id += 10;
  } else if (maneuver.travel_mode() == TravelMode::kBicycle) {
    phrase_id += 15;
  }

  if (maneuver.include_verbal_pre_transition_length()) {
    // Increment phrase id for length
    phrase_id += 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.start_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kCardinalDirectionTag, cardinal_direction);
  boost::replace_all(instruction, kLengthTag,
                     FormLength(maneuver, dictionary_.start_verbal_subset.metric_lengths,
                                dictionary_.start_verbal_subset.us_customary_lengths));

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string
NarrativeBuilder::FormVerbalSuccinctTurnTransitionInstruction(Maneuver& maneuver,
                                                              bool limit_by_consecutive_count,
                                                              uint32_t element_max_count,
                                                              const std::string& delim) {

  // "0": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION>."
  // "4": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> at <JUNCTION_NAME>."
  // "5": "Turn/Bear/Make a sharp <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  const TurnSubset* subset = nullptr;
  switch (maneuver.type()) {
    case DirectionsLeg_Maneuver_Type_kSlightRight:
    case DirectionsLeg_Maneuver_Type_kSlightLeft:
      subset = &dictionary_.bear_verbal_subset;
      break;
    case DirectionsLeg_Maneuver_Type_kRight:
    case DirectionsLeg_Maneuver_Type_kLeft:
      subset = &dictionary_.turn_verbal_subset;
      break;
    case DirectionsLeg_Maneuver_Type_kSharpRight:
    case DirectionsLeg_Maneuver_Type_kSharpLeft:
      subset = &dictionary_.sharp_verbal_subset;
      break;
    default:
      throw valhalla_exception_t{230};
  }

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 5;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 4;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
  }

  // Set instruction to the determined tagged phrase
  instruction = subset->phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(), subset->relative_directions));
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string
NarrativeBuilder::FormVerbalSuccinctUturnTransitionInstruction(Maneuver& maneuver,
                                                               bool limit_by_consecutive_count,
                                                               uint32_t element_max_count,
                                                               const std::string& delim) {
  // "0": "Make a <RELATIVE_DIRECTION> U-turn."
  // "6": "Make a <RELATIVE_DIRECTION> U-turn at <JUNCTION_NAME>."
  // "7": "Make a <RELATIVE_DIRECTION> U-turn toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string junction_name;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Set the toward phrase - it takes priority over street names and junction name
    phrase_id = 7;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  } else if (maneuver.HasJunctionNameSign()) {
    // Set the junction phrase - it takes priority over street names
    phrase_id = 6;
    // Assign guide sign
    junction_name =
        maneuver.signs().GetJunctionNameString(element_max_count, limit_by_consecutive_count, delim,
                                               maneuver.verbal_formatter(), &markup_formatter_);
  }
  // Set instruction to the determined tagged phrase
  instruction = dictionary_.uturn_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(),
                                              dictionary_.uturn_verbal_subset.relative_directions));
  boost::replace_all(instruction, kJunctionNameTag, junction_name);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }
  return instruction;
}

std::string
NarrativeBuilder::FormVerbalSuccinctMergeTransitionInstruction(Maneuver& maneuver,
                                                               bool limit_by_consecutive_count,
                                                               uint32_t element_max_count,
                                                               const std::string& delim) {
  // "0": "Merge."
  // "1": "Merge <RELATIVE_DIRECTION>."
  // "4": "Merge toward <TOWARD_SIGN>."
  // "5": "Merge <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Use toward phrase if street names is empty
    phrase_id = 4;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  }

  // Check for merge relative direction
  std::string relative_direction;
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeLeft) ||
      (maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeRight)) {
    phrase_id += 1;
    relative_direction =
        FormRelativeTwoDirection(maneuver.type(),
                                 dictionary_.merge_verbal_subset.relative_directions);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.merge_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalSuccinctEnterRoundaboutTransitionInstruction(
    Maneuver& maneuver,
    bool limit_by_consecutive_count,
    uint32_t element_max_count,
    const std::string& delim) {
  // "0": "Enter the roundabout.",
  // "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."
  // "4": "Enter the roundabout and take the <ORDINAL_VALUE> exit toward <TOWARD_SIGN>.",
  // "7": "Enter the roundabout and take the exit toward <TOWARD_SIGN>.",

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string guide_sign;
  std::string ordinal_value;
  if ((maneuver.roundabout_exit_count() >= kRoundaboutExitCountLowerBound) &&
      (maneuver.roundabout_exit_count() <= kRoundaboutExitCountUpperBound)) {
    // Increment for ordinal phrase
    phrase_id += 1;
    // Set ordinal_value
    ordinal_value = dictionary_.enter_roundabout_verbal_subset.ordinal_values.at(
        maneuver.roundabout_exit_count() - 1);
  }
  if (maneuver.roundabout_exit_signs().HasGuide()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id += 3;
    // Assign guide sign
    guide_sign =
        maneuver.roundabout_exit_signs().GetGuideString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter(),
                                                        &markup_formatter_);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_roundabout_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kOrdinalValueTag, ordinal_value);
  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalSuccinctExitRoundaboutTransitionInstruction(
    Maneuver& maneuver,
    bool limit_by_consecutive_count,
    uint32_t element_max_count,
    const std::string& delim) {
  // "0": "Exit the roundabout."
  // "3": "Exit the roundabout toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string guide_sign;

  if (maneuver.HasGuideSign()) {
    // Skip to the toward phrase - it takes priority over street names
    phrase_id = 3;
    // Assign guide sign
    guide_sign = maneuver.signs().GetGuideString(element_max_count, limit_by_consecutive_count, delim,
                                                 maneuver.verbal_formatter(), &markup_formatter_);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_roundabout_verbal_subset.phrases.at(std::to_string(phrase_id));

  boost::replace_all(instruction, kTowardSignTag, guide_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormElevatorInstruction(Maneuver& maneuver) {
  // "0": "Take the elevator.",
  // "1": "Take the elevator to <LEVEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string end_level;

  if (!maneuver.end_level_ref().empty()) {
    phrase_id += 1;
    end_level = maneuver.end_level_ref();
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.elevator_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kLevelTag, end_level);

  return instruction;
}

std::string NarrativeBuilder::FormStepsInstruction(Maneuver& maneuver) {
  // "0": "Take the stairs.",
  // "1": "Take the stairs to <LEVEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string end_level;

  if (!maneuver.end_level_ref().empty()) {
    phrase_id += 1;
    end_level = maneuver.end_level_ref();
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.steps_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kLevelTag, end_level);

  return instruction;
}

std::string NarrativeBuilder::FormEscalatorInstruction(Maneuver& maneuver) {
  // "0": "Take the escalator.",
  // "1": "Take the escalator to <LEVEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string end_level;

  if (!maneuver.end_level_ref().empty()) {
    phrase_id += 1;
    end_level = maneuver.end_level_ref();
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.escalator_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kLevelTag, end_level);

  return instruction;
}

std::string NarrativeBuilder::FormEnterBuildingInstruction(Maneuver& maneuver) {
  // "0": "Enter the building.",
  // "1": "Enter the building, and continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.enter_building_subset.empty_street_name_labels, true);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id += 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_building_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  return instruction;
}

std::string NarrativeBuilder::FormExitBuildingInstruction(Maneuver& maneuver) {
  // "0": "Exit the building.",
  // "1": "Exit the building, and continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.exit_building_subset.empty_street_name_labels, true);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id += 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_building_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  return instruction;
}

std::string NarrativeBuilder::FormTransitPlatformCountLabel(
    size_t stop_count,
    const std::unordered_map<std::string, std::string>& transit_stop_count_labels) {
  const auto plural_category = GetPluralCategory(stop_count);
  const auto item = transit_stop_count_labels.find(plural_category);
  if (item != transit_stop_count_labels.end()) {
    return item->second;
  }
  // Return "other" label by default
  return transit_stop_count_labels.at(kPluralCategoryOtherKey);
}

std::string NarrativeBuilder::FormPassInstruction(Maneuver& maneuver) {
  // "0": "Pass <object>.",
  // "1": "Pass traffic lights on <object>.",
  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string object_label;
  auto dictionary_object_index = kStreetIntersectionIndex; // kGateIndex;
  if (maneuver.has_node_type()) {
    switch (maneuver.node_type()) {
      case TripLeg_Node_Type_kGate:
        dictionary_object_index = kGateIndex;
        break;
      case TripLeg_Node_Type_kBollard:
        dictionary_object_index = kBollardIndex;
        break;
      case TripLeg_Node_Type_kStreetIntersection:
        dictionary_object_index = kStreetIntersectionIndex;
        if (maneuver.traffic_signal())
          phrase_id = 1;
        if (maneuver.HasCrossStreetNames())
          object_label = FormStreetNames(maneuver, maneuver.cross_street_names());
        break;
      default:
        break;
    }
    if (object_label.empty())
      object_label = dictionary_.pass_subset.object_labels.at(dictionary_object_index);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.pass_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kObjectLabelTag, object_label);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::GetPluralCategory(size_t count) {
  if (count == 1) {
    return kPluralCategoryOneKey;
  }
  return kPluralCategoryOtherKey;
}

std::string NarrativeBuilder::FormLength(Maneuver& maneuver,
                                         const std::vector<std::string>& metric_lengths,
                                         const std::vector<std::string>& us_customary_lengths) {
  switch (options_.units()) {
    case Options::miles: {
      return FormUsCustomaryLength((maneuver.has_combined_enter_exit_roundabout()
                                        ? maneuver.roundabout_exit_length(Options::miles)
                                        : maneuver.length(Options::miles)),
                                   us_customary_lengths);
    }
    default: {
      return FormMetricLength((maneuver.has_combined_enter_exit_roundabout()
                                   ? maneuver.roundabout_exit_length(Options::kilometers)
                                   : maneuver.length(Options::kilometers)),
                              metric_lengths);
    }
  }
}

std::string NarrativeBuilder::FormLength(float distance,
                                         const std::vector<std::string>& metric_lengths,
                                         const std::vector<std::string>& us_customary_lengths) {
  switch (options_.units()) {
    case Options::miles: {
      return FormUsCustomaryLength(distance, us_customary_lengths);
    }
    default: {
      return FormMetricLength(distance, metric_lengths);
    }
  }
}

std::string NarrativeBuilder::FormMetricLength(float kilometers,
                                               const std::vector<std::string>& metric_lengths) {

  // 0 "<KILOMETERS> kilometers"
  // 1 "1 kilometer"
  // 2 "<METERS> meters" (10-900 meters)
  // 3 "less than 10 meters"

  std::string length_string;
  length_string.reserve(kLengthStringInitialCapacity);

  // Follow locale rules turning numbers into strings
  std::stringstream distance;
  distance.imbue(dictionary_.GetLocale());

  float meters = std::round(kilometers * midgard::kMetersPerKm);
  float rounded = 0.f;

  // These will determine what we say
  // For distances that will round to 1km or greater
  if (meters > 949) {
    if (kilometers > 3) {
      // Round to integer for distances greater than 3km
      rounded = std::round(kilometers);
    } else {
      // Round to whole or half km for 1km to 3km distances
      rounded = std::round(kilometers * 2) / 2;
    }

    if (rounded == 1.f) {
      // "1 kilometer"
      length_string += metric_lengths.at(kOneKilometerIndex);
    } else {
      // "<KILOMETERS> kilometers"
      length_string += metric_lengths.at(kKilometersIndex);
      // 1 digit of precision for float and 0 for int
      distance << std::setiosflags(std::ios::fixed)
               << std::setprecision(rounded != static_cast<int>(rounded)) << rounded;
    }
  } else {
    if (meters > 94) {
      // "<METERS> meters" (100-900 meters)
      length_string += metric_lengths.at(kMetersIndex);
      distance << (std::round(meters / 100) * 100);
    } else if (meters > 9) {
      // "<METERS> meters" (10-90 meters)
      length_string += metric_lengths.at(kMetersIndex);
      distance << (std::round(meters / 10) * 10);
    } else {
      // "less than 10 meters"
      length_string += metric_lengths.at(kSmallMetersIndex);
    }
  }

  // TODO: why do we need separate tags for kilometers and meters?
  // Replace tags with length values
  boost::replace_all(length_string, kKilometersTag, distance.str());
  boost::replace_all(length_string, kMetersTag, distance.str());

  return length_string;
}

std::string
NarrativeBuilder::FormUsCustomaryLength(float miles,
                                        const std::vector<std::string>& us_customary_lengths) {

  // 0  "<MILES> miles"
  // 1  "1 mile"
  // 2  "a half mile"
  // 3  "a quarter mile"
  // 4  "<FEET> feet" (10-90, 100-500)
  // 5  "less than 10 feet"

  std::string length_string;
  length_string.reserve(kLengthStringInitialCapacity);

  // Follow locale rules turning numbers into strings
  std::stringstream distance;
  distance.imbue(dictionary_.GetLocale());
  float feet = std::round(miles * midgard::kFeetPerMile);
  float rounded = 0.f;

  // These will determine what we say
  if (feet > 1000) {
    if (miles > 2) {
      rounded = std::round(miles);
    } else if (miles >= 0.625) {
      rounded = std::round(miles * 2) / 2;
    } else {
      rounded = std::round(miles * 4) / 4;
    }

    if (rounded == 0.25f) {
      // "a quarter mile"
      length_string += us_customary_lengths.at(kQuarterMileIndex);
    } else if (rounded == 0.5f) {
      // "a half mile"
      length_string += us_customary_lengths.at(kHalfMileIndex);
    } else if (rounded == 1.f) {
      // "1 mile"
      length_string += us_customary_lengths.at(kOneMileIndex);
    } else {
      // "<MILES> miles"
      length_string += us_customary_lengths.at(kMilesIndex);
      distance << std::setiosflags(std::ios::fixed) << std::setprecision(rounded == 1.5f) << rounded;
    }
  } else {
    if (feet > 94) {
      // "<FEET> feet" (100-1000)
      length_string += us_customary_lengths.at(kFeetIndex);
      distance << (std::round(feet / 100) * 100);
    } else if (feet > 9) {
      // "<FEET> feet" (10-90)
      length_string += us_customary_lengths.at(kFeetIndex);
      distance << (std::round(feet / 10) * 10);
    } else {
      // "less than 10 feet"
      length_string += us_customary_lengths.at(kSmallFeetIndex);
    }
  }

  // TODO: why do we need separate tags for miles, tenths and feet?
  // Replace tags with length values
  boost::replace_all(length_string, kMilesTag, distance.str());
  boost::replace_all(length_string, kTenthsOfMilesTag, distance.str());
  boost::replace_all(length_string, kFeetTag, distance.str());

  return length_string;
}

std::string
NarrativeBuilder::FormRelativeTwoDirection(DirectionsLeg_Maneuver_Type type,
                                           const std::vector<std::string>& relative_directions) {
  switch (type) {
    case DirectionsLeg_Maneuver_Type_kLeft:
    case DirectionsLeg_Maneuver_Type_kSharpLeft:
    case DirectionsLeg_Maneuver_Type_kSlightLeft:
    case DirectionsLeg_Maneuver_Type_kUturnLeft:
    case DirectionsLeg_Maneuver_Type_kRampLeft:
    case DirectionsLeg_Maneuver_Type_kExitLeft:
    case DirectionsLeg_Maneuver_Type_kMergeLeft:
    case DirectionsLeg_Maneuver_Type_kDestinationLeft: {
      return relative_directions.at(0); // "left"
    }
    case DirectionsLeg_Maneuver_Type_kRight:
    case DirectionsLeg_Maneuver_Type_kSharpRight:
    case DirectionsLeg_Maneuver_Type_kSlightRight:
    case DirectionsLeg_Maneuver_Type_kUturnRight:
    case DirectionsLeg_Maneuver_Type_kRampRight:
    case DirectionsLeg_Maneuver_Type_kExitRight:
    case DirectionsLeg_Maneuver_Type_kMergeRight:
    case DirectionsLeg_Maneuver_Type_kDestinationRight: {
      return relative_directions.at(1); // "right"
    }
    default: {
      throw valhalla_exception_t{231};
    }
  }
}

std::string
NarrativeBuilder::FormRelativeThreeDirection(DirectionsLeg_Maneuver_Type type,
                                             const std::vector<std::string>& relative_directions) {
  switch (type) {
    case DirectionsLeg_Maneuver_Type_kStayLeft: {
      return relative_directions.at(0); // "left"
    }
    case DirectionsLeg_Maneuver_Type_kStayStraight: {
      return relative_directions.at(1); // "straight"
    }
    case DirectionsLeg_Maneuver_Type_kStayRight: {
      return relative_directions.at(2); // "right"
    }
    default: {
      throw valhalla_exception_t{232};
    }
  }
}

std::string
NarrativeBuilder::FormTransitName(const Maneuver& maneuver,
                                  const std::vector<std::string>& empty_transit_name_labels) {
  if (!maneuver.transit_info().short_name.empty()) {
    return maneuver.transit_info().short_name;
  } else if (!maneuver.transit_info().long_name.empty()) {
    return (maneuver.transit_info().long_name);
  }
  return empty_transit_name_labels.at(maneuver.transit_type());
}

// NOTE: Tried to use 'contains' instead of 'ends_with'
//       however, the results were not good when name had the label in the middle.
//       Should implement per language.
bool NarrativeBuilder::HasLabel(const std::string& name, const std::string& label) {
  return boost::algorithm::iends_with(name, label);
}

std::string
NarrativeBuilder::FormStreetNames(const Maneuver& maneuver,
                                  const StreetNames& street_names,
                                  const std::vector<std::string>* empty_street_name_labels,
                                  bool enhance_empty_street_names,
                                  uint32_t max_count,
                                  const std::string& delim,
                                  const VerbalTextFormatter* verbal_formatter) {
  std::string street_names_string;

  // Verify that the street name list is not empty
  if (!street_names.empty()) {
    street_names_string = FormStreetNames(street_names, max_count, delim, verbal_formatter);
  }

  // If empty street names string
  // then determine if walkway or bike path
  if (enhance_empty_street_names && street_names_string.empty() && empty_street_name_labels) {
    // Set names in blind user mode:
    if (maneuver.pedestrian_type() == PedestrianType::kBlind) {
      if (maneuver.is_steps())
        street_names_string = empty_street_name_labels->at(kStepsIndex);
      else if (maneuver.is_bridge())
        street_names_string = empty_street_name_labels->at(kBridgeIndex);
      else if (maneuver.is_tunnel())
        street_names_string = empty_street_name_labels->at(kTunnelIndex);
    }
    // If pedestrian travel mode on unnamed footway
    // then set street names string to walkway. Additionally, if the path
    // is a pedestrian crossing, use appropriate phrasing.
    else if ((maneuver.travel_mode() == TravelMode::kPedestrian) && maneuver.unnamed_walkway()) {
      auto dictionary_index =
          maneuver.pedestrian_crossing() ? kPedestrianCrossingIndex : kWalkwayIndex;
      street_names_string = empty_street_name_labels->at(dictionary_index);
    }

    // If bicycle travel mode on unnamed cycleway
    // then set street names string to cycleway
    else if ((maneuver.travel_mode() == TravelMode::kBicycle) && maneuver.unnamed_cycleway()) {
      street_names_string = empty_street_name_labels->at(kCyclewayIndex);
    }

    // If bicycle travel mode on unnamed mountain bike trail
    // then set street names string to mountain bike trail
    else if ((maneuver.travel_mode() == TravelMode::kBicycle) &&
             maneuver.unnamed_mountain_bike_trail()) {
      street_names_string = empty_street_name_labels->at(kMountainBikeTrailIndex);
    }
  }

  return street_names_string;
}

std::string NarrativeBuilder::FormStreetNames(const StreetNames& street_names,
                                              uint32_t max_count,
                                              const std::string& delim,
                                              const VerbalTextFormatter* verbal_formatter) {
  std::string street_names_string;
  uint32_t count = 0;

  for (const auto& street_name : street_names) {
    // If supplied, limit by max count
    if ((max_count > 0) && (count == max_count)) {
      break;
    }
    // If the street_names_string is not empty then add the delimiter
    if (!street_names_string.empty()) {
      street_names_string += delim;
    }

    // Append next name to string
    street_names_string += (verbal_formatter)
                               ? verbal_formatter->Format(street_name, &markup_formatter_)
                               : street_name->value();
    ++count;
  }

  return street_names_string;
}

void NarrativeBuilder::FormVerbalMultiCue(std::list<Maneuver>& maneuvers) {
  Maneuver* prev_maneuver = nullptr;
  for (auto& maneuver : maneuvers) {
    if (maneuver.pedestrian_type() == PedestrianType::kBlind)
      continue;
    if (prev_maneuver && IsVerbalMultiCuePossible(*prev_maneuver, maneuver)) {
      // Determine if imminent or distant verbal multi-cue
      // if previous maneuver has an intersecting traversable outbound edge
      // in the same direction as the maneuver
      switch (maneuver.type()) {
        case DirectionsLeg_Maneuver_Type_kSlightRight:
        case DirectionsLeg_Maneuver_Type_kRight:
        case DirectionsLeg_Maneuver_Type_kSharpRight:
        case DirectionsLeg_Maneuver_Type_kUturnRight:
        case DirectionsLeg_Maneuver_Type_kRampRight:
        case DirectionsLeg_Maneuver_Type_kExitRight:
        case DirectionsLeg_Maneuver_Type_kStayRight: {
          if (prev_maneuver->has_right_traversable_outbound_intersecting_edge()) {
            prev_maneuver->set_distant_verbal_multi_cue(true);
          } else {
            prev_maneuver->set_imminent_verbal_multi_cue(true);
          }
          break;
        }
        case DirectionsLeg_Maneuver_Type_kSlightLeft:
        case DirectionsLeg_Maneuver_Type_kLeft:
        case DirectionsLeg_Maneuver_Type_kSharpLeft:
        case DirectionsLeg_Maneuver_Type_kUturnLeft:
        case DirectionsLeg_Maneuver_Type_kRampLeft:
        case DirectionsLeg_Maneuver_Type_kExitLeft:
        case DirectionsLeg_Maneuver_Type_kStayLeft: {
          if (prev_maneuver->has_left_traversable_outbound_intersecting_edge()) {
            prev_maneuver->set_distant_verbal_multi_cue(true);
          } else {
            prev_maneuver->set_imminent_verbal_multi_cue(true);
          }
          break;
        }
        case DirectionsLeg_Maneuver_Type_kDestination:
        case DirectionsLeg_Maneuver_Type_kDestinationLeft:
        case DirectionsLeg_Maneuver_Type_kDestinationRight: {
          if (prev_maneuver->has_left_traversable_outbound_intersecting_edge() ||
              prev_maneuver->has_right_traversable_outbound_intersecting_edge()) {
            prev_maneuver->set_distant_verbal_multi_cue(true);
          } else {
            prev_maneuver->set_imminent_verbal_multi_cue(true);
          }
          break;
        }
        default: {
          prev_maneuver->set_imminent_verbal_multi_cue(true);
          break;
        }
      }

      // Set verbal succinct transition instruction as a verbal multi-cue
      if (prev_maneuver->HasVerbalSuccinctTransitionInstruction()) {
        prev_maneuver->set_verbal_succinct_transition_instruction(
            FormVerbalMultiCue(*prev_maneuver, maneuver, true));
      }

      // Set verbal pre transition instruction as a verbal multi-cue
      prev_maneuver->set_verbal_pre_transition_instruction(
          FormVerbalMultiCue(*prev_maneuver, maneuver));
    }

    // Update previous maneuver
    prev_maneuver = &maneuver;
  }
}

std::string NarrativeBuilder::FormVerbalMultiCue(Maneuver& maneuver,
                                                 Maneuver& next_maneuver,
                                                 bool process_succinct) {
  // Set current verbal cue
  const std::string& current_verbal_cue =
      ((process_succinct && maneuver.HasVerbalSuccinctTransitionInstruction())
           ? maneuver.verbal_succinct_transition_instruction()
           : maneuver.verbal_pre_transition_instruction());

  // Set next verbal cue
  std::string next_verbal_cue = next_maneuver.HasVerbalTransitionAlertInstruction()
                                    ? next_maneuver.verbal_transition_alert_instruction()
                                    : next_maneuver.verbal_pre_transition_instruction();

  return FormVerbalMultiCue(maneuver, current_verbal_cue, next_verbal_cue);
}

std::string NarrativeBuilder::FormVerbalMultiCue(Maneuver& maneuver,
                                                 const std::string& first_verbal_cue,
                                                 const std::string& second_verbal_cue) {
  // "0": "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>"
  // "1": "<CURRENT_VERBAL_CUE> Then, in <LENGTH>, <NEXT_VERBAL_CUE>"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the proper verbal multi-cue
  uint8_t phrase_id = 0;
  if (maneuver.distant_verbal_multi_cue()) {
    phrase_id = 1;
  }
  instruction = dictionary_.verbal_multi_cue_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kCurrentVerbalCueTag, first_verbal_cue);
  boost::replace_all(instruction, kNextVerbalCueTag, second_verbal_cue);
  boost::replace_all(instruction, kLengthTag,
                     FormLength(maneuver, dictionary_.post_transition_verbal_subset.metric_lengths,
                                dictionary_.post_transition_verbal_subset.us_customary_lengths));

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

bool NarrativeBuilder::IsVerbalMultiCuePossible(Maneuver& maneuver, Maneuver& next_maneuver) {
  // Current maneuver must have a verbal pre-transition instruction
  // Next maneuver must have a verbal transition alert or a verbal pre-transition instruction
  // Current maneuver must be within verbal multi-cue bounds
  // Next maneuver must not be a merge
  // Current is not a roundabout OR current maneuver has combined enter/exit roundabout instruction
  // Next maneuver must not be a roundabout
  // Current and next maneuvers must not be transit or transit connection
  if (maneuver.HasVerbalPreTransitionInstruction() &&
      (next_maneuver.HasVerbalTransitionAlertInstruction() ||
       next_maneuver.HasVerbalPreTransitionInstruction()) &&
      IsWithinVerbalMultiCueBounds(maneuver) && !next_maneuver.IsMergeType() &&
      (!maneuver.roundabout() || maneuver.has_combined_enter_exit_roundabout()) &&
      !((maneuver.type() == DirectionsLeg_Maneuver_Type_kRoundaboutExit) &&
        next_maneuver.roundabout()) &&
      !maneuver.IsTransit() && !next_maneuver.IsTransit() && !maneuver.transit_connection() &&
      !next_maneuver.transit_connection()) {
    return true;
  }
  return false;
}

bool NarrativeBuilder::IsWithinVerbalMultiCueBounds(Maneuver& maneuver) {
  if (maneuver.IsStartType()) {
    return (maneuver.basic_time() < kVerbalMultiCueTimeStartManeuverThreshold);
  }
  // Maneuver must be quick (basic time < 13 sec)
  return (maneuver.basic_time() < kVerbalMultiCueTimeThreshold);
}

void NarrativeBuilder::UpdateObviousManeuverStreetNames(Maneuver& maneuver,
                                                        std::string& begin_street_names,
                                                        std::string& street_names) {
  if (maneuver.contains_obvious_maneuver() && !begin_street_names.empty()) {
    street_names = begin_street_names;
    begin_street_names.clear();
  }
}

///////////////////////////////////////////////////////////////////////////////
std::string NarrativeBuilder_csCZ::GetPluralCategory(size_t count) {
  if (count == 1) {
    return kPluralCategoryOneKey;
  } else if ((count > 1) && (count < 5)) {
    return kPluralCategoryFewKey;
  }
  return kPluralCategoryOtherKey;
}

///////////////////////////////////////////////////////////////////////////////
std::string NarrativeBuilder_hiIN::GetPluralCategory(size_t /*count*/) {
  return kPluralCategoryOtherKey;
}

///////////////////////////////////////////////////////////////////////////////
const std::unordered_map<std::string, std::string> NarrativeBuilder_itIT::articulated_prepositions_ =
    {{" su il ", " sul "}, {" su la ", " sulla "}};

void NarrativeBuilder_itIT::FormArticulatedPrepositions(std::string& instruction) {
  for (const auto& item : NarrativeBuilder_itIT::articulated_prepositions_) {
    boost::replace_all(instruction, item.first, item.second);
  }
}

///////////////////////////////////////////////////////////////////////////////
std::string NarrativeBuilder_ruRU::GetPluralCategory(size_t count) {
  size_t rem10 = count % 10, rem100 = count % 100;

  // http://www.unicode.org/cldr/charts/29/supplemental/language_plural_rules.html#ru
  if (rem10 == 1 && rem100 != 11) {
    return kPluralCategoryOneKey;
  } else if ((rem10 > 1 && rem10 < 5) && !(rem100 > 11 && rem100 < 15)) {
    return kPluralCategoryFewKey;
  }
  return kPluralCategoryOtherKey;
}

std::string NarrativeBuilder::FormBssManeuverType(DirectionsLeg_Maneuver_BssManeuverType type) {
  switch (type) {
    case DirectionsLeg_Maneuver_BssManeuverType_kRentBikeAtBikeShare: {
      return "Then rent a bike at BSS. ";
    }
    case DirectionsLeg_Maneuver_BssManeuverType_kReturnBikeAtBikeShare: {
      return "Then return the bike to BSS. ";
    }
    default:
      return "";
  }
}
} // namespace odin
} // namespace valhalla
