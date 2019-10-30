#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>

#include "baldr/verbal_text_formatter.h"

#include "odin/enhancedtrippath.h"
#include "odin/maneuver.h"
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
constexpr auto kVerbalMultiCueTimeThreshold = 10;

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
                                   const NarrativeDictionary& dictionary)
    : options_(options), trip_path_(trip_path), dictionary_(dictionary),
      articulated_preposition_enabled_(false) {
}

void NarrativeBuilder::Build(const Options& options,
                             const EnhancedTripLeg* etp,
                             std::list<Maneuver>& maneuvers) {
  Maneuver* prev_maneuver = nullptr;
  for (auto& maneuver : maneuvers) {
    switch (maneuver.type()) {
      case DirectionsLeg_Maneuver_Type_kStartRight:
      case DirectionsLeg_Maneuver_Type_kStart:
      case DirectionsLeg_Maneuver_Type_kStartLeft: {
        // Set instruction
        maneuver.set_instruction(FormStartInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalStartInstruction(maneuver));

        // Set verbal post transition instruction only if there are
        // begin street names
        if (maneuver.HasBeginStreetNames()) {
          maneuver.set_verbal_post_transition_instruction(
              FormVerbalPostTransitionInstruction(maneuver, maneuver.HasBeginStreetNames()));
        }
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
        // Set instruction
        maneuver.set_instruction(FormBecomesInstruction(maneuver, prev_maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalBecomesInstruction(maneuver, prev_maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver, maneuver.HasBeginStreetNames()));
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
        if (maneuver.length() > kVerbalPostMinimumRampLength) {
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
        if (maneuver.length() > kVerbalPostMinimumRampLength) {
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
        if (maneuver.length() > kVerbalPostMinimumRampLength) {
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

          // Only set verbal post if > min ramp length
          if (maneuver.length() > kVerbalPostMinimumRampLength) {
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

          // Only set verbal post if > min ramp length
          if (maneuver.length() > kVerbalPostMinimumRampLength) {
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

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            FormVerbalAlertEnterRoundaboutInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalEnterRoundaboutInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kRoundaboutExit: {
        // Set instruction
        maneuver.set_instruction(FormExitRoundaboutInstruction(maneuver));

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
      case DirectionsLeg_Maneuver_Type_kFerryExit: {
        // Set instruction
        maneuver.set_instruction(FormExitFerryInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            FormVerbalAlertExitFerryInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(FormVerbalExitFerryInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver, maneuver.HasBeginStreetNames()));
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
      case DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination: {
        // Set instruction
        maneuver.set_instruction(FormPostTransitConnectionDestinationInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalPostTransitConnectionDestinationInstruction(maneuver));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            FormVerbalPostTransitionInstruction(maneuver));
        break;
      }
      case DirectionsLeg_Maneuver_Type_kContinue:
      default: {
        // Set instruction
        maneuver.set_instruction(FormContinueInstruction(maneuver));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            FormVerbalAlertContinueInstruction(maneuver));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            FormVerbalContinueInstruction(maneuver, options.units()));
        // NOTE: No verbal post transition instruction
        break;
      }
    }

    // Update previous maneuver
    prev_maneuver = &maneuver;
  }

  // Iterate over maneuvers to form verbal multi-cue instructions
  FormVerbalMultiCue(maneuvers);
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

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id += 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id += 1;
  }
  if (maneuver.travel_mode() == TripLeg_TravelMode_kDrive) {
    phrase_id += 4;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kPedestrian) {
    phrase_id += 8;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) {
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
  // "0": "Head <CARDINAL_DIRECTION> for <LENGTH>.",
  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>.",
  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",

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

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id += 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id += 1;
  }
  if (maneuver.travel_mode() == TripLeg_TravelMode_kDrive) {
    phrase_id += 4;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kPedestrian) {
    phrase_id += 8;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) {
    phrase_id += 16;
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
  if (dest.has_name() && !(dest.name().empty())) {
    phrase_id += 1;
    destination = dest.name();
  }
  // Check for destination street
  else if (dest.has_street() && !(dest.street().empty())) {
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
  if (dest.has_name() && !(dest.name().empty())) {
    phrase_id += 1;
    destination = dest.name();
  }
  // Check for destination street
  else if (dest.has_street() && !(dest.street().empty())) {
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
  if (dest.has_name() && !(dest.name().empty())) {
    phrase_id += 1;
    destination = dest.name();
  }
  // Check for destination street
  else if (dest.has_street() && !(dest.street().empty())) {
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

std::string NarrativeBuilder::FormContinueInstruction(Maneuver& maneuver) {
  // "0": "Continue.",
  // "1": "Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.continue_subset.empty_street_name_labels, true);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.continue_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertContinueInstruction(Maneuver& maneuver,
                                                                 uint32_t element_max_count,
                                                                 const std::string& delim) {
  // "0": "Continue.",
  // "1": "Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.continue_verbal_alert_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.continue_verbal_alert_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalContinueInstruction(Maneuver& maneuver,
                                                            Options_Units units,
                                                            uint32_t element_max_count,
                                                            const std::string& delim) {
  // "0": "Continue for <LENGTH>.",
  // "1": "Continue on <STREET_NAMES> for <LENGTH>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.continue_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.continue_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kLengthTag,
                     FormLength(maneuver, dictionary_.continue_verbal_subset.metric_lengths,
                                dictionary_.continue_verbal_subset.us_customary_lengths));
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormTurnInstruction(Maneuver& maneuver) {
  // "0": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION>.",
  // "1": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  // "2": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on
  // <STREET_NAMES>.",
  // "3": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
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

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  }
  if (maneuver.to_stay_on()) {
    phrase_id = 3;
  }

  // Set instruction to the determined tagged phrase
  instruction = subset->phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(), subset->relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertTurnInstruction(Maneuver& maneuver,
                                                             uint32_t element_max_count,
                                                             const std::string& delim) {
  // "0": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION>.",
  // "1": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  // "2": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.",
  // "3": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."

  return FormVerbalTurnInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalTurnInstruction(Maneuver& maneuver,
                                                        uint32_t element_max_count,
                                                        const std::string& delim) {
  // "0": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION>.",
  // "1": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  // "2": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.",
  // "3": "Turn/Bear/Turn sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."

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

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  }
  if (maneuver.to_stay_on()) {
    phrase_id = 3;
  }

  // Set instruction to the determined tagged phrase
  instruction = subset->phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(), subset->relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormUturnInstruction(Maneuver& maneuver) {
  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.",
  // "4": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.",
  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."

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
  if (!street_names.empty()) {
    phrase_id += 1;
    if (maneuver.to_stay_on()) {
      phrase_id += 1;
    }
  }
  if (!cross_street_names.empty()) {
    phrase_id += 3;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.uturn_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeTwoDirection(maneuver.type(),
                                              dictionary_.uturn_subset.relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kCrossStreetNamesTag, cross_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertUturnInstruction(Maneuver& maneuver,
                                                              uint32_t element_max_count,
                                                              const std::string& delim) {
  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>."

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
  if (!street_names.empty()) {
    phrase_id = 1;
    if (maneuver.to_stay_on()) {
      phrase_id = 2;
    }
  }
  if (!cross_street_names.empty()) {
    phrase_id = 3;
  }

  return FormVerbalUturnInstruction(phrase_id,
                                    FormRelativeTwoDirection(maneuver.type(),
                                                             dictionary_.uturn_verbal_subset
                                                                 .relative_directions),
                                    street_names, cross_street_names);
}

std::string NarrativeBuilder::FormVerbalUturnInstruction(Maneuver& maneuver,
                                                         uint32_t element_max_count,
                                                         const std::string& delim) {
  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.",
  // "4": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.",
  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."

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
  if (!street_names.empty()) {
    phrase_id += 1;
    if (maneuver.to_stay_on()) {
      phrase_id += 1;
    }
  }
  if (!cross_street_names.empty()) {
    phrase_id += 3;
  }

  return FormVerbalUturnInstruction(phrase_id,
                                    FormRelativeTwoDirection(maneuver.type(),
                                                             dictionary_.uturn_verbal_subset
                                                                 .relative_directions),
                                    street_names, cross_street_names);
}

std::string NarrativeBuilder::FormVerbalUturnInstruction(uint8_t phrase_id,
                                                         const std::string& relative_dir,
                                                         const std::string& street_names,
                                                         const std::string& cross_street_names) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.uturn_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kCrossStreetNamesTag, cross_street_names);

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
                                             maneuver.verbal_formatter());
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id = 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  } else if (maneuver.HasExitNameSign()) {
    phrase_id = 4;
    // Assign name sign
    exit_name_sign = maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
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
                                             maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign() && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign = maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
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
                                             maneuver.verbal_formatter());
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  } else if (maneuver.HasExitNameSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign = maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
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
                                             maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign() && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign name sign
    exit_name_sign = maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
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
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  } else if (maneuver.HasExitNameSign()) {
    phrase_id += 8;
    // Assign name sign
    exit_name_sign = maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
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
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter());
  }
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    // Assign branch sign
    exit_branch_sign =
        maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitNumberSign()) {
    phrase_id += 8;
    // Assign name sign
    exit_name_sign = maneuver.signs().GetExitNameString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
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

  // Assign the street names
  std::string street_names;

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

  // Determine which phrase to use
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (!street_names.empty()) {
    phrase_id += 2;
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeThreeDirection(maneuver.type(),
                                                dictionary_.keep_subset.relative_directions));
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);

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

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.keep_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // If street names string is empty and the maneuver has sign branch info
  // then assign the sign branch name to the street names string
  if (street_names.empty() && maneuver.HasExitBranchSign()) {
    street_names = maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
  }

  // Determine which phrase to use
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter());
  } else if (!street_names.empty()) {
    phrase_id += 2;
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  }

  return FormVerbalKeepInstruction(phrase_id,
                                   FormRelativeThreeDirection(maneuver.type(),
                                                              dictionary_.keep_verbal_subset
                                                                  .relative_directions),
                                   street_names, exit_number_sign, exit_toward_sign);
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

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.keep_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // If street names string is empty and the maneuver has sign branch info
  // then assign the sign branch name to the street names string
  if (street_names.empty() && maneuver.HasExitBranchSign()) {
    street_names = maneuver.signs().GetExitBranchString(element_max_count, limit_by_consecutive_count,
                                                        delim, maneuver.verbal_formatter());
  }

  // Determine which phrase to use
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter());
  }
  if (!street_names.empty()) {
    phrase_id += 2;
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  }

  return FormVerbalKeepInstruction(phrase_id,
                                   FormRelativeThreeDirection(maneuver.type(),
                                                              dictionary_.keep_verbal_subset
                                                                  .relative_directions),
                                   street_names, exit_number_sign, exit_toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepInstruction(uint8_t phrase_id,
                                                        const std::string& relative_dir,
                                                        const std::string& street_names,
                                                        const std::string& exit_number_sign,
                                                        const std::string& exit_toward_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);

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
  // <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.keep_to_stay_on_subset.empty_street_name_labels, true,
                      element_max_count);

  // Determine which phrase to use
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_to_stay_on_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag,
                     FormRelativeThreeDirection(maneuver.type(), dictionary_.keep_to_stay_on_subset
                                                                     .relative_directions));
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertKeepToStayOnInstruction(Maneuver& maneuver,
                                                                     bool limit_by_consecutive_count,
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

  // Determine which phrase to use
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    // Assign number sign
    exit_number_sign =
        maneuver.signs().GetExitNumberString(0, false, delim, maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    // Assign toward sign
    exit_toward_sign =
        maneuver.signs().GetExitTowardString(element_max_count, limit_by_consecutive_count, delim,
                                             maneuver.verbal_formatter());
  }

  return FormVerbalKeepToStayOnInstruction(
      phrase_id,
      FormRelativeThreeDirection(maneuver.type(),
                                 dictionary_.keep_to_stay_on_verbal_subset.relative_directions),
      street_names, exit_number_sign, exit_toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepToStayOnInstruction(uint8_t phrase_id,
                                                                const std::string& relative_dir,
                                                                const std::string& street_names,
                                                                const std::string& exit_number_sign,
                                                                const std::string& exit_toward_sign) {

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.keep_to_stay_on_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_dir);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kNumberSignTag, exit_number_sign);
  boost::replace_all(instruction, kTowardSignTag, exit_toward_sign);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormMergeInstruction(Maneuver& maneuver) {
  // "0", "Merge.",
  // "1", "Merge <RELATIVE_DIRECTION>.",
  // "2", "Merge onto <STREET_NAMES>.",
  // "3", "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;

  // Check for merge relative direction
  std::string relative_direction;
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeLeft) ||
      (maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeRight)) {
    phrase_id += 1;
    relative_direction =
        FormRelativeTwoDirection(maneuver.type(), dictionary_.merge_subset.relative_directions);
  }

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.merge_subset.empty_street_name_labels, true);
  if (!street_names.empty()) {
    phrase_id += 2;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.merge_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertMergeInstruction(Maneuver& maneuver,
                                                              uint32_t element_max_count,
                                                              const std::string& delim) {
  // "0", "Merge.",
  // "1", "Merge <RELATIVE_DIRECTION>.",
  // "2", "Merge onto <STREET_NAMES>.",
  // "3", "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."

  return FormVerbalMergeInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalMergeInstruction(Maneuver& maneuver,
                                                         uint32_t element_max_count,
                                                         const std::string& delim) {
  // "0", "Merge.",
  // "1", "Merge <RELATIVE_DIRECTION>.",
  // "2", "Merge onto <STREET_NAMES>.",
  // "3", "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;

  // Check for merge relative direction
  std::string relative_direction;
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeLeft) ||
      (maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeRight)) {
    phrase_id += 1;
    relative_direction =
        FormRelativeTwoDirection(maneuver.type(),
                                 dictionary_.merge_verbal_subset.relative_directions);
  }

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.merge_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());
  if (!street_names.empty()) {
    phrase_id += 2;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.merge_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kRelativeDirectionTag, relative_direction);
  boost::replace_all(instruction, kStreetNamesTag, street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormEnterRoundaboutInstruction(Maneuver& maneuver) {
  // "0": "Enter the roundabout.",
  // "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string ordinal_value;
  if ((maneuver.roundabout_exit_count() >= kRoundaboutExitCountLowerBound) &&
      (maneuver.roundabout_exit_count() <= kRoundaboutExitCountUpperBound)) {
    phrase_id = 1;
    // Set ordinal_value
    ordinal_value =
        dictionary_.enter_roundabout_subset.ordinal_values.at(maneuver.roundabout_exit_count() - 1);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_roundabout_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kOrdinalValueTag, ordinal_value);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertEnterRoundaboutInstruction(Maneuver& maneuver,
                                                                        uint32_t element_max_count,
                                                                        const std::string& delim) {
  // "0": "Enter the roundabout.",
  // "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string ordinal_value;
  if ((maneuver.roundabout_exit_count() >= kRoundaboutExitCountLowerBound) &&
      (maneuver.roundabout_exit_count() <= kRoundaboutExitCountUpperBound)) {
    phrase_id = 1;
    // Set ordinal_value
    ordinal_value = dictionary_.enter_roundabout_verbal_subset.ordinal_values.at(
        maneuver.roundabout_exit_count() - 1);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_roundabout_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kOrdinalValueTag, ordinal_value);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalEnterRoundaboutInstruction(Maneuver& maneuver,
                                                                   uint32_t element_max_count,
                                                                   const std::string& delim) {
  // "0": "Enter the roundabout.",
  // "1": "Enter the roundabout and take the <ORDINAL_VALUE> exit."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  std::string ordinal_value;
  if ((maneuver.roundabout_exit_count() >= kRoundaboutExitCountLowerBound) &&
      (maneuver.roundabout_exit_count() <= kRoundaboutExitCountUpperBound)) {
    phrase_id = 1;
    // Set ordinal_value
    ordinal_value = dictionary_.enter_roundabout_verbal_subset.ordinal_values.at(
        maneuver.roundabout_exit_count() - 1);
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.enter_roundabout_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kOrdinalValueTag, ordinal_value);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormExitRoundaboutInstruction(Maneuver& maneuver) {
  // "0": "Exit the roundabout.",
  // "1": "Exit the roundabout onto <STREET_NAMES>.",
  // "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

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

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_roundabout_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalExitRoundaboutInstruction(Maneuver& maneuver,
                                                                  uint32_t element_max_count,
                                                                  const std::string& delim) {
  // "0": "Exit the roundabout.",
  // "1": "Exit the roundabout onto <STREET_NAMES>.",
  // "2": "Exit the roundabout onto <BEGIN_STREET_NAMES>."

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

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_roundabout_verbal_subset.phrases.at(std::to_string(phrase_id));

  // Replace phrase tags with values
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormEnterFerryInstruction(Maneuver& maneuver) {
  // "0": "Take the Ferry.",
  // "1": "Take the <STREET_NAMES>.",
  // "2": "Take the <STREET_NAMES> <FERRY_LABEL>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.enter_ferry_subset.empty_street_name_labels, true);

  std::string ferry_label = dictionary_.enter_ferry_subset.ferry_label;

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!street_names.empty()) {
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

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertEnterFerryInstruction(Maneuver& maneuver,
                                                                   uint32_t element_max_count,
                                                                   const std::string& delim) {
  // "0": "Take the Ferry.",
  // "1": "Take the <STREET_NAMES>.",
  // "2": "Take the <STREET_NAMES> <FERRY_LABEL>."

  return FormVerbalEnterFerryInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalEnterFerryInstruction(Maneuver& maneuver,
                                                              uint32_t element_max_count,
                                                              const std::string& delim) {
  // "0": "Take the Ferry.",
  // "1": "Take the <STREET_NAMES>.",
  // "2": "Take the <STREET_NAMES> <FERRY_LABEL>."

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
  if (!street_names.empty()) {
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

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

std::string NarrativeBuilder::FormExitFerryInstruction(Maneuver& maneuver) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
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
      dictionary_.exit_ferry_subset.cardinal_directions.at(maneuver.begin_cardinal_direction());

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.exit_ferry_subset.empty_street_name_labels, true);

  // Assign the begin street names
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(),
                      &dictionary_.exit_ferry_subset.empty_street_name_labels);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (maneuver.travel_mode() == TripLeg_TravelMode_kDrive) {
    phrase_id += 4;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kPedestrian) {
    phrase_id += 8;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) {
    phrase_id += 16;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_ferry_subset.phrases.at(std::to_string(phrase_id));

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

std::string NarrativeBuilder::FormVerbalAlertExitFerryInstruction(Maneuver& maneuver,
                                                                  uint32_t element_max_count,
                                                                  const std::string& delim) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."
  // "4": "Drive <CARDINAL_DIRECTION>.",
  // "5": "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "6": "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "8": "Walk <CARDINAL_DIRECTION>.",
  // "9": "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "10": "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "16": "Bike <CARDINAL_DIRECTION>.",
  // "17": "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "18": "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."

  return FormVerbalExitFerryInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalExitFerryInstruction(Maneuver& maneuver,
                                                             uint32_t element_max_count,
                                                             const std::string& delim) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."
  // "4": "Drive <CARDINAL_DIRECTION>.",
  // "5": "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "6": "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "8": "Walk <CARDINAL_DIRECTION>.",
  // "9": "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "10": "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "16": "Bike <CARDINAL_DIRECTION>.",
  // "17": "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "18": "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set cardinal_direction value
  std::string cardinal_direction = dictionary_.exit_ferry_verbal_subset.cardinal_directions.at(
      maneuver.begin_cardinal_direction());

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.exit_ferry_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Assign the begin street names
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(),
                      &dictionary_.exit_ferry_verbal_subset.empty_street_name_labels, false,
                      element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (maneuver.travel_mode() == TripLeg_TravelMode_kDrive) {
    phrase_id += 4;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kPedestrian) {
    phrase_id += 8;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) {
    phrase_id += 16;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.exit_ferry_verbal_subset.phrases.at(std::to_string(phrase_id));

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

std::string NarrativeBuilder::FormPostTransitConnectionDestinationInstruction(Maneuver& maneuver) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
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
      dictionary_.post_transit_connection_destination_subset.cardinal_directions.at(
          maneuver.begin_cardinal_direction());

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             &dictionary_.post_transit_connection_destination_subset
                                                  .empty_street_name_labels,
                                             true);

  // Assign the begin street names
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(),
                      &dictionary_.post_transit_connection_destination_subset
                           .empty_street_name_labels);

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (maneuver.travel_mode() == TripLeg_TravelMode_kDrive) {
    phrase_id += 4;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kPedestrian) {
    phrase_id += 8;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) {
    phrase_id += 16;
  }

  // Set instruction to the determined tagged phrase
  instruction =
      dictionary_.post_transit_connection_destination_subset.phrases.at(std::to_string(phrase_id));

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

std::string
NarrativeBuilder::FormVerbalPostTransitConnectionDestinationInstruction(Maneuver& maneuver,
                                                                        uint32_t element_max_count,
                                                                        const std::string& delim) {
  // 0 "Head <FormCardinalDirection>."
  // 1 "Head <FormCardinalDirection> on <STREET_NAMES>."
  // 2 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>."
  // "4": "Drive <CARDINAL_DIRECTION>.",
  // "5": "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "6": "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "8": "Walk <CARDINAL_DIRECTION>.",
  // "9": "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "10": "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.",
  // "16": "Bike <CARDINAL_DIRECTION>.",
  // "17": "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "18": "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set cardinal_direction value
  std::string cardinal_direction =
      dictionary_.post_transit_connection_destination_verbal_subset.cardinal_directions.at(
          maneuver.begin_cardinal_direction());

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.post_transit_connection_destination_verbal_subset
                           .empty_street_name_labels,
                      true, element_max_count, delim, maneuver.verbal_formatter());

  // Assign the begin street names
  std::string begin_street_names =
      FormStreetNames(maneuver, maneuver.begin_street_names(),
                      &dictionary_.post_transit_connection_destination_verbal_subset
                           .empty_street_name_labels,
                      false, element_max_count, delim, maneuver.verbal_formatter());

  // Determine which phrase to use
  uint8_t phrase_id = 0;
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (maneuver.travel_mode() == TripLeg_TravelMode_kDrive) {
    phrase_id += 4;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kPedestrian) {
    phrase_id += 8;
  } else if (maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) {
    phrase_id += 16;
  }

  // Set instruction to the determined tagged phrase
  instruction = dictionary_.post_transit_connection_destination_verbal_subset.phrases.at(
      std::to_string(phrase_id));

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

std::string NarrativeBuilder::FormVerbalPostTransitionInstruction(Maneuver& maneuver,
                                                                  bool include_street_names,
                                                                  uint32_t element_max_count,
                                                                  const std::string& delim) {
  // "0": "Continue for <LENGTH>.",
  // "1": "Continue on <STREET_NAMES> for <LENGTH>."

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Assign the street names
  std::string street_names =
      FormStreetNames(maneuver, maneuver.street_names(),
                      &dictionary_.post_transition_verbal_subset.empty_street_name_labels, true,
                      element_max_count, delim, maneuver.verbal_formatter());

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
      return FormUsCustomaryLength(maneuver.length(Options::miles), us_customary_lengths);
    }
    default: { return FormMetricLength(maneuver.length(Options::kilometers), metric_lengths); }
  }
}

std::string NarrativeBuilder::FormMetricLength(float kilometers,
                                               const std::vector<std::string>& metric_lengths) {

  // 0 "<KILOMETERS> kilometers"
  // 1 "1 kilometer"
  // 2 "a half kilometer"
  // 3 "<METERS> meters" (30-400 and 600-900 meters)
  // 4 "less than 10 meters"

  std::string length_string;
  length_string.reserve(kLengthStringInitialCapacity);

  // Follow locale rules turning numbers into strings
  std::stringstream distance;
  distance.imbue(dictionary_.GetLocale());
  // These will determine what we say
  int tenths = std::round(kilometers * 10);

  if (tenths > 10) {
    // 0 "<KILOMETERS> kilometers"
    length_string += metric_lengths.at(kKilometersIndex);
    distance << std::setiosflags(std::ios::fixed) << std::setprecision(tenths % 10 > 0) << kilometers;
  } else if (tenths == 10) {
    // 1 "1 kilometer"
    length_string += metric_lengths.at(kOneKilometerIndex);
  } else if (tenths == 5) {
    // 2 "a half kilometer"
    length_string += metric_lengths.at(kHalfKilometerIndex);
  } else {
    int meters = std::round(kilometers * 1000);
    if (meters > 94) {
      // 3 "<METERS> meters" (100-400 and 600-900 meters)
      length_string += metric_lengths.at(kMetersIndex);
      distance << ((meters + 50) / 100) * 100;
    } else if (meters > 9) {
      // 3 "<METERS> meters" (10-90 meters)
      length_string += metric_lengths.at(kMetersIndex);
      distance << ((meters + 5) / 10) * 10;
    } else {
      // 4 "less than 10 meters"
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
  // 3  "<TENTHS_OF_MILE> tenths of a mile" (2-4, 6-9)
  // 4  "1 tenth of a mile"
  // 5  "<FEET> feet" (10-90, 100-500)
  // 6  "less than 10 feet"

  std::string length_string;
  length_string.reserve(kLengthStringInitialCapacity);

  // Follow locale rules turning numbers into strings
  std::stringstream distance;
  distance.imbue(dictionary_.GetLocale());
  // These will determine what we say
  int tenths = std::round(miles * 10);

  if (tenths > 10) {
    // 0  "<MILES> miles"
    length_string += us_customary_lengths.at(kMilesIndex);
    distance << std::setiosflags(std::ios::fixed) << std::setprecision(tenths % 10 > 0) << miles;
  } else if (tenths == 10) {
    // 1  "1 mile"
    length_string += us_customary_lengths.at(kOneMileIndex);
  } else if (tenths == 5) {
    // 2  "a half mile"
    length_string += us_customary_lengths.at(kHalfMileIndex);
  } else if (tenths > 1) {
    // 3  "<TENTHS_OF_MILE> tenths of a mile" (2-4, 6-9)
    length_string += us_customary_lengths.at(kTenthsOfMileIndex);
    distance << tenths;
  } else if (miles > 0.0973f && tenths == 1) {
    // 4  "1 tenth of a mile"
    length_string += us_customary_lengths.at(kOneTenthOfMileIndex);
  } else {
    int feet = std::round(miles * 5280);
    if (feet > 94) {
      // 5  "<FEET> feet" (100-500)
      length_string += us_customary_lengths.at(kFeetIndex);
      distance << ((feet + 50) / 100) * 100;
    } else if (feet > 9) {
      // 5  "<FEET> feet" (10-90)
      length_string += us_customary_lengths.at(kFeetIndex);
      distance << ((feet + 5) / 10) * 10;
    } else {
      // 6  "less than 10 feet"
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
    default: { throw valhalla_exception_t{231}; }
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
    default: { throw valhalla_exception_t{232}; }
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

    // If pedestrian travel mode on unnamed footway
    // then set street names string to walkway
    if ((maneuver.travel_mode() == TripLeg_TravelMode_kPedestrian) && maneuver.unnamed_walkway()) {
      street_names_string = empty_street_name_labels->at(kWalkwayIndex);
    }

    // If bicycle travel mode on unnamed cycleway
    // then set street names string to cycleway
    if ((maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) && maneuver.unnamed_cycleway()) {
      street_names_string = empty_street_name_labels->at(kCyclewayIndex);
    }

    // If bicycle travel mode on unnamed mountain bike trail
    // then set street names string to mountain bike trail
    if ((maneuver.travel_mode() == TripLeg_TravelMode_kBicycle) &&
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
    street_names_string +=
        (verbal_formatter) ? verbal_formatter->Format(street_name->value()) : street_name->value();
    ++count;
  }
  return street_names_string;
}

void NarrativeBuilder::FormVerbalMultiCue(std::list<Maneuver>& maneuvers) {
  Maneuver* prev_maneuver = nullptr;
  for (auto& maneuver : maneuvers) {
    if (prev_maneuver && IsVerbalMultiCuePossible(prev_maneuver, maneuver)) {
      // Set verbal pre transition instruction as a verbal multi-cue
      prev_maneuver->set_verbal_pre_transition_instruction(
          FormVerbalMultiCue(prev_maneuver, maneuver));
      prev_maneuver->set_verbal_multi_cue(true);
    }

    // Update previous maneuver
    prev_maneuver = &maneuver;
  }
}

std::string NarrativeBuilder::FormVerbalMultiCue(Maneuver* maneuver, Maneuver& next_maneuver) {
  // "0": "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>"

  std::string instruction;
  instruction.reserve(kInstructionInitialCapacity);

  // Set current verbal cue
  const std::string& current_verbal_cue = maneuver->verbal_pre_transition_instruction();

  // Set next verbal cue
  std::string next_verbal_cue = next_maneuver.HasVerbalTransitionAlertInstruction()
                                    ? next_maneuver.verbal_transition_alert_instruction()
                                    : next_maneuver.verbal_pre_transition_instruction();

  // Set instruction to the verbal multi-cue
  instruction = dictionary_.verbal_multi_cue_subset.phrases.at("0");

  // Replace phrase tags with values
  boost::replace_all(instruction, kCurrentVerbalCueTag, current_verbal_cue);
  boost::replace_all(instruction, kNextVerbalCueTag, next_verbal_cue);

  // If enabled, form articulated prepositions
  if (articulated_preposition_enabled_) {
    FormArticulatedPrepositions(instruction);
  }

  return instruction;
}

bool NarrativeBuilder::IsVerbalMultiCuePossible(Maneuver* maneuver, Maneuver& next_maneuver) {
  // Current maneuver must have a verbal pre-transition instruction
  // Next maneuver must have a verbal transition alert or a verbal pre-transition instruction
  // Current maneuver must be quick (basic time < 10 sec)
  // Next maneuver must not be a merge
  // Current and next maneuvers must not be a rouandbout
  // Current and next maneuvers must not be transit or transit connection
  if (maneuver->HasVerbalPreTransitionInstruction() &&
      (next_maneuver.HasVerbalTransitionAlertInstruction() ||
       next_maneuver.HasVerbalPreTransitionInstruction()) &&
      maneuver->basic_time() < kVerbalMultiCueTimeThreshold && !next_maneuver.IsMergeType() &&
      !maneuver->roundabout() && !next_maneuver.roundabout() && !maneuver->IsTransit() &&
      !next_maneuver.IsTransit() && !maneuver->transit_connection() &&
      !next_maneuver.transit_connection()) {
    return true;
  }
  return false;
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
std::string NarrativeBuilder_hiIN::GetPluralCategory(size_t count) {
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

} // namespace odin
} // namespace valhalla
