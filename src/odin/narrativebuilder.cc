#include <iostream>
#include <cmath>
#include <string>
#include <valhalla/midgard/logging.h>

#include "odin/narrativebuilder.h"
#include "odin/enhancedtrippath.h"
#include "odin/maneuver.h"
#include <valhalla/baldr/verbal_text_formatter.h>

#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/algorithm/string.hpp>

namespace {
// Text instruction initial capacity
constexpr auto kTextInstructionInitialCapacity = 128;

// Instruction tags
constexpr auto kCardinalDirectionTag = "<CARDINAL_DIRECTION>";
constexpr auto kStreetNamesTag = "<STREET_NAMES>";
constexpr auto kBeginStreetNamesTag = "<BEGIN_STREET_NAMES>";
}

namespace valhalla {
namespace odin {

NarrativeBuilder::NarrativeBuilder(
    const DirectionsOptions& directions_options,
    const EnhancedTripPath* trip_path,
    const boost::property_tree::ptree& dictionary)
    : directions_options_(directions_options),
      trip_path_(trip_path),
      dictionary_(dictionary) {
}

void NarrativeBuilder::Build(const DirectionsOptions& directions_options,
                             const EnhancedTripPath* etp,
                             std::list<Maneuver>& maneuvers) {
  Maneuver* prev_maneuver = nullptr;
  for (auto& maneuver : maneuvers) {
    switch (maneuver.type()) {
      case TripDirections_Maneuver_Type_kStartRight:
      case TripDirections_Maneuver_Type_kStart:
      case TripDirections_Maneuver_Type_kStartLeft: {
        // Set instruction
        maneuver.set_instruction(std::move(FormStartInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(
                FormVerbalStartInstruction(maneuver,
                                           directions_options.units())));

        // Set verbal post transition instruction only if there are
        // begin street names
        if (maneuver.HasBeginStreetNames()) {
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units(),
                      maneuver.HasBeginStreetNames())));
        }
        break;
      }
      case TripDirections_Maneuver_Type_kDestinationRight:
      case TripDirections_Maneuver_Type_kDestination:
      case TripDirections_Maneuver_Type_kDestinationLeft: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormDestinationInstruction(etp, maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertDestinationInstruction(etp, maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalDestinationInstruction(etp, maneuver)));
        break;
      }
      case TripDirections_Maneuver_Type_kBecomes: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormBecomesInstruction(maneuver, prev_maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalBecomesInstruction(maneuver, prev_maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units(),
                    maneuver.HasBeginStreetNames())));
        break;
      }
      case TripDirections_Maneuver_Type_kSlightRight:
      case TripDirections_Maneuver_Type_kSlightLeft: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormBearInstruction(maneuver, prev_maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertBearInstruction(maneuver, prev_maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalBearInstruction(maneuver, prev_maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units(),
                    maneuver.HasBeginStreetNames())));
        break;
      }
      case TripDirections_Maneuver_Type_kRight:
      case TripDirections_Maneuver_Type_kSharpRight:
      case TripDirections_Maneuver_Type_kSharpLeft:
      case TripDirections_Maneuver_Type_kLeft: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormTurnInstruction(maneuver, prev_maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertTurnInstruction(maneuver, prev_maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalTurnInstruction(maneuver, prev_maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units(),
                    maneuver.HasBeginStreetNames())));
        break;
      }
      case TripDirections_Maneuver_Type_kUturnRight:
      case TripDirections_Maneuver_Type_kUturnLeft: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormUturnInstruction(maneuver, prev_maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertUturnInstruction(maneuver, prev_maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalUturnInstruction(maneuver, prev_maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kRampStraight: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormRampStraightInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertRampStraightInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalRampStraightInstruction(maneuver)));

        // Only set verbal post if > min ramp length
        if (maneuver.length() > kVerbalPostMinimumRampLength) {
          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units())));
        }
        break;
      }
      case TripDirections_Maneuver_Type_kRampRight:
      case TripDirections_Maneuver_Type_kRampLeft: {
        // Set instruction
        maneuver.set_instruction(std::move(FormRampInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertRampInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalRampInstruction(maneuver)));

        // Only set verbal post if > min ramp length
        if (maneuver.length() > kVerbalPostMinimumRampLength) {
          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units())));
        }
        break;
      }
      case TripDirections_Maneuver_Type_kExitRight:
      case TripDirections_Maneuver_Type_kExitLeft: {
        // Set instruction
        maneuver.set_instruction(std::move(FormExitInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertExitInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalExitInstruction(maneuver)));

        // Only set verbal post if > min ramp length
        if (maneuver.length() > kVerbalPostMinimumRampLength) {
          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units())));
        }
        break;
      }
      case TripDirections_Maneuver_Type_kStayStraight:
      case TripDirections_Maneuver_Type_kStayRight:
      case TripDirections_Maneuver_Type_kStayLeft: {
        if (maneuver.HasSimilarNames(prev_maneuver)) {
          // Set stay on instruction
          maneuver.set_instruction(
              std::move(FormKeepToStayOnInstruction(maneuver)));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              std::move(FormVerbalAlertKeepToStayOnInstruction(maneuver)));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(
              std::move(FormVerbalKeepToStayOnInstruction(maneuver)));

          // Only set verbal post if > min ramp length
          if (maneuver.length() > kVerbalPostMinimumRampLength) {
            // Set verbal post transition instruction
            maneuver.set_verbal_post_transition_instruction(
                std::move(
                    FormVerbalPostTransitionInstruction(
                        maneuver, directions_options.units())));
          }
        } else {
          // Set instruction
          maneuver.set_instruction(std::move(FormKeepInstruction(maneuver)));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              std::move(FormVerbalAlertKeepInstruction(maneuver)));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(
              std::move(FormVerbalKeepInstruction(maneuver)));

          // Only set verbal post if > min ramp length
          if (maneuver.length() > kVerbalPostMinimumRampLength) {
            // Set verbal post transition instruction
            maneuver.set_verbal_post_transition_instruction(
                std::move(
                    FormVerbalPostTransitionInstruction(
                        maneuver, directions_options.units())));
          }
        }
        break;
      }
      case TripDirections_Maneuver_Type_kMerge: {
        // Set instruction
        maneuver.set_instruction(std::move(FormMergeInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertMergeInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalMergeInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kRoundaboutEnter: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormEnterRoundaboutInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertEnterRoundaboutInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalEnterRoundaboutInstruction(maneuver)));
        break;
      }
      case TripDirections_Maneuver_Type_kRoundaboutExit: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormExitRoundaboutInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalExitRoundaboutInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units(),
                    maneuver.HasBeginStreetNames())));
        break;
      }
      case TripDirections_Maneuver_Type_kFerryEnter: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormEnterFerryInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertEnterFerryInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalEnterFerryInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kFerryExit: {
        // Set instruction
        maneuver.set_instruction(std::move(FormExitFerryInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertExitFerryInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalExitFerryInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units(),
                    maneuver.HasBeginStreetNames())));
        break;
      }
      case TripDirections_Maneuver_Type_kTransitConnectionStart: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormTransitConnectionStartInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalTransitConnectionStartInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kTransitConnectionTransfer: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormTransitConnectionTransferInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(
                FormVerbalTransitConnectionTransferInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kTransitConnectionDestination: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormTransitConnectionDestinationInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(
                FormVerbalTransitConnectionDestinationInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kTransit: {
        // Set depart instruction
        maneuver.set_depart_instruction(
            std::move(FormDepartInstruction(maneuver)));

        // Set verbal depart instruction
        maneuver.set_verbal_depart_instruction(
            std::move(FormVerbalDepartInstruction(maneuver)));

        // Set instruction
        maneuver.set_instruction(std::move(FormTransitInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalTransitInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(FormVerbalPostTransitionTransitInstruction(maneuver)));

        // Set arrive instruction
        maneuver.set_arrive_instruction(
            std::move(FormArriveInstruction(maneuver)));

        // Set verbal arrive instruction
        maneuver.set_verbal_arrive_instruction(
            std::move(FormVerbalArriveInstruction(maneuver)));

        break;
      }
      case TripDirections_Maneuver_Type_kTransitRemainOn: {
        // Set depart instruction
        maneuver.set_depart_instruction(
            std::move(FormDepartInstruction(maneuver)));

        // Set verbal depart instruction
        maneuver.set_verbal_depart_instruction(
            std::move(FormVerbalDepartInstruction(maneuver)));

        // Set instruction
        maneuver.set_instruction(
            std::move(FormTransitRemainOnInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalTransitRemainOnInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(FormVerbalPostTransitionTransitInstruction(maneuver)));

        // Set arrive instruction
        maneuver.set_arrive_instruction(
            std::move(FormArriveInstruction(maneuver)));

        // Set verbal arrive instruction
        maneuver.set_verbal_arrive_instruction(
            std::move(FormVerbalArriveInstruction(maneuver)));

        break;
      }
      case TripDirections_Maneuver_Type_kTransitTransfer: {
        // Set depart instruction
        maneuver.set_depart_instruction(
            std::move(FormDepartInstruction(maneuver)));

        // Set verbal depart instruction
        maneuver.set_verbal_depart_instruction(
            std::move(FormVerbalDepartInstruction(maneuver)));

        // Set instruction
        maneuver.set_instruction(
            std::move(FormTransitTransferInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalTransitTransferInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(FormVerbalPostTransitionTransitInstruction(maneuver)));

        // Set arrive instruction
        maneuver.set_arrive_instruction(
            std::move(FormArriveInstruction(maneuver)));

        // Set verbal arrive instruction
        maneuver.set_verbal_arrive_instruction(
            std::move(FormVerbalArriveInstruction(maneuver)));
        break;
      }
      case TripDirections_Maneuver_Type_kPostTransitConnectionDestination: {
        // Set instruction
        maneuver.set_instruction(
            std::move(
                FormPostTransitConnectionDestinationInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(
                FormVerbalPostTransitConnectionDestinationInstruction(
                    maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kContinue:
      default: {
        // Set instruction
        maneuver.set_instruction(std::move(FormContinueInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertContinueInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(
                FormVerbalContinueInstruction(maneuver,
                                              directions_options.units())));

        // NOTE: No verbal post transition instruction
        break;
      }

    }

    // Update previous maneuver
    prev_maneuver = &maneuver;
  }
}

// TODO - we will have to optimize when we actually use the language specific
// dictionary

std::string NarrativeBuilder::FormStartInstruction(Maneuver& maneuver) {
  // "0": "Head <CARDINAL_DIRECTION>.",
  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",

  // Get the start group
  const auto& start_group = dictionary_.get_child("instructions.start");

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  std::string cardinal_direction = FormCardinalDirection(
      maneuver.begin_cardinal_direction());
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names());
  uint8_t phrase_id = 0;

  if (!street_names.empty()) {
    phrase_id += 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id += 1;
  }

  instruction = start_group.get<std::string>(std::to_string(phrase_id));
  LOG_TRACE("start_group=" + instruction);

  boost::replace_all(instruction, kCardinalDirectionTag, cardinal_direction);
  boost::replace_all(instruction, kStreetNamesTag, street_names);
  boost::replace_all(instruction, kBeginStreetNamesTag, begin_street_names);
  LOG_TRACE("start_group=" + instruction);

  // TODO - side of street

  return instruction;
}

std::string NarrativeBuilder::FormVerbalStartInstruction(
    Maneuver& maneuver, DirectionsOptions_Units units,
    uint32_t element_max_count, std::string delim) {
  // 0 "Head <FormCardinalDirection> for <DISTANCE>."
  // 1 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>."
  // 2 "Head <FormCardinalDirection> on <STREET_NAMES> for <DISTANCE>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  std::string cardinal_direction = FormCardinalDirection(
      maneuver.begin_cardinal_direction());
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  uint8_t phrase_id = 0;

  if (!begin_street_names.empty()) {
    phrase_id = 1;
  } else if (!street_names.empty()) {
    phrase_id = 2;
  }

  switch (phrase_id) {
    // 1 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>."
    case 1: {
      instruction = (boost::format("Head %1% on %2%.")
          % cardinal_direction % begin_street_names).str();
      break;
    }
    // 2 "Head <FormCardinalDirection> on <STREET_NAMES> for <DISTANCE>."
    case 2: {
      instruction = (boost::format("Head %1% on %2% for %3%.")
          % cardinal_direction % street_names % FormDistance(maneuver, units))
          .str();
      break;
    }
    // 0 "Head <FormCardinalDirection> for <DISTANCE>."
    default: {
      instruction = (boost::format("Head %1% for %2%.") % cardinal_direction
          % FormDistance(maneuver, units)).str();
      break;
    }
  }
  // TODO - side of street

  return instruction;
}

std::string NarrativeBuilder::FormDestinationInstruction(
    const EnhancedTripPath* etp, Maneuver& maneuver) {
  // 0 "You have arrived at your destination."
  // 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
  // 2 "Your destination is on the <SOS>."
  // 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."

  uint8_t phrase_id = 0;
  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // Determine if location (name or street) exists
  std::string location;
  auto& dest = etp->GetDestination();
  // Check for location name
  if (dest.has_name() && !(dest.name().empty())) {
    phrase_id += 1;
    location = dest.name();
  }
  // Check for location street
  else if (dest.has_street() && !(dest.street().empty())) {
    phrase_id += 1;
    location = dest.street();
  }

  // Check for side of street
  std::string sos;
  if ((maneuver.type() == TripDirections_Maneuver_Type_kDestinationLeft)
      || (maneuver.type() == TripDirections_Maneuver_Type_kDestinationRight)) {
    phrase_id += 2;
    sos = FormTurnTypeInstruction(maneuver.type());
  }

  switch (phrase_id) {
    // 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
    case 1: {
      instruction = (boost::format("You have arrived at %1%.") % location).str();
      break;
    }
    // 2 "Your destination is on the <SOS>."
    case 2: {
      instruction = (boost::format("Your destination is on the %1%.")
          % sos).str();
      break;
    }
    // 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."
    case 3: {
      instruction = (boost::format(
          "%1% is on the %2%.") % location
          % sos).str();
      break;
    }
    // 0 "You have arrived at your destination."
    default: {
      instruction = "You have arrived at your destination.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertDestinationInstruction(
    const EnhancedTripPath* etp, Maneuver& maneuver) {
  // 0 "You will arrive at your destination."
  // 1 "You will arrive at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
  // 2 "Your destination will be on the <SOS>."
  // 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> will be on the <SOS>."

  uint8_t phrase_id = 0;
  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // Determine if location (name or street) exists
  std::string location;
  auto& dest = etp->GetDestination();
  // Check for location name
  if (dest.has_name() && !(dest.name().empty())) {
    phrase_id += 1;
    location = dest.name();
  }
  // Check for location street
  else if (dest.has_street() && !(dest.street().empty())) {
    phrase_id += 1;
    auto* verbal_formatter = maneuver.verbal_formatter();
    if (verbal_formatter)
      location = verbal_formatter->Format(dest.street());
    else
      location = dest.street();
  }

  // Check for side of street
  std::string sos;
  if ((maneuver.type() == TripDirections_Maneuver_Type_kDestinationLeft)
      || (maneuver.type() == TripDirections_Maneuver_Type_kDestinationRight)) {
    phrase_id += 2;
    sos = FormTurnTypeInstruction(maneuver.type());
  }

  switch (phrase_id) {
    // 1 "You will arrive at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
    case 1: {
      instruction = (boost::format("You will arrive at %1%.") % location).str();
      break;
    }
    // 2 "Your destination will be on the <SOS>."
    case 2: {
      instruction = (boost::format("Your destination will be on the %1%.")
          % sos).str();
      break;
    }
    // 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> will be on the <SOS>."
    case 3: {
      instruction = (boost::format(
          "%1% will be on the %2%.") % location
          % sos).str();
      break;
    }
    // 0 "You will arrive at your destination."
    default: {
      instruction = "You will arrive at your destination.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalDestinationInstruction(
    const EnhancedTripPath* etp, Maneuver& maneuver) {
  // 0 "You have arrived at your destination."
  // 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
  // 2 "Your destination is on the <SOS>."
  // 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."

  uint8_t phrase_id = 0;
  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // Determine if location (name or street) exists
  std::string location;
  auto& dest = etp->GetDestination();
  // Check for location name
  if (dest.has_name() && !(dest.name().empty())) {
    phrase_id += 1;
    location = dest.name();
  }
  // Check for location street
  else if (dest.has_street() && !(dest.street().empty())) {
    phrase_id += 1;
    auto* verbal_formatter = maneuver.verbal_formatter();
    if (verbal_formatter)
      location = verbal_formatter->Format(dest.street());
    else
      location = dest.street();
  }

  // Check for side of street
  std::string sos;
  if ((maneuver.type() == TripDirections_Maneuver_Type_kDestinationLeft)
      || (maneuver.type() == TripDirections_Maneuver_Type_kDestinationRight)) {
    phrase_id += 2;
    sos = FormTurnTypeInstruction(maneuver.type());
  }

  switch (phrase_id) {
    // 1 "You have arrived at <LOCATION_NAME|LOCATION_STREET_ADDRESS>."
    case 1: {
      instruction = (boost::format("You have arrived at %1%.") % location).str();
      break;
    }
    // 2 "Your destination is on the <SOS>."
    case 2: {
      instruction = (boost::format("Your destination is on the %1%.")
          % sos).str();
      break;
    }
    // 3 "<LOCATION_NAME|LOCATION_STREET_ADDRESS> is on the <SOS>."
    case 3: {
      instruction = (boost::format(
          "%1% is on the %2%.") % location
          % sos).str();
      break;
    }
    // 0 "You have arrived at your destination."
    default: {
      instruction = "You have arrived at your destination.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormBecomesInstruction(Maneuver& maneuver,
                                              Maneuver* prev_maneuver) {
  // "<PREV_STREET_NAMES> becomes <STREET_NAMES>."

  // Assign the street names and the previous maneuver street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string prev_street_names;
  if (prev_maneuver) {
    prev_street_names = FormStreetNames(maneuver,
                                        prev_maneuver->street_names());
  }

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // If previous maneuver has names
  // and current maneuver has names
  // then form "becomes" narrative
  if (!prev_street_names.empty() && !street_names.empty()) {
    instruction += prev_street_names;
    instruction += " becomes ";
    instruction += street_names;
  }
  // Items are missing - fallback to just "Continue" narrative
  else {
    instruction += "Continue";

    if (!street_names.empty()) {
      instruction += " on ";
      instruction += street_names;
    }
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalBecomesInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  // "<PREV_STREET_NAMES(2)> becomes <STREET_NAMES(2)>."

  // Assign the street names and the previous maneuver street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string prev_street_names;
  if (prev_maneuver) {
    prev_street_names = FormStreetNames(maneuver, maneuver.begin_street_names(),
                                        false, element_max_count, delim,
                                        maneuver.verbal_formatter());
  }
  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // If previous maneuver has names
  // and current maneuver has names
  // then form "becomes" narrative
  if (!prev_street_names.empty() && !street_names.empty()) {
    instruction += prev_street_names;
    instruction += " becomes ";
    instruction += street_names;
  }
  // Items are missing - fallback to just "Continue" narrative
  else {
    instruction += "Continue";

    if (!street_names.empty()) {
      instruction += " on ";
      instruction += street_names;
    }
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormContinueInstruction(Maneuver& maneuver) {
  // "Continue"
  // "Continue on <STREET_NAMES>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (!street_names.empty()) {
    instruction += " on ";
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertContinueInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Continue"
  //  "Continue on <STREET_NAMES(1)>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (!street_names.empty()) {
    instruction += " on ";
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalContinueInstruction(
    Maneuver& maneuver, DirectionsOptions_Units units,
    uint32_t element_max_count, std::string delim) {
  //  "Continue for <DISTANCE>"
  //  "Continue on <STREET_NAMES(2)> for <DISTANCE>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (!street_names.empty()) {
    instruction += " on ";
    instruction += street_names;
  }

  instruction += " for ";
  instruction += FormDistance(maneuver, units);
  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormTurnInstruction(Maneuver& maneuver,
                                                  Maneuver* prev_maneuver) {
  // 0 "Turn <FormTurnTypeInstruction>."
  // 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES>."
  // 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
  // 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES>."

  // TODO
  // maneuver.HasSimilarNames(prev_maneuver, true))

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  }
  if (begin_street_names.empty()
      && maneuver.HasSimilarNames(prev_maneuver, true)) {
    phrase_id = 3;
  }

  switch (phrase_id) {
    // 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES>."
    case 1: {
      instruction = (boost::format("Turn %1% onto %2%.")
          % FormTurnTypeInstruction(maneuver.type())
          % street_names).str();
      break;
    }
      // 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
    case 2: {
      instruction = (boost::format("Turn %1% onto %2%. Continue on %3%.")
          % FormTurnTypeInstruction(maneuver.type())
          % begin_street_names
          % street_names).str();
      break;
    }
      // 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
    case 3: {
      instruction = (boost::format("Turn %1% to stay on %2%.")
          % FormTurnTypeInstruction(maneuver.type())
          % street_names).str();
      break;
    }
      // 0 "Turn <FormTurnTypeInstruction>."
    default: {
      instruction = (boost::format("Turn %1%.")
          % FormTurnTypeInstruction(maneuver.type())).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertTurnInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  // 0 "Turn <FormTurnTypeInstruction>."
  // 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES(1)>."
  // 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(1)>."
  // 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES(1)>."

  return FormVerbalTurnInstruction(maneuver, prev_maneuver, element_max_count,
                                   delim);
}

std::string NarrativeBuilder::FormVerbalTurnInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  // 0 "Turn <FormTurnTypeInstruction>."
  // 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES(2)>."
  // 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(2)>."
  // 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  }
  if (begin_street_names.empty()
      && maneuver.HasSimilarNames(prev_maneuver, true)) {
    phrase_id = 3;
  }

  switch (phrase_id) {
    // 1 "Turn <FormTurnTypeInstruction> onto <STREET_NAMES(2)>."
    case 1: {
      instruction =
          (boost::format("Turn %1% onto %2%.")
              % FormTurnTypeInstruction(maneuver.type())
              % street_names).str();
      break;
    }
    // 2 "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(2)>."
    case 2: {
      instruction =
          (boost::format("Turn %1% onto %2%.")
              % FormTurnTypeInstruction(maneuver.type())
              % begin_street_names).str();
      break;
    }
    // 3 "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."
    case 3: {
      instruction =
          (boost::format("Turn %1% to stay on %2%.")
              % FormTurnTypeInstruction(maneuver.type())
              % street_names).str();
      break;
    }
    // 0 "Turn <FormTurnTypeInstruction>."
    default: {
      instruction = (boost::format("Turn %1%.")
          % FormTurnTypeInstruction(maneuver.type())).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormBearInstruction(Maneuver& maneuver,
                                                  Maneuver* prev_maneuver) {
  //  0 "Bear <FormTurnTypeInstruction>."
  //  1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES>."
  //  2 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
  //  3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  }
  if (begin_street_names.empty()
      && maneuver.HasSimilarNames(prev_maneuver, true)) {
    phrase_id = 3;
  }

  switch (phrase_id) {
    // 1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES>."
    case 1: {
      instruction = (boost::format("Bear %1% onto %2%.")
          % FormTurnTypeInstruction(maneuver.type())
          % street_names).str();
      break;
    }
      // 2 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
    case 2: {
      instruction = (boost::format("Bear %1% onto %2%. Continue on %3%.")
          % FormTurnTypeInstruction(maneuver.type())
          % begin_street_names
          % street_names).str();
      break;
    }
      // 3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
    case 3: {
      instruction = (boost::format("Bear %1% to stay on %2%.")
          % FormTurnTypeInstruction(maneuver.type())
          % street_names).str();
      break;
    }
      // 0 "Bear <FormTurnTypeInstruction>."
    default: {
      instruction = (boost::format("Bear %1%.")
          % FormTurnTypeInstruction(maneuver.type())).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertBearInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  //  0 "Bear <FormTurnTypeInstruction>."
  //  1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES(1)>."
  //  2 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(1)>."
  //  3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES(1)>."

  return FormVerbalBearInstruction(maneuver, prev_maneuver, element_max_count,
                                   delim);
}

std::string NarrativeBuilder::FormVerbalBearInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  //  0 "Bear <FormTurnTypeInstruction>."
  //  1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES(2)>."
  //  1 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(2)>."
  //  3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!street_names.empty()) {
    phrase_id = 1;
  }
  if (!begin_street_names.empty()) {
    phrase_id = 2;
  }
  if (begin_street_names.empty()
      && maneuver.HasSimilarNames(prev_maneuver, true)) {
    phrase_id = 3;
  }

  switch (phrase_id) {
    // 1 "Bear <FormTurnTypeInstruction> onto <STREET_NAMES(2)>."
    case 1: {
      instruction =
          (boost::format("Bear %1% onto %2%.")
              % FormTurnTypeInstruction(maneuver.type())
              % street_names).str();
      break;
    }
    // 2 "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES(2)>."
    case 2: {
      instruction =
          (boost::format("Bear %1% onto %2%.")
              % FormTurnTypeInstruction(maneuver.type())
              % begin_street_names).str();
      break;
    }
    // 3 "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."
    case 3: {
      instruction =
          (boost::format("Bear %1% to stay on %2%.")
              % FormTurnTypeInstruction(maneuver.type())
              % street_names).str();
      break;
    }
    // 0 "Bear <FormTurnTypeInstruction>."
    default: {
      instruction = (boost::format("Bear %1%.")
          % FormTurnTypeInstruction(maneuver.type())).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormUturnInstruction(Maneuver& maneuver,
                                                   Maneuver* prev_maneuver) {
  // 0 "Make a <FormTurnTypeInstruction> U-turn."
  // 1 "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES>."
  // 2 "Make a <FormTurnTypeInstruction> U-turn to stay on <STREET_NAMES>."
  // 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES>."
  // 4 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>."
  // 5 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
  // TODO: rework with phrase ids

  // Assign the street names and the cross street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string cross_street_names = FormStreetNames(
      maneuver, maneuver.cross_street_names());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Make a ";
  instruction += FormTurnTypeInstruction(maneuver.type());
  instruction += " U-turn";

  if (!cross_street_names.empty()) {
    instruction += " at ";
    instruction += cross_street_names;
  }

  if (!street_names.empty()) {
    if (maneuver.HasSameNames(prev_maneuver, true)) {
      instruction += " to stay on ";
    } else {
      instruction += " onto ";
    }
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertUturnInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  // 0 "Make a <FormTurnTypeInstruction> U-turn."
  // 1 "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES(1)>."
  // 2 "Make a <FormTurnTypeInstruction> U-turn to stay on <STREET_NAMES(1)>."
  // 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(1)>."
  // TODO: rework with phrase ids

  // Assign the street names and the cross street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string cross_street_names = FormStreetNames(
      maneuver, maneuver.cross_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Make a ";
  instruction += FormTurnTypeInstruction(maneuver.type());
  instruction += " U-turn";

  if (!cross_street_names.empty()) {
    instruction += " at ";
    instruction += cross_street_names;
  } else if (!street_names.empty()) {
    if (maneuver.HasSameNames(prev_maneuver, true)) {
      instruction += " to stay on ";
    } else {
      instruction += " onto ";
    }
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalUturnInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  // 0 "Make a <FormTurnTypeInstruction> U-turn."
  // 1 "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES(2)>."
  // 2 "Make a <FormTurnTypeInstruction> U-turn to stay on <STREET_NAMES(2)>."
  // 3 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)>."
  // 4 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)> onto <STREET_NAMES(2)>."
  // 5 "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)> to stay on <STREET_NAMES(2)>."
  // TODO: rework with phrase ids

  // Assign the street names and the cross street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string cross_street_names = FormStreetNames(
      maneuver, maneuver.cross_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Make a ";
  instruction += FormTurnTypeInstruction(maneuver.type());
  instruction += " U-turn";

  if (!cross_street_names.empty()) {
    instruction += " at ";
    instruction += cross_street_names;
  }

  if (!street_names.empty()) {
    if (maneuver.HasSameNames(prev_maneuver, true)) {
      instruction += " to stay on ";
    } else {
      instruction += " onto ";
    }
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormRampStraightInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count) {

  // 0 "Stay straight to take the ramp."
  // 1 "Stay straight to take the <BRANCH_SIGN> ramp."
  // 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
  // 3 "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
  // 4 "Stay straight to take the <NAME_SIGN> ramp."

  // Examples
  // 0 Stay straight to take the ramp
  // 1 Stay straight to take the I 95 South ramp
  // 2 Stay straight to take the ramp toward Baltimore
  // 3 Stay straight to take the I 95 South ramp toward Baltimore
  // 4 Stay straight to take the Gettysburg Pike ramp

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  // 1 = branch
  // 2 = toward
  // 4 = name (if no branch and toward)
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign()
      && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count);
  }

  switch (phrase_id) {
    // 1 "Stay straight to take the <BRANCH_SIGN> ramp."
    case 1: {
      instruction = (boost::format("Stay straight to take the %1% ramp.")
          % exit_branch_sign).str();
      break;
    }
    // 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
    case 2: {
      instruction = (boost::format("Stay straight to take the ramp toward %1%.")
          % exit_toward_sign).str();
      break;
    }
    // 3 "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
    case 3: {
      instruction = (boost::format(
          "Stay straight to take the %1% ramp toward %2%.") % exit_branch_sign
          % exit_toward_sign).str();
      break;
    }
    // 4 "Stay straight to take the <NAME_SIGN> ramp."
    case 4: {
      instruction = (boost::format("Stay straight to take the %1% ramp.")
          % exit_name_sign).str();
      break;
    }
    default: {
      // 0 "Stay straight to take the ramp."
      instruction = "Stay straight to take the ramp.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertRampStraightInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  // 0 "Stay straight to take the ramp."
  // 1 "Stay straight to take the <BRANCH_SIGN(1)> ramp."
  // 2 "Stay straight to take the ramp toward <TOWARD_SIGN(1)>."
  // 3 "Stay straight to take the <NAME_SIGN(1)> ramp."

  // Examples
  // 0 Stay straight to take the ramp
  // 1 Stay straight to take the I 95 South ramp
  // 2 Stay straight to take the ramp toward Baltimore
  // 3 Stay straight to take the Gettysburg Pike ramp

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  // 1 = branch
  // 2 = toward
  // 3 = name (if no branch and toward)
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitBranchSign()) {
    phrase_id = 1;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id = 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  } else if (maneuver.HasExitNameSign()) {
    phrase_id = 3;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  switch (phrase_id) {
    // 1 "Stay straight to take the <BRANCH_SIGN> ramp."
    case 1: {
      instruction = (boost::format("Stay straight to take the %1% ramp.")
          % exit_branch_sign).str();
      break;
    }
    // 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
    case 2: {
      instruction = (boost::format("Stay straight to take the ramp toward %1%.")
          % exit_toward_sign).str();
      break;
    }
    // 3 "Stay straight to take the <NAME_SIGN> ramp."
    case 3: {
      instruction = (boost::format("Stay straight to take the %1% ramp.")
          % exit_name_sign).str();
      break;
    }
    default: {
      // 0 "Stay straight to take the ramp."
      instruction = "Stay straight to take the ramp.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalRampStraightInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  // 0 "Stay straight to take the ramp."
  // 1 "Stay straight to take the <BRANCH_SIGN(2)> ramp."
  // 2 "Stay straight to take the ramp toward <TOWARD_SIGN(2)>."
  // 3 "Stay straight to take the <BRANCH_SIGN(2)> ramp toward <TOWARD_SIGN(2)>."
  // 4 "Stay straight to take the <NAME_SIGN(2)> ramp."

  // Examples
  // 0 Stay straight to take the ramp
  // 1 Stay straight to take the I 95 South ramp
  // 2 Stay straight to take the ramp toward Baltimore
  // 3 Stay straight to take the I 95 South ramp toward Baltimore
  // 4 Stay straight to take the Gettysburg Pike ramp

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  // 1 = branch
  // 2 = toward
  // 4 = name (if no branch and toward)
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign()
      && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  switch (phrase_id) {
    // 1 "Stay straight to take the <BRANCH_SIGN> ramp."
    case 1: {
      instruction = (boost::format("Stay straight to take the %1% ramp.")
          % exit_branch_sign).str();
      break;
    }
    // 2 "Stay straight to take the ramp toward <TOWARD_SIGN>."
    case 2: {
      instruction = (boost::format("Stay straight to take the ramp toward %1%.")
          % exit_toward_sign).str();
      break;
    }
    // 3 "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
    case 3: {
      instruction = (boost::format(
          "Stay straight to take the %1% ramp toward %2%.") % exit_branch_sign
          % exit_toward_sign).str();
      break;
    }
    // 4 "Stay straight to take the <NAME_SIGN> ramp."
    case 4: {
      instruction = (boost::format("Stay straight to take the %1% ramp.")
          % exit_name_sign).str();
      break;
    }
    default: {
      // 0 "Stay straight to take the ramp."
      instruction = "Stay straight to take the ramp.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormRampInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count) {
  // 0 "Take the ramp on the <FormTurnTypeInstruction>."
  // 1 "Take the <BRANCH_SIGN> ramp on the <FormTurnTypeInstruction>."
  // 2 "Take the ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
  // 3 "Take the <BRANCH_SIGN> ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
  // 4 "Take the <NAME_SIGN> ramp on the <FormTurnTypeInstruction>."
  // 5 "Turn <FormTurnTypeInstruction> to take the ramp."
  // 6 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN> ramp."
  // 7 "Turn <FormTurnTypeInstruction> to take the ramp toward <TOWARD_SIGN>."
  // 8 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
  // 9 "Turn <FormTurnTypeInstruction> to take the <NAME_SIGN> ramp."

  // Examples
  // 0 Take the ramp on the right
  // 1 Take the I 95 South ramp on the right
  // 2 Take the ramp on the right toward Baltimore
  // 3 Take the I 95 South ramp on the right toward Baltimore
  // 4 Take the Gettysburg Pike ramp on the right
  // 5 Turn right to take the ramp
  // 6 Turn right to take the I 95 South ramp
  // 7 Turn right to take the ramp toward Baltimore
  // 8 Turn right to take the I 95 South ramp toward Baltimore
  // 9 Turn right to take the Gettysburg Pike ramp

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // 0 = Take the ramp on the right
  // 1 = branch
  // 2 = toward
  // 4 = name (if no branch and toward)
  // 5 = Turn right to take the ramp
  uint8_t phrase_id = 0;

  if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight)
      || (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft))
    phrase_id = 5;

  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign()
      && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count);
  }

  switch (phrase_id) {
    // 1 "Take the <BRANCH_SIGN> ramp on the <FormTurnTypeInstruction>."
    case 1: {
      instruction = (boost::format("Take the %1% ramp on the %2%.")
          % exit_branch_sign % turn).str();
      break;
    }
      // 2 "Take the ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 2: {
      instruction = (boost::format("Take the ramp on the %1% toward %2%.")
          % turn % exit_toward_sign).str();
      break;
    }
      // 3 "Take the <BRANCH_SIGN> ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 3: {
      instruction = (boost::format("Take the %1% ramp on the %2% toward %3%.")
          % exit_branch_sign % turn % exit_toward_sign).str();
      break;
    }
      // 4 "Take the <NAME_SIGN> ramp on the <FormTurnTypeInstruction>."
    case 4: {
      instruction = (boost::format("Take the %1% ramp on the %2%.")
          % exit_name_sign % turn).str();
      break;
    }
      // 5 "Turn <FormTurnTypeInstruction> to take the ramp."
    case 5: {
      instruction = (boost::format("Turn %1% to take the ramp.") % turn).str();
      break;
    }
      // 6 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN> ramp."
    case 6: {
      instruction = (boost::format("Turn %1% to take the %2% ramp.")
          % turn % exit_branch_sign).str();
      break;
    }
      // 7 "Turn <FormTurnTypeInstruction> to take the ramp toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format("Turn %1% to take the ramp toward %2%.")
          % turn % exit_toward_sign).str();
      break;
    }
      // 8 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
    case 8: {
      instruction = (boost::format("Turn %1% to take the %2% ramp toward %3%.")
          % turn % exit_branch_sign % exit_toward_sign).str();
      break;
    }
      // 9 "Turn <FormTurnTypeInstruction> to take the <NAME_SIGN> ramp."
    case 9: {
      instruction = (boost::format("Turn %1% to take the %2% ramp.")
          % turn % exit_name_sign).str();
      break;
    }
      // 0 "Take the ramp on the <FormTurnTypeInstruction>."
    default: {
      instruction = (boost::format("Take the ramp on the %1%.") % turn).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertRampInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {
  // 0 "Take the ramp on the <FormTurnTypeInstruction>."
  // 1 "Take the <BRANCH_SIGN(1)> ramp on the <FormTurnTypeInstruction>."
  // 2 "Take the ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(1)>."
  // 4 "Take the <NAME_SIGN(1)> ramp on the <FormTurnTypeInstruction>."
  // 5 "Turn <FormTurnTypeInstruction> to take the ramp."
  // 6 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN(1)> ramp."
  // 7 "Turn <FormTurnTypeInstruction> to take the ramp toward <TOWARD_SIGN(1)>."
  // 9 "Turn <FormTurnTypeInstruction> to take the <NAME_SIGN(1)> ramp."

  // 0 = Take the ramp on the right
  // 1 = branch
  // 2 = toward
  // 4 = name (if no branch and toward)
  // 5 = Turn right to take the ramp
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight)
      || (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft))
    phrase_id = 5;

  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  } else if (maneuver.HasExitNameSign()) {
    phrase_id += 4;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalRampInstruction(phrase_id,
                                   FormTurnTypeInstruction(maneuver.type()),
                                   exit_branch_sign, exit_toward_sign,
                                   exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalRampInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {
  // 0 "Take the ramp on the <FormTurnTypeInstruction>."
  // 1 "Take the <BRANCH_SIGN(2)> ramp on the <FormTurnTypeInstruction>."
  // 2 "Take the ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(2)>."
  // 3 "Take the <BRANCH_SIGN(2)> ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(2)>."
  // 4 "Take the <NAME_SIGN(2)> ramp on the <FormTurnTypeInstruction>."
  // 5 "Turn <FormTurnTypeInstruction> to take the ramp."
  // 6 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN(2)> ramp."
  // 7 "Turn <FormTurnTypeInstruction> to take the ramp toward <TOWARD_SIGN(2)>."
  // 8 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN(2)> ramp toward <TOWARD_SIGN(2)>."
  // 9 "Turn <FormTurnTypeInstruction> to take the <NAME_SIGN(2)> ramp."


  // 0 = Take the ramp on the right
  // 1 = branch
  // 2 = toward
  // 4 = name (if no branch and toward)
  // 5 = Turn right to take the ramp
  uint8_t phrase_id = 0;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight)
      || (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft))
    phrase_id = 5;

  if (maneuver.HasExitBranchSign()) {
    phrase_id += 1;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitBranchSign()
      && !maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalRampInstruction(phrase_id,
                                   FormTurnTypeInstruction(maneuver.type()),
                                   exit_branch_sign, exit_toward_sign,
                                   exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalRampInstruction(
    uint8_t phrase_id, const std::string& turn,
    const std::string& exit_branch_sign, const std::string& exit_toward_sign,
    const std::string& exit_name_sign) {

  // Examples
  // 0 Take the ramp on the right
  // 1 Take the I 95 South ramp on the right
  // 2 Take the ramp on the right toward Baltimore
  // 3 Take the I 95 South ramp on the right toward Baltimore
  // 4 Take the Gettysburg Pike ramp on the right
  // 5 Turn right to take the ramp
  // 6 Turn right to take the I 95 South ramp
  // 7 Turn right to take the ramp toward Baltimore
  // 8 Turn right to take the I 95 South ramp toward Baltimore
  // 9 Turn right to take the Gettysburg Pike ramp

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  switch (phrase_id) {
    // 1 "Take the <BRANCH_SIGN> ramp on the <FormTurnTypeInstruction>."
    case 1: {
      instruction = (boost::format("Take the %1% ramp on the %2%.")
          % exit_branch_sign % turn).str();
      break;
    }
      // 2 "Take the ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 2: {
      instruction = (boost::format("Take the ramp on the %1% toward %2%.")
          % turn % exit_toward_sign).str();
      break;
    }
      // 3 "Take the <BRANCH_SIGN> ramp on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 3: {
      instruction = (boost::format("Take the %1% ramp on the %2% toward %3%.")
          % exit_branch_sign % turn % exit_toward_sign).str();
      break;
    }
      // 4 "Take the <NAME_SIGN> ramp on the <FormTurnTypeInstruction>."
    case 4: {
      instruction = (boost::format("Take the %1% ramp on the %2%.")
          % exit_name_sign % turn).str();
      break;
    }
      // 5 "Turn <FormTurnTypeInstruction> to take the ramp."
    case 5: {
      instruction = (boost::format("Turn %1% to take the ramp.") % turn).str();
      break;
    }
      // 6 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN> ramp."
    case 6: {
      instruction = (boost::format("Turn %1% to take the %2% ramp.")
          % turn % exit_branch_sign).str();
      break;
    }
      // 7 "Turn <FormTurnTypeInstruction> to take the ramp toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format("Turn %1% to take the ramp toward %2%.")
          % turn % exit_toward_sign).str();
      break;
    }
      // 8 "Turn <FormTurnTypeInstruction> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
    case 8: {
      instruction = (boost::format("Turn %1% to take the %2% ramp toward %3%.")
          % turn % exit_branch_sign % exit_toward_sign).str();
      break;
    }
      // 9 "Turn <FormTurnTypeInstruction> to take the <NAME_SIGN> ramp."
    case 9: {
      instruction = (boost::format("Turn %1% to take the %2% ramp.")
          % turn % exit_name_sign).str();
      break;
    }
      // 0 "Take the ramp on the <FormTurnTypeInstruction>."
    default: {
      instruction = (boost::format("Take the ramp on the %1%.") % turn).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormExitInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count) {
  //  0 "Take the exit on the right"
  //  1 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction>."
  //  2 "Take the <BRANCH_SIGN> exit on the <FormTurnTypeInstruction>."
  //  3 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN>."
  //  4 "Take the exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
  //  5 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
  //  6 "Take the <BRANCH_SIGN> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
  //  7 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
  //  8 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction>."
  //  10 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN>."
  //  12 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
  //  14 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // 0 Take the exit on the right
  // 1 = exit number
  // 2 = branch
  // 4 = toward
  // 8 = name (when no number)
  uint8_t phrase_id = 0;
  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count);
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitNumberSign()) {
    phrase_id += 8;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count);
  }

  switch (phrase_id) {
    //  1 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction>."
    case 1: {
      instruction = (boost::format("Take exit %1% on the %2%.")
          % exit_number_sign % turn).str();
      break;
    }
      //  2 "Take the <BRANCH_SIGN> exit on the <FormTurnTypeInstruction>."
    case 2: {
      instruction = (boost::format("Take the %1% exit on the %2%.")
          % exit_branch_sign % turn).str();
      break;
    }
      //  3 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN>."
    case 3: {
      instruction = (boost::format("Take exit %1% on the %2% onto %3%.")
          % exit_number_sign % turn % exit_branch_sign).str();
      break;
    }
      //  4 "Take the exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 4: {
      instruction = (boost::format("Take the exit on the %1% toward %2%.")
          % turn % exit_toward_sign).str();
      break;
    }
      //  5 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 5: {
      instruction = (boost::format("Take exit %1% on the %2% toward %3%.")
          % exit_number_sign % turn % exit_toward_sign).str();
      break;
    }
      //  6 "Take the <BRANCH_SIGN> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 6: {
      instruction = (boost::format("Take the %1% exit on the %2% toward %3%.")
          % exit_branch_sign % turn % exit_toward_sign).str();
      break;
    }
      //  7 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format(
          "Take exit %1% on the %2% onto %3% toward %4%.") % exit_number_sign
          % turn % exit_branch_sign % exit_toward_sign).str();
      break;
    }
      //  8 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction>."
    case 8: {
      instruction = (boost::format("Take the %1% exit on the %2%.")
          % exit_name_sign % turn).str();
      break;
    }
      //  10 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN>."
    case 10: {
      instruction = (boost::format("Take the %1% exit on the %2% onto %3%.")
          % exit_name_sign % turn % exit_branch_sign).str();
      break;
    }
      //  12 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 12: {
      instruction = (boost::format("Take the %1% exit on the %2% toward %3%.")
          % exit_name_sign % turn % exit_toward_sign).str();
      break;
    }
      //  14 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 14: {
      instruction = (boost::format(
          "Take the %1% exit on the %2% onto %3% toward %4%.") % exit_name_sign
          % turn % exit_branch_sign % exit_toward_sign).str();
      break;
    }
    default: {
      //  0 "Take the exit on the right"
      instruction = (boost::format("Take the exit on the %1%.") % turn).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertExitInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {
  //  0 "Take the exit on the right"
  //  1 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction>."
  //  2 "Take the <BRANCH_SIGN(1)> exit on the <FormTurnTypeInstruction>."
  //  4 "Take the exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(1)>."
  //  8 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction>."

  // 0 Take the exit on the right
  // 1 = exit number
  // 2 = branch
  // 4 = toward
  // 8 = name (when no number)
  uint8_t phrase_id = 0;
  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString(
        0, false, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  } else if (maneuver.HasExitNameSign()) {
    phrase_id += 8;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalExitInstruction(phrase_id, turn, exit_number_sign,
                                   exit_branch_sign, exit_toward_sign,
                                   exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalExitInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {
  //  0 "Take the exit on the right"
  //  1 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction>."
  //  2 "Take the <BRANCH_SIGN(2)> exit on the <FormTurnTypeInstruction>."
  //  3 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN(2)>."
  //  4 "Take the exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(2)>."
  //  5 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(2)>."
  //  6 "Take the <BRANCH_SIGN(2)> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(2)>."
  //  7 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN(2)> toward <TOWARD_SIGN(2)>."
  //  8 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction>."
  //  10 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN(2)>."
  //  12 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN(2)>."
  //  14 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN(2)> toward <TOWARD_SIGN(2)>."

  // 0 Take the exit on the right
  // 1 = exit number
  // 2 = branch
  // 4 = toward
  // 8 = name (when no number)
  uint8_t phrase_id = 0;
  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_branch_sign;
  std::string exit_toward_sign;
  std::string exit_name_sign;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString(
        0, false, delim, maneuver.verbal_formatter());
  }
  if (maneuver.HasExitBranchSign()) {
    phrase_id += 2;
    exit_branch_sign = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  if (maneuver.HasExitNameSign() && !maneuver.HasExitNumberSign()) {
    phrase_id += 8;
    exit_name_sign = maneuver.signs().GetExitNameString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalExitInstruction(phrase_id, turn, exit_number_sign,
                                   exit_branch_sign, exit_toward_sign,
                                   exit_name_sign);
}

std::string NarrativeBuilder::FormVerbalExitInstruction(
    uint8_t phrase_id, const std::string& turn,
    const std::string& exit_number_sign, const std::string& exit_branch_sign,
    const std::string& exit_toward_sign, const std::string& exit_name_sign) {

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  switch (phrase_id) {
    //  1 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction>."
    case 1: {
      instruction = (boost::format("Take exit %1% on the %2%.")
          % exit_number_sign % turn).str();
      break;
    }
      //  2 "Take the <BRANCH_SIGN> exit on the <FormTurnTypeInstruction>."
    case 2: {
      instruction = (boost::format("Take the %1% exit on the %2%.")
          % exit_branch_sign % turn).str();
      break;
    }
      //  3 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN>."
    case 3: {
      instruction = (boost::format("Take exit %1% on the %2% onto %3%.")
          % exit_number_sign % turn % exit_branch_sign).str();
      break;
    }
      //  4 "Take the exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 4: {
      instruction = (boost::format("Take the exit on the %1% toward %2%.")
          % turn % exit_toward_sign).str();
      break;
    }
      //  5 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 5: {
      instruction = (boost::format("Take exit %1% on the %2% toward %3%.")
          % exit_number_sign % turn % exit_toward_sign).str();
      break;
    }
      //  6 "Take the <BRANCH_SIGN> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 6: {
      instruction = (boost::format("Take the %1% exit on the %2% toward %3%.")
          % exit_branch_sign % turn % exit_toward_sign).str();
      break;
    }
      //  7 "Take exit <NUMBER_SIGN> on the <FormTurnTypeInstruction> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format(
          "Take exit %1% on the %2% onto %3% toward %4%.") % exit_number_sign
          % turn % exit_branch_sign % exit_toward_sign).str();
      break;
    }
      //  8 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction>."
    case 8: {
      instruction = (boost::format("Take the %1% exit on the %2%.")
          % exit_name_sign % turn).str();
      break;
    }
      //  10 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN>."
    case 10: {
      instruction = (boost::format("Take the %1% exit on the %2% onto %3%.")
          % exit_name_sign % turn % exit_branch_sign).str();
      break;
    }
      //  12 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 12: {
      instruction = (boost::format("Take the %1% exit on the %2% toward %3%.")
          % exit_name_sign % turn % exit_toward_sign).str();
      break;
    }
      //  14 "Take the <NAME_SIGN> exit on the <FormTurnTypeInstruction> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 14: {
      instruction = (boost::format(
          "Take the %1% exit on the %2% onto %3% toward %4%.") % exit_name_sign
          % turn % exit_branch_sign % exit_toward_sign).str();
      break;
    }
    default: {
      //  0 "Take the exit on the right"
      instruction = (boost::format("Take the exit on the %1%.") % turn).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormKeepInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count) {

  //  0 "Keep <FormTurnTypeInstruction> at the fork."
  //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN>."
  //  2 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN>."
  //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN>."
  //  4 "Keep <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
  //  5 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>."
  //  6 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."
  //  7 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count);

  // If street names string is empty and the maneuver has sign branch info
  // then assign the sign branch name to the street names string
  if (street_names.empty() && maneuver.HasExitBranchSign()) {
    street_names = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count);
  }
  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (!street_names.empty()) {
    phrase_id += 2;
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count);
  }

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  switch (phrase_id) {
    //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN>."
    case 1: {
      instruction = (boost::format("Keep %1% to take exit %2%.") % turn
          % exit_number_sign).str();
      break;
    }
      //  2 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN>."
    case 2: {
      instruction =
          (boost::format("Keep %1% to take %2%.") % turn % street_names).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN>."
    case 3: {
      instruction = (boost::format("Keep %1% to take exit %2% onto %3%.") % turn
          % exit_number_sign % street_names).str();
      break;
    }
      //  4 "Keep <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 4: {
      instruction = (boost::format("Keep %1% toward %2%.") % turn
          % exit_toward_sign).str();
      break;
    }
      //  5 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>."
    case 5: {
      instruction = (boost::format("Keep %1% to take exit %2% toward %3%.")
          % turn % exit_number_sign % exit_toward_sign).str();
      break;
    }
      //  6 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 6: {
      instruction = (boost::format("Keep %1% to take %2% toward %3%.") % turn
          % street_names % exit_toward_sign).str();
      break;
    }
      //  7 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format(
          "Keep %1% to take exit %2% onto %3% toward %4%.") % turn
          % exit_number_sign % street_names % exit_toward_sign).str();
      break;
    }
      //  0 "Keep <FormTurnTypeInstruction> at the fork."
    default: {
      instruction = (boost::format("Keep %1% at the fork.") % turn).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertKeepInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  //  0 "Keep <FormTurnTypeInstruction> at the fork."
  //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN>."
  //  2 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN(1)>."
  //  4 "Keep <FormTurnTypeInstruction> toward <TOWARD_SIGN(1)>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());

  // If street names string is empty and the maneuver has sign branch info
  // then assign the sign branch name to the street names string  std::string street_name;
  if (street_names.empty() && maneuver.HasExitBranchSign()) {
    street_names = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString(
        0, false, delim, maneuver.verbal_formatter());
  } else if (!street_names.empty()) {
    phrase_id += 2;
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalKeepInstruction(phrase_id, turn, street_names,
                                   exit_number_sign, exit_toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  //  0 "Keep <FormTurnTypeInstruction> at the fork."
  //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN>."
  //  2 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN(2)>."
  //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN(2)>."
  //  4 "Keep <FormTurnTypeInstruction> toward <TOWARD_SIGN(2)>."
  //  5 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN(2)>."
  //  6 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN(2)> toward <TOWARD_SIGN(2)>."
  //  7 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN(2)> toward <TOWARD_SIGN(2)>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());

  // If street names string is empty and the maneuver has sign branch info
  // then assign the sign branch name to the street names string  std::string street_name;
  if (street_names.empty() && maneuver.HasExitBranchSign()) {
    street_names = maneuver.signs().GetExitBranchString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }
  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString(
        0, false, delim, maneuver.verbal_formatter());
  }
  if (!street_names.empty()) {
    phrase_id += 2;
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalKeepInstruction(phrase_id, turn, street_names,
                                   exit_number_sign, exit_toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepInstruction(
    uint8_t phrase_id, const std::string& turn, const std::string& street_names,
    const std::string& exit_number_sign, const std::string& exit_toward_sign) {

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  switch (phrase_id) {
    //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN>."
    case 1: {
      instruction = (boost::format("Keep %1% to take exit %2%.") % turn
          % exit_number_sign).str();
      break;
    }
      //  2 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN>."
    case 2: {
      instruction =
          (boost::format("Keep %1% to take %2%.") % turn % street_names).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN>."
    case 3: {
      instruction = (boost::format("Keep %1% to take exit %2% onto %3%.") % turn
          % exit_number_sign % street_names).str();
      break;
    }
      //  4 "Keep <FormTurnTypeInstruction> toward <TOWARD_SIGN>."
    case 4: {
      instruction = (boost::format("Keep %1% toward %2%.") % turn
          % exit_toward_sign).str();
      break;
    }
      //  5 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>."
    case 5: {
      instruction = (boost::format("Keep %1% to take exit %2% toward %3%.")
          % turn % exit_number_sign % exit_toward_sign).str();
      break;
    }
      //  6 "Keep <FormTurnTypeInstruction> to take <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 6: {
      instruction = (boost::format("Keep %1% to take %2% toward %3%.") % turn
          % street_names % exit_toward_sign).str();
      break;
    }
      //  7 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format(
          "Keep %1% to take exit %2% onto %3% toward %4%.") % turn
          % exit_number_sign % street_names % exit_toward_sign).str();
      break;
    }
      //  0 "Keep <FormTurnTypeInstruction> at the fork."
    default: {
      instruction = (boost::format("Keep %1% at the fork.") % turn).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormKeepToStayOnInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count) {

  //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
  //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>."
  //  2 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
  //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count);
  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString();
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count);
  }

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  switch (phrase_id) {
    //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>."
    case 1: {
      instruction += (boost::format("Keep %1% to take exit %2% to stay on %3%.")
          % turn % exit_number_sign % street_names).str();
      break;
    }
      //  2 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 2: {
      instruction += (boost::format("Keep %1% to stay on %2% toward %3%.")
          % turn % street_names % exit_toward_sign).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 3: {
      instruction += (boost::format(
          "Keep %1% to take exit %2% to stay on %3% toward %4%.") % turn
          % exit_number_sign % street_names % exit_toward_sign).str();
      break;
    }
      //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
    default: {
      instruction += (boost::format("Keep %1% to stay on %2%.") % turn
          % street_names).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertKeepToStayOnInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES(1)>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());

  std::string turn = FormTurnTypeInstruction(maneuver.type());

  return FormVerbalKeepToStayOnInstruction(0, turn, street_names);
}

std::string NarrativeBuilder::FormVerbalKeepToStayOnInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."
  //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES(2)>."
  //  2 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)> toward <TOWARD_SIGN(2)>."
  //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES(2)> toward <TOWARD_SIGN(2)>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());

  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string exit_number_sign;
  std::string exit_toward_sign;
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign()) {
    phrase_id += 1;
    exit_number_sign = maneuver.signs().GetExitNumberString(
        0, false, delim, maneuver.verbal_formatter());
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 2;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalKeepToStayOnInstruction(phrase_id, turn, street_names,
                                           exit_number_sign, exit_toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepToStayOnInstruction(
      uint8_t phrase_id, const std::string& turn,
      const std::string& street_names, const std::string& exit_number_sign,
      const std::string& exit_toward_sign) {

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  switch (phrase_id) {
    //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>."
    case 1: {
      instruction += (boost::format("Keep %1% to take exit %2% to stay on %3%.")
          % turn % exit_number_sign % street_names).str();
      break;
    }
      //  2 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 2: {
      instruction += (boost::format("Keep %1% to stay on %2% toward %3%.")
          % turn % street_names % exit_toward_sign).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 3: {
      instruction += (boost::format(
          "Keep %1% to take exit %2% to stay on %3% toward %4%.") % turn
          % exit_number_sign % street_names % exit_toward_sign).str();
      break;
    }
      //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
    default: {
      instruction += (boost::format("Keep %1% to stay on %2%.") % turn
          % street_names).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormMergeInstruction(Maneuver& maneuver) {
  //  0 "Merge."
  //  1 "Merge onto <STREET_NAMES>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  if (!street_names.empty()) {
    instruction += "Merge onto ";
    instruction += street_names;
  } else
    instruction += "Merge";

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertMergeInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Merge."
  //  1 "Merge onto <STREET_NAMES(1)>."

  return FormVerbalMergeInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalMergeInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Merge."
  //  1 "Merge onto <STREET_NAMES(2)>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  if (!street_names.empty()) {
    instruction += "Merge onto ";
    instruction += street_names;
  } else
    instruction += "Merge";

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormEnterRoundaboutInstruction(Maneuver& maneuver) {
  //  0 "Enter the roundabout."
  //  1 "Enter the roundabout and take the <FormOrdinalValue> exit."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Enter the roundabout";
  if ((maneuver.roundabout_exit_count() > 0)
      && (maneuver.roundabout_exit_count() < 11)) {
    instruction += " and take the ";
    instruction += FormOrdinalValue(maneuver.roundabout_exit_count());
    instruction += " exit";
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertEnterRoundaboutInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Enter roundabout."
  // TODO - determine if we need anything more here

  return "Enter roundabout.";
}

std::string NarrativeBuilder::FormVerbalEnterRoundaboutInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Enter the roundabout."
  //  1 "Enter the roundabout and take the <FormOrdinalValue> exit."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Enter the roundabout";
  if ((maneuver.roundabout_exit_count() > 0)
      && (maneuver.roundabout_exit_count() < 11)) {
    instruction += " and take the ";
    instruction += FormOrdinalValue(maneuver.roundabout_exit_count());
    instruction += " exit";
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormExitRoundaboutInstruction(Maneuver& maneuver) {
  //  0 "Exit the roundabout."
  //  1 "Exit the roundabout onto <STREET_NAMES>."
  //  2 "Exit the roundabout onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Exit the roundabout";

  if (!begin_street_names.empty()) {
    instruction += " onto ";
    instruction += begin_street_names;
    instruction += ". Continue on ";
    instruction += street_names;
  } else if (!street_names.empty()) {
    instruction += " onto ";
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalExitRoundaboutInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Exit the roundabout."
  //  1 "Exit the roundabout onto <BEGIN_STREET_NAMES|STREET_NAMES(2)>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Exit the roundabout";

  if (!begin_street_names.empty()) {
    instruction += " onto ";
    instruction += begin_street_names;
  } else if (!street_names.empty()) {
    instruction += " onto ";
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormEnterFerryInstruction(Maneuver& maneuver) {
  //  0 "Take the Ferry."
  //  1 "Take the <STREET_NAMES>."
  //  2 "Take the <STREET_NAMES> Ferry."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Take the ";
  if (!street_names.empty()) {
    instruction += street_names;
  }

  // TODO - handle properly with locale narrative builder
  std::string ferry_label = " Ferry";
  if (!boost::algorithm::ends_with(instruction, ferry_label)) {
    instruction += ferry_label;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertEnterFerryInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Take the Ferry."
  //  1 "Take the <STREET_NAMES(1)>."
  //  2 "Take the <STREET_NAMES(1)> Ferry."

  return FormVerbalEnterFerryInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalEnterFerryInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Take the Ferry."
  //  1 "Take the <STREET_NAMES(2)>."
  //  2 "Take the <STREET_NAMES(2)> Ferry."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Take the ";
  if (!street_names.empty()) {
    instruction += street_names;
  }

  // TODO - handle properly with locale narrative builder
  std::string ferry_label = " Ferry";
  if (!boost::algorithm::ends_with(instruction, ferry_label)) {
    instruction += ferry_label;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormExitFerryInstruction(Maneuver& maneuver) {
  //  0 "Head <FormCardinalDirection>."
  //  1 "Head <FormCardinalDirection> on <STREET_NAMES>."
  //  2 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Head ";
  instruction += FormCardinalDirection(
      maneuver.begin_cardinal_direction());

  if (!begin_street_names.empty()) {
    instruction += " on ";
    instruction += begin_street_names;
    instruction += ". Continue on ";
    instruction += street_names;
  } else if (!street_names.empty()) {
    instruction += " on ";
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertExitFerryInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Head <FormCardinalDirection>."
  //  1 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES|STREET_NAMES(1)>."

  return FormVerbalExitFerryInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalExitFerryInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Head <FormCardinalDirection>."
  //  1 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES|STREET_NAMES(2)>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Head ";
  instruction += FormCardinalDirection(maneuver.begin_cardinal_direction());

  if (!begin_street_names.empty()) {
    instruction += " on ";
    instruction += begin_street_names;
  } else if (!street_names.empty()) {
    instruction += " on ";
    instruction += street_names;
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormTransitConnectionStartInstruction(
    Maneuver& maneuver) {
  // 0 "Enter station."
  // 1 "Enter the <TRANSIT_CONNECTION_STOP> station."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!maneuver.transit_connection_stop().name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Enter the <TRANSIT_CONNECTION_STOP> station."
    case 1: {
      instruction = (boost::format("Enter the %1% station.")
          % maneuver.transit_connection_stop().name).str();
      break;
    }
    // 0 "Enter station."
    default: {
      instruction = "Enter station.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitConnectionStartInstruction(
    Maneuver& maneuver) {
  // 0 "Enter station."
  // 1 "Enter the <TRANSIT_CONNECTION_STOP> station."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!maneuver.transit_connection_stop().name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Enter the <TRANSIT_CONNECTION_STOP> station."
    case 1: {
      instruction = (boost::format("Enter the %1% station.")
          % maneuver.transit_connection_stop().name).str();
      break;
    }
    // 0 "Enter station."
    default: {
      instruction = "Enter station.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitConnectionTransferInstruction(
    Maneuver& maneuver) {
  // 0 "Transfer at station."
  // 1 "Transfer at the <TRANSIT_CONNECTION_STOP> station."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!maneuver.transit_connection_stop().name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Transfer at the <TRANSIT_CONNECTION_STOP> station."
    case 1: {
      instruction = (boost::format("Transfer at the %1% station.")
          % maneuver.transit_connection_stop().name).str();
      break;
    }
    // 0 "Transfer at station."
    default: {
      instruction = "Transfer at station.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitConnectionTransferInstruction(
    Maneuver& maneuver) {
  // 0 "Transfer at station."
  // 1 "Transfer at the <TRANSIT_CONNECTION_STOP> station."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!maneuver.transit_connection_stop().name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Transfer at the <TRANSIT_CONNECTION_STOP> station."
    case 1: {
      instruction = (boost::format("Transfer at the %1% station.")
          % maneuver.transit_connection_stop().name).str();
      break;
    }
    // 0 "Transfer at station."
    default: {
      instruction = "Transfer at station.";
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitConnectionDestinationInstruction(
    Maneuver& maneuver) {
  // 0 "Exit station."
  // 1 "Exit the <TRANSIT_CONNECTION_STOP> station."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!maneuver.transit_connection_stop().name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Exit the <TRANSIT_CONNECTION_STOP> station."
    case 1: {
      instruction = (boost::format("Exit the %1% station.")
          % maneuver.transit_connection_stop().name).str();
      break;
    }
    // 0 "Exit station."
    default: {
      instruction = "Exit station.";
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormVerbalTransitConnectionDestinationInstruction(
    Maneuver& maneuver) {
  // 0 "Exit station."
  // 1 "Exit the <TRANSIT_CONNECTION_STOP> station."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  if (!maneuver.transit_connection_stop().name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Exit the <TRANSIT_CONNECTION_STOP> station."
    case 1: {
      instruction = (boost::format("Exit the %1% station.")
          % maneuver.transit_connection_stop().name).str();
      break;
    }
    // 0 "Exit station."
    default: {
      instruction = "Exit station.";
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormDepartInstruction(Maneuver& maneuver) {
  // 0 "Depart: <GetFormattedTransitDepartureTime>.
  // 1 "Depart: <GetFormattedTransitDepartureTime> from <FIRST_TRANSIT_STOP>.

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().front().name;

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Depart: <GetFormattedTransitDepartureTime> from <FIRST_TRANSIT_STOP>.
    case 1: {
      instruction =
          (boost::format("Depart: %1% from %2%.")
              % maneuver.GetFormattedTransitDepartureTime() % transit_stop_name)
              .str();
      break;
    }
    // 0 "Depart: <GetFormattedTransitDepartureTime>.
    default: {
      instruction = (boost::format("Depart: %1%.")
          % maneuver.GetFormattedTransitDepartureTime()).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalDepartInstruction(Maneuver& maneuver) {
  // 0 "Depart at <GetFormattedTransitDepartureTime>.
  // 1 "Depart at <GetFormattedTransitDepartureTime> from <FIRST_TRANSIT_STOP>.

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().front().name;

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Depart at <GetFormattedTransitDepartureTime> from <FIRST_TRANSIT_STOP>.
    case 1: {
      instruction =
          (boost::format("Depart at %1% from %2%.")
              % maneuver.GetFormattedTransitDepartureTime() % transit_stop_name)
              .str();
      break;
    }
    // 0 "Depart at <GetFormattedTransitDepartureTime>.
    default: {
      instruction = (boost::format("Depart at %1%.")
          % maneuver.GetFormattedTransitDepartureTime()).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormArriveInstruction(Maneuver& maneuver) {
  // 0 "Arrive: <GetFormattedTransitArrivalTime>.
  // 1 "Arrive: <GetFormattedTransitArrivalTime> at <LAST_TRANSIT_STOP>.

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().back().name;

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Arrive: <GetFormattedTransitArrivalTime> at <LAST_TRANSIT_STOP>.
    case 1: {
      instruction =
          (boost::format("Arrive: %1% at %2%.")
              % maneuver.GetFormattedTransitArrivalTime() % transit_stop_name)
              .str();
      break;
    }
    // 0 "Arrive: <GetFormattedTransitArrivalTime>.
    default: {
      instruction = (boost::format("Arrive: %1%.")
          % maneuver.GetFormattedTransitArrivalTime()).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalArriveInstruction(Maneuver& maneuver) {
  // 0 "Arrive at <GetFormattedTransitArrivalTime>.
  // 1 "Arrive at <GetFormattedTransitArrivalTime> at <LAST_TRANSIT_STOP>.

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_stop_name = maneuver.GetTransitStops().back().name;

  if (!transit_stop_name.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Arrive at <GetFormattedTransitArrivalTime> at <LAST_TRANSIT_STOP>.
    case 1: {
      instruction =
          (boost::format("Arrive at %1% at %2%.")
              % maneuver.GetFormattedTransitArrivalTime() % transit_stop_name)
              .str();
      break;
    }
    // 0 "Arrive at <GetFormattedTransitArrivalTime>.
    default: {
      instruction = (boost::format("Arrive at %1%.")
          % maneuver.GetFormattedTransitArrivalTime()).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitInstruction(
    Maneuver& maneuver) {
  // 0 "Take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
  // 1 "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_route_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
    case 1: {
      instruction = (boost::format("Take the %1% toward %2%. (%3% %4%)")
          % FormTransitName(maneuver) % transit_headsign
          % maneuver.GetTransitStopCount()
          % FormStopCountLabel(maneuver.GetTransitStopCount())).str();
      break;
    }
    // 0 "Take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
    default: {
      instruction = (boost::format("Take the %1%. (%2% %3%)")
          % FormTransitName(maneuver) % maneuver.GetTransitStopCount()
          % FormStopCountLabel(maneuver.GetTransitStopCount())).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalTransitInstruction(Maneuver& maneuver) {
  // 0 "Take the <TRANSIT_NAME>."
  // 1 "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_route_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."
    case 1: {
      instruction = (boost::format("Take the %1% toward %2%.")
          % FormTransitName(maneuver) % transit_headsign).str();
      break;
    }
    // 0 "Take the <TRANSIT_NAME>."
    default: {
      instruction = (boost::format("Take the %1%.") % FormTransitName(maneuver))
          .str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormTransitRemainOnInstruction(
    Maneuver& maneuver) {
  // 0 Remain on the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
  // 1 Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_route_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
    case 1: {
      instruction = (boost::format("Remain on the %1% toward %2%. (%3% %4%)")
          % FormTransitName(maneuver) % transit_headsign
          % maneuver.GetTransitStopCount()
          % FormStopCountLabel(maneuver.GetTransitStopCount())).str();
      break;
    }
    // 0 Remain on the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
    default: {
      instruction = (boost::format("Remain on the %1%. (%2% %3%)")
          % FormTransitName(maneuver) % maneuver.GetTransitStopCount()
          % FormStopCountLabel(maneuver.GetTransitStopCount())).str();
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormVerbalTransitRemainOnInstruction(
    Maneuver& maneuver) {
  // 0 Remain on the <TRANSIT_NAME>."
  // 1 Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_route_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."
    case 1: {
      instruction = (boost::format("Remain on the %1% toward %2%.")
          % FormTransitName(maneuver) % transit_headsign).str();
      break;
    }
    // 0 Remain on the <TRANSIT_NAME>."
    default: {
      instruction = (boost::format("Remain on the %1%.")
          % FormTransitName(maneuver)).str();
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormTransitTransferInstruction(
    Maneuver& maneuver) {
  // 0 "Transfer to take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
  // 1 "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_route_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
    case 1: {
      instruction = (boost::format("Transfer to take the %1% toward %2%. (%3% %4%)")
          % FormTransitName(maneuver) % transit_headsign
          % maneuver.GetTransitStopCount()
          % FormStopCountLabel(maneuver.GetTransitStopCount())).str();
      break;
    }
    // 0 "Transfer to take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <FormStopCountLabel>)"
    default: {
      instruction = (boost::format("Transfer to take the %1%. (%2% %3%)")
          % FormTransitName(maneuver) % maneuver.GetTransitStopCount()
          % FormStopCountLabel(maneuver.GetTransitStopCount())).str();
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormVerbalTransitTransferInstruction(
    Maneuver& maneuver) {
  // 0 "Transfer to take the <TRANSIT_NAME>."
  // 1 "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string transit_headsign = maneuver.transit_route_info().headsign;

  if (!transit_headsign.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>."
    case 1: {
      instruction = (boost::format(
          "Transfer to take the %1% toward %2%.")
          % FormTransitName(maneuver) % transit_headsign).str();
      break;
    }
    // 0 "Transfer to take the <TRANSIT_NAME>."
    default: {
      instruction = (boost::format("Transfer to take the %1%.")
          % FormTransitName(maneuver)).str();
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormPostTransitConnectionDestinationInstruction(
    Maneuver& maneuver) {
  // 0 "Head <FormCardinalDirection>."
  // 1 "Head <FormCardinalDirection> on <STREET_NAMES>."
  // 2 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true);
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string cardinal_direction = FormCardinalDirection(
        maneuver.begin_cardinal_direction());

  if (!begin_street_names.empty() && !street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Head <FormCardinalDirection> on <STREET_NAMES>."
    case 1: {
      instruction = (boost::format("Head %1% on %2%.") % cardinal_direction
          % street_names).str();
      break;
    }
    // 2 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
    case 2: {
      instruction = (boost::format("Head %1% on %2%. Continue on %3%.")
          % cardinal_direction % begin_street_names % street_names).str();
      break;
    }
    // 0 "Head <FormCardinalDirection>."
    default: {
      instruction = (boost::format("Head %1%.") % cardinal_direction).str();
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormVerbalPostTransitConnectionDestinationInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  // 0 "Head <FormCardinalDirection>."
  // 1 "Head <FormCardinalDirection> on <STREET_NAMES>."
  // 2 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  // Assign the street names and the begin street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());
  std::string begin_street_names = FormStreetNames(
      maneuver, maneuver.begin_street_names(), false, element_max_count, delim,
      maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  std::string cardinal_direction = FormCardinalDirection(
        maneuver.begin_cardinal_direction());

  if (!begin_street_names.empty() && !street_names.empty()) {
    phrase_id = 2;
  } else if (!street_names.empty()) {
    phrase_id = 1;
  }

  switch (phrase_id) {
    // 1 "Head <FormCardinalDirection> on <STREET_NAMES>."
    case 1: {
      instruction = (boost::format("Head %1% on %2%.") % cardinal_direction
          % street_names).str();
      break;
    }
    // 2 "Head <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
    case 2: {
      instruction = (boost::format("Head %1% on %2%. Continue on %3%.")
          % cardinal_direction % begin_street_names % street_names).str();
      break;
    }
    // 0 "Head <FormCardinalDirection>."
    default: {
      instruction = (boost::format("Head %1%.") % cardinal_direction).str();
      break;
    }
  }

  return instruction;

}

std::string NarrativeBuilder::FormVerbalPostTransitionInstruction(
    Maneuver& maneuver, DirectionsOptions_Units units,
    bool include_street_names, uint32_t element_max_count,
    std::string delim) {
  // "Continue for <DISTANCE>."
  // "Continue on <STREET_NAMES(2)> for <DISTANCE>."

  // Assign the street names
  std::string street_names = FormStreetNames(maneuver, maneuver.street_names(),
                                             true, element_max_count, delim,
                                             maneuver.verbal_formatter());

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (include_street_names && !street_names.empty()) {
    instruction += " on ";
    instruction += street_names;
  }
  instruction += " for ";
  instruction += FormDistance(maneuver, units);
  instruction += ".";

  return instruction;
}

std::string NarrativeBuilder::FormVerbalPostTransitionTransitInstruction(
    Maneuver& maneuver) {
  // 0 "Travel <TRANSIT_STOP_COUNT> <FormStopCountLabel>"

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;

  switch (phrase_id) {
    // 0 "Travel <TRANSIT_STOP_COUNT> <FormStopCountLabel>"
    default: {
      instruction = (boost::format("Travel %1% %2%.")
          % maneuver.GetTransitStopCount()
          % FormStopCountLabel(maneuver.GetTransitStopCount())).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormDistance(
    Maneuver& maneuver, DirectionsOptions_Units units) {
  switch (units) {
    case DirectionsOptions_Units_kMiles: {
      return FormMiles(maneuver.length(DirectionsOptions_Units_kMiles));
    }
    default: {
      return FormKilometers(maneuver.length(DirectionsOptions_Units_kKilometers));
    }
  }
}

std::string NarrativeBuilder::FormKilometers(float kilometers) {

  // 0 "1.2 kilometers"
  // 1 "1 kilometer"
  // 2 "a half kilometer" (500 meters)
  // 3 "100 meters" (30-400 and 600-900 meters)
  // 4 "less than 10 meters"

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  float kilometer_tenths = std::round(kilometers * 10) / 10;

  if (kilometer_tenths > 1.0f) {
    instruction += (boost::format("%.1f kilometers") % kilometer_tenths).str();
  } else if (kilometer_tenths == 1.0f) {
    instruction += "1 kilometer";
  } else if (kilometer_tenths == 0.5f) {
    instruction += "a half kilometer";
  } else {
    float kilometer_hundredths = std::round(kilometers * 100) / 100;
    float kilometer_thousandths = std::round(kilometers * 1000) / 1000;

    // Process hundred meters
    if (kilometer_hundredths > 0.09f) {
      instruction += (boost::format("%d meters")
          % static_cast<uint32_t>(kilometer_tenths * 1000)).str();
    } else if (kilometer_thousandths > 0.009f) {
      instruction += (boost::format("%d meters")
          % static_cast<uint32_t>(kilometer_hundredths * 1000)).str();
    } else {
      instruction += "less than 10 meters";
    }

  }

  return instruction;
}

std::string NarrativeBuilder::FormMiles(float miles) {

  // 0  "1.2 miles"
  // 1  "1 mile."
  // 2  "a half mile."
  // 3  "2 tenths of a mile." (2-4, 6-9)
  // 4  "1 tenth of a mile."
  // 5  "200 feet." (10-90, 100-500)
  // 6  "less than 10 feet."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  float mile_tenths = std::round(miles * 10) / 10;

  if (mile_tenths > 1.0f) {
    instruction += (boost::format("%.1f miles") % mile_tenths).str();
  } else if (mile_tenths == 1.0f) {
    instruction += "1 mile";
  } else if (mile_tenths == 0.5f) {
    instruction += "a half mile";
  } else if (mile_tenths > 0.1f) {
      instruction += (boost::format("%d tenths of a mile")
          % static_cast<uint32_t>(mile_tenths * 10)).str();
  } else if ((miles > 0.0973f) && (mile_tenths == 0.1f)) {
    instruction += "1 tenth of a mile";
  } else {
    uint32_t feet = static_cast<uint32_t>(std::round(miles * 5280));
    if (feet > 94) {
      instruction += (boost::format("%d feet")
          % static_cast<uint32_t>(std::round(feet / 100.0f) * 100)).str();
    } else if (feet > 9) {
      instruction += (boost::format("%d feet")
          % static_cast<uint32_t>(std::round(feet / 10.0f) * 10)).str();
    } else {
      instruction += "less than 10 feet";
    }

  }
  return instruction;
}

std::string NarrativeBuilder::FormCardinalDirection(
    TripDirections_Maneuver_CardinalDirection cardinal_direction) {
  switch (cardinal_direction) {
    case TripDirections_Maneuver_CardinalDirection_kNorth: {
      return "north";
    }
    case TripDirections_Maneuver_CardinalDirection_kNorthEast: {
      return "northeast";
    }
    case TripDirections_Maneuver_CardinalDirection_kEast: {
      return "east";
    }
    case TripDirections_Maneuver_CardinalDirection_kSouthEast: {
      return "southeast";
    }
    case TripDirections_Maneuver_CardinalDirection_kSouth: {
      return "south";
    }
    case TripDirections_Maneuver_CardinalDirection_kSouthWest: {
      return "southwest";
    }
    case TripDirections_Maneuver_CardinalDirection_kWest: {
      return "west";
    }
    case TripDirections_Maneuver_CardinalDirection_kNorthWest: {
      return "northwest";
    }
    default: {
      throw std::runtime_error(
          "Invalid TripDirections_Maneuver_CardinalDirection in method FormCardinalDirection.");
    }
  }
}

std::string NarrativeBuilder::FormTurnTypeInstruction(
    TripDirections_Maneuver_Type type) {
  switch (type) {
    case TripDirections_Maneuver_Type_kSlightRight:
    case TripDirections_Maneuver_Type_kRight:
    case TripDirections_Maneuver_Type_kUturnRight:
    case TripDirections_Maneuver_Type_kRampRight:
    case TripDirections_Maneuver_Type_kExitRight:
    case TripDirections_Maneuver_Type_kStayRight:
    case TripDirections_Maneuver_Type_kDestinationRight: {
      return "right";
    }
    case TripDirections_Maneuver_Type_kSharpRight: {
      return "sharp right";
    }
    case TripDirections_Maneuver_Type_kSharpLeft: {
      return "sharp left";
    }
    case TripDirections_Maneuver_Type_kSlightLeft:
    case TripDirections_Maneuver_Type_kLeft:
    case TripDirections_Maneuver_Type_kUturnLeft:
    case TripDirections_Maneuver_Type_kRampLeft:
    case TripDirections_Maneuver_Type_kExitLeft:
    case TripDirections_Maneuver_Type_kStayLeft:
    case TripDirections_Maneuver_Type_kDestinationLeft: {
      return "left";
    }
    case TripDirections_Maneuver_Type_kStayStraight: {
      return "straight";
    }
    default: {
      throw std::runtime_error(
          "Invalid TripDirections_Maneuver_Type in method FormTurnTypeInstruction.");
    }
  }

}

std::string NarrativeBuilder::FormOrdinalValue(uint32_t value) {
  switch (value) {
    case 1: {
      return "1st";
    }
    case 2: {
      return "2nd";
    }
    case 3: {
      return "3rd";
    }
    case 4: {
      return "4th";
    }
    case 5: {
      return "5th";
    }
    case 6: {
      return "6th";
    }
    case 7: {
      return "7th";
    }
    case 8: {
      return "8th";
    }
    case 9: {
      return "9th";
    }
    case 10: {
      return "10th";
    }
    default: {
      return "undefined";
    }
  }

}

std::string NarrativeBuilder::FormStopCountLabel(size_t stop_count) {
  switch (stop_count) {
    case 1: {
      return "stop";
    }
    default: {
      return "stops";
    }
  }
}

std::string NarrativeBuilder::FormTransitName(Maneuver& maneuver) {
  if (!maneuver.transit_route_info().short_name.empty()) {
    return maneuver.transit_route_info().short_name;
  } else if (!maneuver.transit_route_info().long_name.empty()) {
    return (maneuver.transit_route_info().long_name);
  } else if (maneuver.bus()) {
    return "bus";
  }
  return "train";
}

std::string NarrativeBuilder::FormStreetNames(
    const Maneuver& maneuver, const StreetNames& street_names,
    bool enhance_blank_street_names, uint32_t max_count, std::string delim,
    const VerbalTextFormatter* verbal_formatter) {
  std::string street_names_string;

  // Verify that the street name list is not empty
  if (!street_names.empty()) {
    street_names_string = street_names.ToString(max_count, delim,
                                                verbal_formatter);
  }

  // If empty street names string
  // then determine if walkway or bike path
  if (enhance_blank_street_names && street_names_string.empty()) {

    // If pedestrian travel mode on unnamed footway
    // then set street names string to walkway
    if ((maneuver.travel_mode() ==  TripPath_TravelMode_kPedestrian)
        && maneuver.unnamed_walkway()) {
      street_names_string = "walkway";
    }

    // If bicycle travel mode on unnamed cycleway
    // then set street names string to cycleway
    if ((maneuver.travel_mode() == TripPath_TravelMode_kBicycle)
        && maneuver.unnamed_cycleway()) {
      street_names_string = "cycleway";
    }

    // If bicycle travel mode on unnamed mountain bike trail
    // then set street names string to mountain bike trail
    if ((maneuver.travel_mode() == TripPath_TravelMode_kBicycle)
        && maneuver.unnamed_mountain_bike_trail()) {
      street_names_string = "mountain bike trail";
    }
  }

  return street_names_string;
}

}
}

