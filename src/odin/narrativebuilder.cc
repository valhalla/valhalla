#include <iostream>
#include <cmath>
#include <valhalla/midgard/logging.h>

#include "odin/narrativebuilder.h"
#include "odin/enhancedtrippath.h"
#include "odin/maneuver.h"

#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>

namespace {
// Text instruction initial capacity
constexpr auto kTextInstructionInitialCapacity = 128;
}

namespace valhalla {
namespace odin {

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
            std::move(FormVerbalStartInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units(),
                    maneuver.HasBeginStreetNames())));
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
      case TripDirections_Maneuver_Type_kContinue: {
        // Set instruction
        maneuver.set_instruction(std::move(FormContinueInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertContinueInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalContinueInstruction(maneuver)));

        // Set verbal post transition instruction
        maneuver.set_verbal_post_transition_instruction(
            std::move(
                FormVerbalPostTransitionInstruction(
                    maneuver, directions_options.units())));
        break;
      }
      case TripDirections_Maneuver_Type_kSlightRight:
      case TripDirections_Maneuver_Type_kSlightLeft: {
        if (maneuver.HasSimilarNames(prev_maneuver, true)) {
          // Set stay on instruction
          maneuver.set_instruction(
              std::move(FormBearToStayOnInstruction(maneuver)));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              std::move(FormVerbalAlertBearToStayOnInstruction(maneuver)));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(
              std::move(FormVerbalBearToStayOnInstruction(maneuver)));

          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units())));
        } else {
          FormBearInstruction(maneuver);
          // Set instruction
          maneuver.set_instruction(std::move(FormBearInstruction(maneuver)));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              std::move(FormVerbalAlertBearInstruction(maneuver)));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(
              std::move(FormVerbalBearInstruction(maneuver)));

          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units(),
                      maneuver.HasBeginStreetNames())));
        }
        break;
      }
      case TripDirections_Maneuver_Type_kRight:
      case TripDirections_Maneuver_Type_kSharpRight:
      case TripDirections_Maneuver_Type_kSharpLeft:
      case TripDirections_Maneuver_Type_kLeft: {
        if (maneuver.HasSimilarNames(prev_maneuver, true)) {
          // Set stay on instruction
          maneuver.set_instruction(
              std::move(FormTurnToStayOnInstruction(maneuver)));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              std::move(FormVerbalAlertTurnToStayOnInstruction(maneuver)));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(
              std::move(FormVerbalTurnToStayOnInstruction(maneuver)));

          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units())));
        } else {
          // Set instruction
          maneuver.set_instruction(std::move(FormTurnInstruction(maneuver)));

          // Set verbal transition alert instruction
          maneuver.set_verbal_transition_alert_instruction(
              std::move(FormVerbalAlertTurnInstruction(maneuver)));

          // Set verbal pre transition instruction
          maneuver.set_verbal_pre_transition_instruction(
              std::move(FormVerbalTurnInstruction(maneuver)));

          // Set verbal post transition instruction
          maneuver.set_verbal_post_transition_instruction(
              std::move(
                  FormVerbalPostTransitionInstruction(
                      maneuver, directions_options.units(),
                      maneuver.HasBeginStreetNames())));
        }
        break;
      }
      case TripDirections_Maneuver_Type_kUturnRight:
      case TripDirections_Maneuver_Type_kUturnLeft: {
        // Set instruction
        maneuver.set_instruction(
            std::move(FormUturnInstruction(maneuver)));

        // Set verbal transition alert instruction
        maneuver.set_verbal_transition_alert_instruction(
            std::move(FormVerbalAlertUturnInstruction(maneuver)));

        // Set verbal pre transition instruction
        maneuver.set_verbal_pre_transition_instruction(
            std::move(FormVerbalUturnInstruction(maneuver)));

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
        FormTransitConnectionStartInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kTransitConnectionTransfer: {
        FormTransitConnectionTransferInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kTransitConnectionDestination: {
        FormTransitConnectionDestinationInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kTransit: {
        FormTransitInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kTransitRemainOn: {
        FormTransitRemainOnInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kTransitTransfer: {
        FormTransitTransferInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kPostTransitConnectionDestination: {
        FormPostTransitConnectionDestinationInstruction(maneuver);
        break;
      }
      default: {
        FormContinueInstruction(maneuver);
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
  // 0 "Go <FormCardinalDirection>."
  // 1 "Go <FormCardinalDirection> on <STREET_NAMES>."
  // 2 "Go <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  std::string cardinal_direction = FormCardinalDirection(
      maneuver.begin_cardinal_direction());
  std::string street_names;
  std::string begin_street_names;
  uint8_t phrase_id = 0;

  if (maneuver.HasStreetNames()) {
    phrase_id += 1;
    street_names = maneuver.street_names().ToString();
  }
  if (maneuver.HasBeginStreetNames()) {
    phrase_id += 1;
    begin_street_names = maneuver.begin_street_names().ToString();
  }

  switch (phrase_id) {
    // 1 "Go <FormCardinalDirection> on <STREET_NAMES>."
    case 1: {
      instruction = (boost::format("Go %1% on %2%.")
          % cardinal_direction % street_names).str();
      break;
    }
    // 2 "Go <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
    case 2: {
      instruction = (boost::format("Go %1% on %2%. Continue on %3%.")
          % cardinal_direction % begin_street_names % street_names).str();
      break;
    }
    // 0 "Go <FormCardinalDirection>."
    default: {
      instruction = (boost::format("Go %1%.") % cardinal_direction).str();
      break;
    }
  }
  // TODO - side of street

  return instruction;
}

std::string NarrativeBuilder::FormVerbalStartInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  // 0 "Go <FormCardinalDirection>."
  // 1 "Go <FormCardinalDirection> on <STREET_NAMES|BEGIN_STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  std::string cardinal_direction = FormCardinalDirection(
      maneuver.begin_cardinal_direction());
  std::string street_names;
  uint8_t phrase_id = 0;

  if (maneuver.HasBeginStreetNames()) {
    phrase_id = 1;
    street_names = maneuver.begin_street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasStreetNames()) {
    phrase_id = 1;
    street_names = maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  switch (phrase_id) {
    // 1 "Go <FormCardinalDirection> on <STREET_NAMES|BEGIN_STREET_NAMES>."
    case 1: {
      instruction = (boost::format("Go %1% on %2%.")
          % cardinal_direction % street_names).str();
      break;
    }
    // 0 "Go <FormCardinalDirection>."
    default: {
      instruction = (boost::format("Go %1%.") % cardinal_direction).str();
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

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // If previous maneuver has names
  // and current maneuver has names
  // then form "becomes" narrative
  if (prev_maneuver && prev_maneuver->HasStreetNames()
      && maneuver.HasStreetNames()) {
    instruction += prev_maneuver->street_names().ToString();
    instruction += " becomes ";
    instruction += maneuver.street_names().ToString();
  }
  // Items are missing - fallback to just "Continue" narrative
  else {
    instruction += "Continue";

    if (maneuver.HasStreetNames()) {
      instruction += " on ";
      instruction += maneuver.street_names().ToString();
    }
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalBecomesInstruction(
    Maneuver& maneuver, Maneuver* prev_maneuver, uint32_t element_max_count,
    std::string delim) {
  // "<PREV_STREET_NAMES(2)> becomes <STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  // If previous maneuver has names
  // and current maneuver has names
  // then form "becomes" narrative
  if (prev_maneuver && prev_maneuver->HasStreetNames()
      && maneuver.HasStreetNames()) {
    instruction += prev_maneuver->street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
    instruction += " becomes ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }
  // Items are missing - fallback to just "Continue" narrative
  else {
    instruction += "Continue";

    if (maneuver.HasStreetNames()) {
      instruction += " on ";
      instruction += maneuver.street_names().ToString(
          element_max_count, delim, maneuver.verbal_formatter());
    }
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormContinueInstruction(Maneuver& maneuver) {
  // "Continue"
  // "Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (maneuver.HasStreetNames()) {
    instruction += " on ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertContinueInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Continue"
  //  "Continue on <STREET_NAMES(1)>."
  return FormVerbalContinueInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalContinueInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Continue"
  //  "Continue on <STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (maneuver.HasStreetNames()) {
    instruction += " on ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormTurnInstruction(Maneuver& maneuver) {
  //  "Turn <FormTurnTypeInstruction>."
  //  "Turn <FormTurnTypeInstruction> onto <STREET_NAMES>."
  //  "Turn <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Turn ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasBeginStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.begin_street_names().ToString();
    instruction += ". Continue on ";
    instruction += maneuver.street_names().ToString();
  } else if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertTurnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
    //    "Turn <FormTurnTypeInstruction>."
    //    "Turn <FormTurnTypeInstruction> onto <STREET_NAMES(1)|BEGIN_STREET_NAMES(1)>."

  return FormVerbalTurnInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalTurnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Turn <FormTurnTypeInstruction>."
  //  "Turn <FormTurnTypeInstruction> onto <STREET_NAMES(2)|BEGIN_STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Turn ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasBeginStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.begin_street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormTurnToStayOnInstruction(Maneuver& maneuver) {
  // "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Turn ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasStreetNames()) {
    instruction += " to stay on ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertTurnToStayOnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  // "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."

  return FormVerbalTurnToStayOnInstruction(maneuver, element_max_count,
                                           delim);
}

std::string NarrativeBuilder::FormVerbalTurnToStayOnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  // "Turn <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Turn ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasStreetNames()) {
    instruction += " to stay on ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormBearInstruction(Maneuver& maneuver) {
  //  "Bear <FormTurnTypeInstruction>."
  //  "Bear <FormTurnTypeInstruction> onto <STREET_NAMES>."
  //  "Bear <FormTurnTypeInstruction> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Bear ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasBeginStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.begin_street_names().ToString();
    instruction += ". Continue on ";
    instruction += maneuver.street_names().ToString();
  } else if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertBearInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Bear <FormTurnTypeInstruction>."
  //  "Bear <FormTurnTypeInstruction> onto <STREET_NAMES(1)|BEGIN_STREET_NAMES(1)>."

  return FormVerbalBearInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalBearInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Bear <FormTurnTypeInstruction>."
  //  "Bear <FormTurnTypeInstruction> onto <STREET_NAMES(2)|BEGIN_STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Bear ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasBeginStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.begin_street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormBearToStayOnInstruction(Maneuver& maneuver) {
  // "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Bear ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasStreetNames()) {
    instruction += " to stay on ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertBearToStayOnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  // "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES(1)>."

  return FormVerbalBearToStayOnInstruction(maneuver, element_max_count,
                                           delim);
}

std::string NarrativeBuilder::FormVerbalBearToStayOnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  // "Bear <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Bear ";
  instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasStreetNames()) {
    instruction += " to stay on ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormUturnInstruction(Maneuver& maneuver) {
  //  "Make a <FormTurnTypeInstruction> U-turn."
  //  "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES>."
  //  "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>."
  //  "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Make a ";
  instruction += FormTurnTypeInstruction(maneuver.type());
  instruction += " U-turn";

  if (maneuver.HasCrossStreetNames()) {
    instruction += " at ";
    instruction += maneuver.cross_street_names().ToString();
  }

  if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertUturnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Make a <FormTurnTypeInstruction> U-turn."
  //  "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(1)>."
  //  "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES(1)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Make a ";
  instruction += FormTurnTypeInstruction(maneuver.type());
  instruction += " U-turn";

  if (maneuver.HasCrossStreetNames()) {
    instruction += " at ";
    instruction += maneuver.cross_street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalUturnInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  "Make a <FormTurnTypeInstruction> U-turn."
  //  "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)>."
  //  "Make a <FormTurnTypeInstruction> U-turn at <CROSS_STREET_NAMES(2)> onto <STREET_NAMES(2)>."
  //  "Make a <FormTurnTypeInstruction> U-turn onto <STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Make a ";
  instruction += FormTurnTypeInstruction(maneuver.type());
  instruction += " U-turn";

  if (maneuver.HasCrossStreetNames()) {
    instruction += " at ";
    instruction += maneuver.cross_street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
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

  // Assign maneuver street name or sign branch name
  std::string street_name;
  if (maneuver.HasStreetNames()) {
    street_name = maneuver.street_names().ToString(element_max_count);
  } else if (maneuver.HasExitBranchSign()) {
    street_name = maneuver.signs().GetExitBranchString(
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
  if (!street_name.empty()) {
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
          (boost::format("Keep %1% to take %2%.") % turn % street_name).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN>."
    case 3: {
      instruction = (boost::format("Keep %1% to take exit %2% onto %3%.") % turn
          % exit_number_sign % street_name).str();
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
          % street_name % exit_toward_sign).str();
      break;
    }
      //  7 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format(
          "Keep %1% to take exit %2% onto %3% toward %4%.") % turn
          % exit_number_sign % street_name % exit_toward_sign).str();
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

  // Assign maneuver street name or sign branch name
  std::string street_name;
  if (maneuver.HasStreetNames()) {
    street_name = maneuver.street_names().ToString(element_max_count, delim,
                                                   maneuver.verbal_formatter());
  } else if (maneuver.HasExitBranchSign()) {
    street_name = maneuver.signs().GetExitBranchString(
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
  } else if (!street_name.empty()) {
    phrase_id += 2;
  } else if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalKeepInstruction(phrase_id, turn, street_name,
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

  // Assign maneuver street name or sign branch name
  std::string street_name;
  if (maneuver.HasStreetNames()) {
    street_name = maneuver.street_names().ToString(element_max_count, delim,
                                                   maneuver.verbal_formatter());
  } else if (maneuver.HasExitBranchSign()) {
    street_name = maneuver.signs().GetExitBranchString(
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
  if (!street_name.empty()) {
    phrase_id += 2;
  }
  if (maneuver.HasExitTowardSign()) {
    phrase_id += 4;
    exit_toward_sign = maneuver.signs().GetExitTowardString(
        element_max_count, limit_by_consecutive_count, delim,
        maneuver.verbal_formatter());
  }

  return FormVerbalKeepInstruction(phrase_id, turn, street_name,
                                   exit_number_sign, exit_toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepInstruction(
    uint8_t phrase_id, const std::string& turn, const std::string& street_name,
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
          (boost::format("Keep %1% to take %2%.") % turn % street_name).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN>."
    case 3: {
      instruction = (boost::format("Keep %1% to take exit %2% onto %3%.") % turn
          % exit_number_sign % street_name).str();
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
          % street_name % exit_toward_sign).str();
      break;
    }
      //  7 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> onto <STREET_NAMES OR BRANCH_SIGN> toward <TOWARD_SIGN>."
    case 7: {
      instruction = (boost::format(
          "Keep %1% to take exit %2% onto %3% toward %4%.") % turn
          % exit_number_sign % street_name % exit_toward_sign).str();
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

  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string street_name = maneuver.street_names().ToString(element_max_count);
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
          % turn % exit_number_sign % street_name).str();
      break;
    }
      //  2 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 2: {
      instruction += (boost::format("Keep %1% to stay on %2% toward %3%.")
          % turn % street_name % exit_toward_sign).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 3: {
      instruction += (boost::format(
          "Keep %1% to take exit %2% to stay on %3% toward %4%.") % turn
          % exit_number_sign % street_name % exit_toward_sign).str();
      break;
    }
      //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
    default: {
      instruction += (boost::format("Keep %1% to stay on %2%.") % turn
          % street_name).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertKeepToStayOnInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES(1)>."

  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string street_name = maneuver.street_names().ToString(
      element_max_count, delim, maneuver.verbal_formatter());
return FormVerbalKeepToStayOnInstruction(0, turn, street_name);
}

std::string NarrativeBuilder::FormVerbalKeepToStayOnInstruction(
    Maneuver& maneuver, bool limit_by_consecutive_count,
    uint32_t element_max_count, std::string delim) {

  //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)>."
  //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES(2)>."
  //  2 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES(2)> toward <TOWARD_SIGN(2)>."
  //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES(2)> toward <TOWARD_SIGN(2)>."

  std::string turn = FormTurnTypeInstruction(maneuver.type());
  std::string street_name = maneuver.street_names().ToString(
      element_max_count, delim, maneuver.verbal_formatter());
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

  return FormVerbalKeepToStayOnInstruction(phrase_id, turn, street_name,
                                           exit_number_sign, exit_toward_sign);
}

std::string NarrativeBuilder::FormVerbalKeepToStayOnInstruction(
      uint8_t phrase_id, const std::string& turn,
      const std::string& street_name, const std::string& exit_number_sign,
      const std::string& exit_toward_sign) {

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  switch (phrase_id) {
    //  1 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>."
    case 1: {
      instruction += (boost::format("Keep %1% to take exit %2% to stay on %3%.")
          % turn % exit_number_sign % street_name).str();
      break;
    }
      //  2 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 2: {
      instruction += (boost::format("Keep %1% to stay on %2% toward %3%.")
          % turn % street_name % exit_toward_sign).str();
      break;
    }
      //  3 "Keep <FormTurnTypeInstruction> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."
    case 3: {
      instruction += (boost::format(
          "Keep %1% to take exit %2% to stay on %3% toward %4%.") % turn
          % exit_number_sign % street_name % exit_toward_sign).str();
      break;
    }
      //  0 "Keep <FormTurnTypeInstruction> to stay on <STREET_NAMES>."
    default: {
      instruction += (boost::format("Keep %1% to stay on %2%.") % turn
          % street_name).str();
      break;
    }
  }

  return instruction;
}

std::string NarrativeBuilder::FormMergeInstruction(Maneuver& maneuver) {
  //  0 "Merge."
  //  1 "Merge onto <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  if (maneuver.HasStreetNames()) {
    instruction += "Merge onto ";
    instruction += maneuver.street_names().ToString();
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

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);

  if (maneuver.HasStreetNames()) {
    instruction += "Merge onto ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
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

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Exit the roundabout";

  if (maneuver.HasBeginStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.begin_street_names().ToString();
    instruction += ". Continue on ";
    instruction += maneuver.street_names().ToString();
  } else if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalExitRoundaboutInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Exit the roundabout."
  //  1 "Exit the roundabout onto <BEGIN_STREET_NAMES|STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Exit the roundabout";

  if (maneuver.HasBeginStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.begin_street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasStreetNames()) {
    instruction += " onto ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormEnterFerryInstruction(Maneuver& maneuver) {
//  0 "Take the Ferry."
//  1 "Take the <STREET_NAMES>."
//  2 "Take the <STREET_NAMES> Ferry."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Take the ";
  if (maneuver.HasStreetNames()) {
    instruction += maneuver.street_names().ToString();
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

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Take the ";
  if (maneuver.HasStreetNames()) {
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
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
  //  0 "Go <FormCardinalDirection>."
  //  1 "Go <FormCardinalDirection> on <STREET_NAMES>."
  //  2 "Go <FormCardinalDirection> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Go ";
  instruction += FormCardinalDirection(
      maneuver.begin_cardinal_direction());

  if (maneuver.HasBeginStreetNames()) {
    instruction += " on ";
    instruction += maneuver.begin_street_names().ToString();
    instruction += ". Continue on ";
    instruction += maneuver.street_names().ToString();
  } else if (maneuver.HasStreetNames()) {
    instruction += " on ";
    instruction += maneuver.street_names().ToString();
  }

  instruction += ".";
  return instruction;
}

std::string NarrativeBuilder::FormVerbalAlertExitFerryInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Go <FormCardinalDirection>."
  //  1 "Go <FormCardinalDirection> on <BEGIN_STREET_NAMES|STREET_NAMES(1)>."

  return FormVerbalExitFerryInstruction(maneuver, element_max_count, delim);
}

std::string NarrativeBuilder::FormVerbalExitFerryInstruction(
    Maneuver& maneuver, uint32_t element_max_count, std::string delim) {
  //  0 "Go <FormCardinalDirection>."
  //  1 "Go <FormCardinalDirection> on <BEGIN_STREET_NAMES|STREET_NAMES(2)>."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Go ";
  instruction += FormCardinalDirection(maneuver.begin_cardinal_direction());

  if (maneuver.HasBeginStreetNames()) {
    instruction += " on ";
    instruction += maneuver.begin_street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  } else if (maneuver.HasStreetNames()) {
    instruction += " on ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }

  instruction += ".";
  return instruction;
}

void NarrativeBuilder::FormTransitConnectionStartInstruction(
    Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Enter";
  if (!maneuver.transit_connection_stop().name.empty()) {
    text_instruction += " the ";
    text_instruction += maneuver.transit_connection_stop().name;
  }
  text_instruction += " station.";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormTransitConnectionTransferInstruction(
    Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Transfer at";
  if (!maneuver.transit_connection_stop().name.empty()) {
    text_instruction += " the ";
    text_instruction += maneuver.transit_connection_stop().name;
  }
  text_instruction += " station.";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormTransitConnectionDestinationInstruction(
    Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Exit";
  if (!maneuver.transit_connection_stop().name.empty()) {
    text_instruction += " the ";
    text_instruction += maneuver.transit_connection_stop().name;
  }
  text_instruction += " station.";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormTransitInstruction(
    Maneuver& maneuver) {
  // TODO - refactor transit instructions
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "DEPART: ";
  text_instruction += maneuver.GetFormattedTransitDepartureTime();
  text_instruction += " from ";
  text_instruction += maneuver.GetTransitStops().front().name;
  text_instruction += ". Take the ";
  text_instruction += maneuver.GetTransitName();
  if (!maneuver.transit_headsign().empty()) {
    text_instruction += " toward ";
    text_instruction += maneuver.transit_headsign();
  }
  text_instruction += ". (";
  text_instruction += std::to_string(maneuver.GetTransitStopCount());
  if (maneuver.GetTransitStopCount() > 1) {
    text_instruction += " stops";
  } else {
    text_instruction += " stop";
  }
  text_instruction += ").";
  text_instruction += " ARRIVE: ";
  text_instruction += maneuver.GetFormattedTransitArrivalTime();
  text_instruction += " at ";
  text_instruction += maneuver.GetTransitStops().back().name;
  text_instruction += ".";

  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormTransitRemainOnInstruction(
    Maneuver& maneuver) {
  // TODO - refactor transit instructions
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "DEPART: ";
  text_instruction += maneuver.GetFormattedTransitDepartureTime();
  text_instruction += " from ";
  text_instruction += maneuver.GetTransitStops().front().name;
  text_instruction += ". Remain on the ";
  text_instruction += maneuver.GetTransitName();
  if (!maneuver.transit_headsign().empty()) {
    text_instruction += " toward ";
    text_instruction += maneuver.transit_headsign();
  }
  text_instruction += ". (";
  text_instruction += std::to_string(maneuver.GetTransitStopCount());
  if (maneuver.GetTransitStopCount() > 1) {
    text_instruction += " stops";
  } else {
    text_instruction += " stop";
  }
  text_instruction += ").";
  text_instruction += " ARRIVE: ";
  text_instruction += maneuver.GetFormattedTransitArrivalTime();
  text_instruction += " at ";
  text_instruction += maneuver.GetTransitStops().back().name;
  text_instruction += ".";

  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormTransitTransferInstruction(
    Maneuver& maneuver) {
  // TODO - refactor transit instructions
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "DEPART: ";
  text_instruction += maneuver.GetFormattedTransitDepartureTime();
  text_instruction += " from ";
  text_instruction += maneuver.GetTransitStops().front().name;
  text_instruction += ". Transfer to take the ";
  text_instruction += maneuver.GetTransitName();
  if (!maneuver.transit_headsign().empty()) {
    text_instruction += " toward ";
    text_instruction += maneuver.transit_headsign();
  }
  text_instruction += ". (";
  text_instruction += std::to_string(maneuver.GetTransitStopCount());
  if (maneuver.GetTransitStopCount() > 1) {
    text_instruction += " stops";
  } else {
    text_instruction += " stop";
  }
  text_instruction += ").";
  text_instruction += " ARRIVE: ";
  text_instruction += maneuver.GetFormattedTransitArrivalTime();
  text_instruction += " at ";
  text_instruction += maneuver.GetTransitStops().back().name;
  text_instruction += ".";


  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormPostTransitConnectionDestinationInstruction(
    Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Go ";
  text_instruction += FormCardinalDirection(
      maneuver.begin_cardinal_direction());

  if (maneuver.HasBeginStreetNames()) {
    text_instruction += " on ";
    text_instruction += maneuver.begin_street_names().ToString();
    text_instruction += ". Continue on ";
    text_instruction += maneuver.street_names().ToString();
  } else if (maneuver.HasStreetNames()) {
    text_instruction += " on ";
    text_instruction += maneuver.street_names().ToString();
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

std::string NarrativeBuilder::FormVerbalPostTransitionInstruction(
    Maneuver& maneuver, DirectionsOptions_Units units,
    bool include_street_names, uint32_t element_max_count,
    std::string delim) {
  switch (units) {
    case DirectionsOptions_Units_kMiles: {
      return FormVerbalPostTransitionMilesInstruction(
          maneuver, include_street_names, element_max_count, delim);
    }
    default: {
      return FormVerbalPostTransitionKilometersInstruction(
          maneuver, include_street_names, element_max_count, delim);
    }
  }
}

std::string NarrativeBuilder::FormVerbalPostTransitionKilometersInstruction(
    Maneuver& maneuver, bool include_street_names,
    uint32_t element_max_count, std::string delim) {

  // 0 "Continue for 1.2 kilometers"
  // 1 "Continue for 1 kilometer."
  // 2 "Continue for a half kilometer." (500 meters)
  // 3 "Continue for 100 meters." (30-400 and 600-900 meters)
  // 4 "Continue for less than 10 meters."
  //
  // 5 "Continue on <STREET_NAMES(2)> for 1.2 kilometers"
  // 6 "Continue on <STREET_NAMES(2)> for 1 kilometer."
  // 7 "Continue on <STREET_NAMES(2)> for a half kilometer." (500 meters)
  // 8 "Continue on <STREET_NAMES(2)> for 100 meters." (30-400 and 600-900 meters)
  // 9 "Continue on <STREET_NAMES(2)> for less than 10 meters."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (include_street_names && maneuver.HasStreetNames()) {
    instruction += " on ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }
  instruction += " for ";

  float kilometers = maneuver.length();
  float kilometer_tenths = std::round(kilometers * 10) / 10;

  if (kilometer_tenths > 1.0f) {
    instruction += (boost::format("%.1f kilometers.") % kilometer_tenths).str();
  } else if (kilometer_tenths == 1.0f) {
    instruction += "1 kilometer.";
  } else if (kilometer_tenths == 0.5f) {
    instruction += "a half kilometer.";
  } else {
    float kilometer_hundredths = std::round(kilometers * 100) / 100;
    float kilometer_thousandths = std::round(kilometers * 1000) / 1000;

    // Process hundred meters
    if (kilometer_hundredths > 0.09f) {
      instruction += (boost::format("%d meters.")
          % static_cast<uint32_t>(kilometer_tenths * 1000)).str();
    } else if (kilometer_thousandths > 0.009f) {
      instruction += (boost::format("%d meters.")
          % static_cast<uint32_t>(kilometer_hundredths * 1000)).str();
    } else {
      instruction += "less than 10 meters.";
    }

  }

  return instruction;
}

std::string NarrativeBuilder::FormVerbalPostTransitionMilesInstruction(
    Maneuver& maneuver, bool include_street_names,
    uint32_t element_max_count, std::string delim) {

  // 0  "Continue for 1.2 miles"
  // 1  "Continue for 1 mile."
  // 2  "Continue for a half mile."
  // 3  "Continue for 2 tenths of a mile." (2-4, 6-9)
  // 4  "Continue for 1 tenth of a mile."
  // 5  "Continue for 200 feet." (10-90, 100-500)
  // 6  "Continue for less than 10 feet."
  //
  // 7  "Continue on <STREET_NAMES(2)> for 1.2 miles"
  // 8  "Continue on <STREET_NAMES(2)> for 1 mile."
  // 9  "Continue on <STREET_NAMES(2)> for a half mile."
  // 10 "Continue on <STREET_NAMES(2)> for two tenths of a mile." (2-4, 6-9)
  // 11 "Continue on <STREET_NAMES(2)> for 1 tenth of a mile."
  // 12 "Continue on <STREET_NAMES(2)> for 200 feet." (10-90, 100-500)
  // 13 "Continue on <STREET_NAMES(2)> for less than 10 feet."

  std::string instruction;
  instruction.reserve(kTextInstructionInitialCapacity);
  instruction += "Continue";

  if (include_street_names && maneuver.HasStreetNames()) {
    instruction += " on ";
    instruction += maneuver.street_names().ToString(
        element_max_count, delim, maneuver.verbal_formatter());
  }
  instruction += " for ";

  float miles = maneuver.length(DirectionsOptions_Units_kMiles);
  float mile_tenths = std::round(miles * 10) / 10;

  if (mile_tenths > 1.0f) {
    instruction += (boost::format("%.1f miles.") % mile_tenths).str();
  } else if (mile_tenths == 1.0f) {
    instruction += "1 mile.";
  } else if (mile_tenths == 0.5f) {
    instruction += "a half mile.";
  } else if (mile_tenths > 0.1f) {
      instruction += (boost::format("%d tenths of a mile.")
          % static_cast<uint32_t>(mile_tenths * 10)).str();
  } else if ((miles > 0.0973f) && (mile_tenths == 0.1f)) {
    instruction += "1 tenth of a mile.";
  } else {
    uint32_t feet = static_cast<uint32_t>(std::round(miles * 5280));
    if (feet > 94) {
      instruction += (boost::format("%d feet.")
          % static_cast<uint32_t>(std::round(feet / 100.0f) * 100)).str();
    } else if (feet > 9) {
      instruction += (boost::format("%d feet.")
          % static_cast<uint32_t>(std::round(feet / 10.0f) * 10)).str();
    } else {
      instruction += "less than 10 feet.";
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

}
}

