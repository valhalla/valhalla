#include <iostream>

#include "odin/narrativebuilder.h"
#include "odin/maneuver.h"

#include "boost/format.hpp"

namespace {
// Text instruction initial capacity
constexpr auto kTextInstructionInitialCapacity = 128;
}

namespace valhalla {
namespace odin {

void NarrativeBuilder::Build(std::list<Maneuver>& maneuvers) {
  for (auto& maneuver : maneuvers) {
    switch (maneuver.type()) {
      case TripDirections_Maneuver_Type_kStartRight:
      case TripDirections_Maneuver_Type_kStart:
      case TripDirections_Maneuver_Type_kStartLeft: {
        FormStartInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kDestinationRight:
      case TripDirections_Maneuver_Type_kDestination:
      case TripDirections_Maneuver_Type_kDestinationLeft: {
        FormDestinationInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kBecomes: {
        // TODO - previous name
        //FormBecomesInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kContinue: {
        FormContinueInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kSlightRight:
      case TripDirections_Maneuver_Type_kRight:
      case TripDirections_Maneuver_Type_kSharpRight:
      case TripDirections_Maneuver_Type_kSharpLeft:
      case TripDirections_Maneuver_Type_kLeft:
      case TripDirections_Maneuver_Type_kSlightLeft: {
        FormTurnInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kUturnRight:
      case TripDirections_Maneuver_Type_kUturnLeft: {
        FormUturnInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kRampStraight:
        FormRampStraightInstruction(maneuver);
        break;
      case TripDirections_Maneuver_Type_kRampRight:
        FormRampRightInstruction(maneuver);
        break;
      case TripDirections_Maneuver_Type_kRampLeft: {
        FormRampLeftInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kExitRight: {
        FormExitRightInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kExitLeft: {
        FormExitLeftInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kStayStraight:
      case TripDirections_Maneuver_Type_kStayRight:
      case TripDirections_Maneuver_Type_kStayLeft: {
        FormStayInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kMerge: {
        FormMergeInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kRoundaboutEnter: {
        FormEnterRoundaboutInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kRoundaboutExit: {
        FormExitRoundaboutInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kFerryEnter: {
        FormEnterFerryInstruction(maneuver);
        break;
      }
      case TripDirections_Maneuver_Type_kFerryExit: {
        FormExitFerryInstruction(maneuver);
        break;
      }
    }
  }
}

NarrativeBuilder::NarrativeBuilder() {
}

// TODO - we will have to optimize when we actually use the language specific
// dictionary

void NarrativeBuilder::FormStartInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Start out going ";
  text_instruction += FormCardinalDirection(
      maneuver.begin_cardinal_direction());
  if (maneuver.HasStreetNames()) {
    text_instruction += " on ";
    text_instruction += maneuver.street_names().ToString();
  }
  // TODO - side of street

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormDestinationInstruction(Maneuver& maneuver) {
  // TODO - phrase will vary depending on location
  // for now just keep it simple
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "You have arrived at your destination.";

  // TODO - side of street

  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormContinueInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Continue";

  if (maneuver.HasStreetNames()) {
    text_instruction += " onto ";
    text_instruction += maneuver.street_names().ToString();
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormTurnInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Turn ";
  text_instruction += FormTurnTypeInstruction(maneuver.type());

  if (maneuver.HasStreetNames()) {
    text_instruction += " onto ";
    text_instruction += maneuver.street_names().ToString();
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormUturnInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Make a ";
  text_instruction += FormTurnTypeInstruction(maneuver.type());
  text_instruction += " U-turn";

  // TODO - add intersecting street name
  // i.e. "at Main Street"

  if (maneuver.HasStreetNames()) {
    text_instruction += " onto ";
    text_instruction += maneuver.street_names().ToString();
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormRampStraightInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Stay straight to take the ramp";

  // TODO - exit info

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormRampRightInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  if (maneuver.begin_relative_direction()
      == Maneuver::RelativeDirection::kRight)
    text_instruction += "Turn right to take the ramp";
  else
    text_instruction += "Take the ramp on the right";

  // TODO - exit info

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormRampLeftInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  if (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft)
    text_instruction += "Turn left to take the ramp";
  else
    text_instruction += "Take the ramp on the left";

  // TODO - exit info

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormExitRightInstruction(Maneuver& maneuver) {
  // 0 Take the exit on the right
  // 1 = exit number
  // 2 = branch
  // 4 = toward
  // 8 = name?

  // 1 Take exit 67A on the right
  // 2 Take the I 95 South exit on the right
  // 3 Take exit 67A on the right onto I 95 South
  // 4 Take the exit on the right toward Baltimore
  // 5 Take exit 67A on the right toward Baltimore
  // 6 Take the I 95 South exit on the right toward Baltimore
  // 7 Take exit 67A on the right onto I 95 South toward Baltimore

  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign())
    phrase_id += 1;
  if (maneuver.HasExitBranchSign())
    phrase_id += 2;
  if (maneuver.HasExitTowardSign())
    phrase_id += 4;

  switch (phrase_id) {
    // 1 Take exit 67A on the right
    case 1: {
      text_instruction += (boost::format("Take exit %1% on the right")
          % maneuver.signs().GetExitNumberString()).str();
      break;
    }
      // 2 Take the I 95 South exit on the right
    case 2: {
      text_instruction += (boost::format("Take the %1% exit on the right")
          % maneuver.signs().GetExitBranchString()).str();
      break;
    }
      // 3 Take exit 67A on the right onto I 95 South
    case 3: {
      text_instruction += (boost::format("Take exit %1% on the right onto %2%")
          % maneuver.signs().GetExitNumberString()
          % maneuver.signs().GetExitBranchString()).str();
      break;
    }
      // 4 Take the exit on the right toward Baltimore
    case 4: {
      text_instruction += (boost::format(
          "Take the exit on the right toward %1%")
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
      // 5 Take exit 67A on the right toward Baltimore
    case 5: {
      text_instruction += (boost::format(
          "Take exit %1% on the right toward %2%")
          % maneuver.signs().GetExitNumberString()
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
      // 6 Take the I 95 South exit on the right toward Baltimore
    case 6: {
      text_instruction += (boost::format(
          "Take the %1% exit on the right toward %2%")
          % maneuver.signs().GetExitBranchString()
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
      // 7 Take exit 67A on the right onto I 95 South toward Baltimore
    case 7: {
      text_instruction += (boost::format(
          "Take exit %1% on the right onto %2% toward %3%")
          % maneuver.signs().GetExitNumberString()
          % maneuver.signs().GetExitBranchString()
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
    default: {
      text_instruction += "Take the exit on the right";
      break;
    }
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormExitLeftInstruction(Maneuver& maneuver) {
  // 0 Take the exit on the left
  // 1 = exit number
  // 2 = branch
  // 4 = toward
  // 8 = name?

  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  uint8_t phrase_id = 0;
  if (maneuver.HasExitNumberSign())
    phrase_id += 1;
  if (maneuver.HasExitBranchSign())
    phrase_id += 2;
  if (maneuver.HasExitTowardSign())
    phrase_id += 4;

  switch (phrase_id) {
    // 1 Take exit 67A on the left
    case 1: {
      text_instruction += (boost::format("Take exit %1% on the left")
          % maneuver.signs().GetExitNumberString()).str();
      break;
    }
      // 2 Take the I 95 South exit on the left
    case 2: {
      text_instruction += (boost::format("Take the %1% exit on the left")
          % maneuver.signs().GetExitBranchString()).str();
      break;
    }
      // 3 Take exit 67A on the left onto I 95 South
    case 3: {
      text_instruction += (boost::format("Take exit %1% on the left onto %2%")
          % maneuver.signs().GetExitNumberString()
          % maneuver.signs().GetExitBranchString()).str();
      break;
    }
      // 4 Take the exit on the left toward Baltimore
    case 4: {
      text_instruction += (boost::format(
          "Take the exit on the left toward %1%")
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
      // 5 Take exit 67A on the left toward Baltimore
    case 5: {
      text_instruction += (boost::format(
          "Take exit %1% on the left toward %2%")
          % maneuver.signs().GetExitNumberString()
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
      // 6 Take the I 95 South exit on the left toward Baltimore
    case 6: {
      text_instruction += (boost::format(
          "Take the %1% exit on the left toward %2%")
          % maneuver.signs().GetExitBranchString()
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
      // 7 Take exit 67A on the left onto I 95 South toward Baltimore
    case 7: {
      text_instruction += (boost::format(
          "Take exit %1% on the left onto %2% toward %3%")
          % maneuver.signs().GetExitNumberString()
          % maneuver.signs().GetExitBranchString()
          % maneuver.signs().GetExitTowardString()).str();
      break;
    }
    default: {
      text_instruction += "Take the exit on the left";
      break;
    }
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormStayInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Stay";

  // TODO - add relative direction and exit info

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormMergeInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  if (maneuver.HasStreetNames()) {
    text_instruction += "Merge onto ";
    text_instruction += maneuver.street_names().ToString();
  } else
    text_instruction += "Merge";

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormEnterRoundaboutInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Enter roundabout";

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormExitRoundaboutInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Take the ";
  text_instruction += "TBD ";  // TODO - roundabout exit count
  text_instruction += "exit";
  if (maneuver.HasStreetNames()) {
    text_instruction += " onto ";
    text_instruction += maneuver.street_names().ToString();
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormEnterFerryInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Take the ";
  if (maneuver.HasStreetNames()) {
    text_instruction += maneuver.street_names().ToString();
    text_instruction += " ";
  }
  text_instruction += "ferry";

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
}

void NarrativeBuilder::FormExitFerryInstruction(Maneuver& maneuver) {
  std::string text_instruction;
  text_instruction.reserve(kTextInstructionInitialCapacity);
  text_instruction += "Go ";
  text_instruction += FormCardinalDirection(
      maneuver.begin_cardinal_direction());
  if (maneuver.HasStreetNames()) {
    text_instruction += " on ";
    text_instruction += maneuver.street_names().ToString();
  }

  text_instruction += ".";
  maneuver.set_instruction(std::move(text_instruction));
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
  }
}

std::string NarrativeBuilder::FormTurnTypeInstruction(
    TripDirections_Maneuver_Type type) {
  switch (type) {
    case TripDirections_Maneuver_Type_kSlightRight: {
      return "slight right";
    }
    case TripDirections_Maneuver_Type_kRight:
    case TripDirections_Maneuver_Type_kUturnRight: {
      return "right";
    }
    case TripDirections_Maneuver_Type_kSharpRight: {
      return "sharp right";
    }
    case TripDirections_Maneuver_Type_kSharpLeft: {
      return "sharp left";
    }
    case TripDirections_Maneuver_Type_kLeft:
    case TripDirections_Maneuver_Type_kUturnLeft: {
      return "left";
    }
    case TripDirections_Maneuver_Type_kSlightLeft: {
      return "slight left";
    }
  }

}

}
}

