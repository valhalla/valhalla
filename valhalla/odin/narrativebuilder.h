#ifndef VALHALLA_ODIN_NARRATIVEBUILDER_H_
#define VALHALLA_ODIN_NARRATIVEBUILDER_H_

#include <vector>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/directions_options.pb.h>

#include <valhalla/odin/maneuver.h>

namespace valhalla {
namespace odin {

constexpr uint32_t kVerbalAlertNameMaxCount = 1;
constexpr uint32_t kVerbalPreNameMaxCount = 2;
constexpr uint32_t kVerbalPostNameMaxCount = 2;
const std::string kVerbalDelim = ", ";

class NarrativeBuilder {
 public:

  static void Build(const DirectionsOptions& directions_options,
                    std::list<Maneuver>& maneuvers);

 protected:
  NarrativeBuilder();

  static std::string FormStartInstruction(Maneuver& maneuver);

  static std::string FormVerbalStartInstruction(
      Maneuver& maneuver,
      uint32_t street_name_max_count = kVerbalPreNameMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormDestinationInstruction(Maneuver& maneuver);

  static std::string FormVerbalAlertDestinationInstruction(Maneuver& maneuver);

  static std::string FormVerbalDestinationInstruction(Maneuver& maneuver);

  static void FormBecomesInstruction(Maneuver& maneuver,
                                     Maneuver* prev_maneuver);

  static void FormContinueInstruction(Maneuver& maneuver);

  static std::string FormTurnInstruction(Maneuver& maneuver);

  static std::string FormVerbalAlertTurnInstruction(
      Maneuver& maneuver,
      uint32_t street_name_max_count = kVerbalAlertNameMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalTurnInstruction(
      Maneuver& maneuver,
      uint32_t street_name_max_count = kVerbalPreNameMaxCount,
      std::string delim = kVerbalDelim);

  static void FormTurnToStayOnInstruction(Maneuver& maneuver);

  static void FormBearInstruction(Maneuver& maneuver);

  static void FormBearToStayOnInstruction(Maneuver& maneuver);

  static void FormUturnInstruction(Maneuver& maneuver);

  static void FormRampStraightInstruction(Maneuver& maneuver);

  static void FormRampRightInstruction(Maneuver& maneuver);

  static void FormRampLeftInstruction(Maneuver& maneuver);

  static void FormExitRightInstruction(Maneuver& maneuver);

  static void FormExitLeftInstruction(Maneuver& maneuver);

  static void FormStayStraightInstruction(Maneuver& maneuver);

  static void FormStayRightInstruction(Maneuver& maneuver);

  static void FormStayLeftInstruction(Maneuver& maneuver);

  static void FormStayStraightToStayOnInstruction(Maneuver& maneuver);

  static void FormStayRightToStayOnInstruction(Maneuver& maneuver);

  static void FormStayLeftToStayOnInstruction(Maneuver& maneuver);

  static void FormMergeInstruction(Maneuver& maneuver);

  static void FormEnterRoundaboutInstruction(Maneuver& maneuver);

  static void FormExitRoundaboutInstruction(Maneuver& maneuver);

  static void FormEnterFerryInstruction(Maneuver& maneuver);

  static void FormExitFerryInstruction(Maneuver& maneuver);

  static void FormTransitConnectionStartInstruction(Maneuver& maneuver);

  static void FormTransitConnectionTransferInstruction(Maneuver& maneuver);

  static void FormTransitConnectionDestinationInstruction(Maneuver& maneuver);

  static void FormTransitInstruction(Maneuver& maneuver);

  static void FormTransitRemainOnInstruction(Maneuver& maneuver);

  static void FormTransitTransferInstruction(Maneuver& maneuver);

  static void FormPostTransitConnectionDestinationInstruction(Maneuver& maneuver);

  static std::string FormVerbalPostTransitionInstruction(
      Maneuver& maneuver, DirectionsOptions_Units units,
      bool include_street_names = false,
      uint32_t street_name_max_count = kVerbalPostNameMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalPostTransitionKilometersInstruction(
      Maneuver& maneuver, bool include_street_names = false,
      uint32_t street_name_max_count = kVerbalPostNameMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalPostTransitionMilesInstruction(
      Maneuver& maneuver, bool include_street_names = false,
      uint32_t street_name_max_count = kVerbalPostNameMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormCardinalDirection(
      TripDirections_Maneuver_CardinalDirection cardinal_direction);

  static std::string FormTurnTypeInstruction(TripDirections_Maneuver_Type type);

  static std::string FormBearTypeInstruction(TripDirections_Maneuver_Type type);

  static std::string FormOrdinalValue(uint32_t value);

};

}
}

#endif  // VALHALLA_ODIN_NARRATIVEBUILDER_H_
