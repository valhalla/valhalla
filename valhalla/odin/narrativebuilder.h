#ifndef VALHALLA_ODIN_NARRATIVEBUILDER_H_
#define VALHALLA_ODIN_NARRATIVEBUILDER_H_

#include <vector>

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/directions_options.pb.h>

#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/odin/maneuver.h>
#include <valhalla/baldr/verbal_text_formatter.h>

namespace valhalla {
namespace odin {

const bool kLimitByConseuctiveCount = true;
constexpr uint32_t kElementMaxCount = 4;
constexpr uint32_t kVerbalAlertElementMaxCount = 1;
constexpr uint32_t kVerbalPreElementMaxCount = 2;
constexpr uint32_t kVerbalPostElementMaxCount = 2;
constexpr float kVerbalPostMinimumRampLength = 2.0f;  // Kilometers
const std::string kVerbalDelim = ", ";

class NarrativeBuilder {
 public:

  NarrativeBuilder() = delete;

  static void Build(const DirectionsOptions& directions_options,
                    const EnhancedTripPath* etp,
                    std::list<Maneuver>& maneuvers);

 protected:

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormStartInstruction(Maneuver& maneuver);

  static std::string FormVerbalStartInstruction(
      Maneuver& maneuver,
      DirectionsOptions_Units units,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormDestinationInstruction(const EnhancedTripPath* etp,
                                                Maneuver& maneuver);

  static std::string FormVerbalAlertDestinationInstruction(
      const EnhancedTripPath* etp, Maneuver& maneuver);

  static std::string FormVerbalDestinationInstruction(
      const EnhancedTripPath* etp, Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormBecomesInstruction(Maneuver& maneuver,
                                            Maneuver* prev_maneuver);

  static std::string FormVerbalBecomesInstruction(
      Maneuver& maneuver, Maneuver* prev_maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormContinueInstruction(Maneuver& maneuver);

  static std::string FormVerbalAlertContinueInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalContinueInstruction(
      Maneuver& maneuver,
      DirectionsOptions_Units units,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTurnInstruction(Maneuver& maneuver,
                                         Maneuver* prev_maneuver);

  static std::string FormVerbalAlertTurnInstruction(
      Maneuver& maneuver, Maneuver* prev_maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalTurnInstruction(
      Maneuver& maneuver, Maneuver* prev_maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormBearInstruction(Maneuver& maneuver,
                                         Maneuver* prev_maneuver);

  static std::string FormVerbalAlertBearInstruction(
      Maneuver& maneuver, Maneuver* prev_maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalBearInstruction(
      Maneuver& maneuver, Maneuver* prev_maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormUturnInstruction(Maneuver& maneuver,
                                          Maneuver* prev_maneuver);

  static std::string FormVerbalAlertUturnInstruction(
      Maneuver& maneuver, Maneuver* prev_maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalUturnInstruction(
      Maneuver& maneuver, Maneuver* prev_maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormRampStraightInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kElementMaxCount);

  static std::string FormVerbalAlertRampStraightInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalRampStraightInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormRampInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kElementMaxCount);

  static std::string FormVerbalAlertRampInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalRampInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalRampInstruction(
      uint8_t phrase_id, const std::string& turn,
      const std::string& exit_branch_sign, const std::string& exit_toward_sign,
      const std::string& exit_name_sign);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormExitInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kElementMaxCount);

  static std::string FormVerbalAlertExitInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalExitInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalExitInstruction(
      uint8_t phrase_id, const std::string& turn,
      const std::string& exit_number_sign, const std::string& exit_branch_sign,
      const std::string& exit_toward_sign, const std::string& exit_name_sign);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormKeepInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kElementMaxCount);

  static std::string FormVerbalAlertKeepInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalKeepInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalKeepInstruction(
      uint8_t phrase_id, const std::string& turn,
      const std::string& street_name, const std::string& exit_number_sign,
      const std::string& exit_toward_sign);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormKeepToStayOnInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kElementMaxCount);

  static std::string FormVerbalAlertKeepToStayOnInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalKeepToStayOnInstruction(
      Maneuver& maneuver,
      bool limit_by_consecutive_count = kLimitByConseuctiveCount,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalKeepToStayOnInstruction(
      uint8_t phrase_id, const std::string& turn,
      const std::string& street_name, const std::string& exit_number_sign = "",
      const std::string& exit_toward_sign = "");

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormMergeInstruction(Maneuver& maneuver);

  static std::string FormVerbalAlertMergeInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalMergeInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormEnterRoundaboutInstruction(Maneuver& maneuver);

  static std::string FormVerbalAlertEnterRoundaboutInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalEnterRoundaboutInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormExitRoundaboutInstruction(Maneuver& maneuver);

  static std::string FormVerbalExitRoundaboutInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormEnterFerryInstruction(Maneuver& maneuver);

  static std::string FormVerbalAlertEnterFerryInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalEnterFerryInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormExitFerryInstruction(Maneuver& maneuver);

  static std::string FormVerbalAlertExitFerryInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalAlertElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalExitFerryInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTransitConnectionStartInstruction(Maneuver& maneuver);

  static std::string FormVerbalTransitConnectionStartInstruction(
      Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTransitConnectionTransferInstruction(
      Maneuver& maneuver);

  static std::string FormVerbalTransitConnectionTransferInstruction(
      Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTransitConnectionDestinationInstruction(
      Maneuver& maneuver);

  static std::string FormVerbalTransitConnectionDestinationInstruction(
      Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormDepartInstruction(Maneuver& maneuver);

  static std::string FormVerbalDepartInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormArriveInstruction(Maneuver& maneuver);

  static std::string FormVerbalArriveInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTransitInstruction(Maneuver& maneuver);

  static std::string FormVerbalTransitInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTransitRemainOnInstruction(Maneuver& maneuver);

  static std::string FormVerbalTransitRemainOnInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTransitTransferInstruction(Maneuver& maneuver);

  static std::string FormVerbalTransitTransferInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormPostTransitConnectionDestinationInstruction(
      Maneuver& maneuver);

  static std::string FormVerbalPostTransitConnectionDestinationInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      std::string delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormVerbalPostTransitionInstruction(
      Maneuver& maneuver, DirectionsOptions_Units units,
      bool include_street_names = false,
      uint32_t element_max_count = kVerbalPostElementMaxCount,
      std::string delim = kVerbalDelim);

  static std::string FormVerbalPostTransitionTransitInstruction(
      Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormDistance(Maneuver& maneuver,
                                  DirectionsOptions_Units units);

  static std::string FormKilometers(float kilometers);

  static std::string FormMiles(float miles);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormCardinalDirection(
      TripDirections_Maneuver_CardinalDirection cardinal_direction);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTurnTypeInstruction(TripDirections_Maneuver_Type type);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormOrdinalValue(uint32_t value);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormStopCountLabel(size_t stop_count);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormTransitName(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  static std::string FormStreetNames(
      const Maneuver& maneuver, const StreetNames& street_names,
      bool enhance_blank_street_names = false, uint32_t max_count = 0,
      std::string delim = "/",
      const VerbalTextFormatter* verbal_formatter = nullptr);

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Processes the specified maneuver list and creates verbal multi-cue
   * instructions based on quick maneuvers.
   *
   * @param maneuvers The maneuver list to process.
   */
  static void FormVerbalMultiCue(std::list<Maneuver>& maneuvers);

  /**
   * Returns the verbal multi-cue instruction based on the specified maneuvers.
   *
   * @param maneuver The current quick maneuver that will be the first verbal
   *                 cue in the returned instruction.
   * @param next_maneuver The next maneuver that will be the second verbal cue
   *                      in the returned instruction.
   *
   * @return the verbal multi-cue instruction based on the specified maneuvers.
   */
  static std::string FormVerbalMultiCue(Maneuver* maneuver,
                                        Maneuver& next_maneuver);

  /**
   * Returns true if a verbal multi-cue instruction should be formed for the
   * two specified maneuvers.
   *
   * @param maneuver The current maneuver that must be short based on time.
   * @param next_maneuver The next maneuver that must meet criteria to be used.
   *
   * @return true if a verbal multi-cue instruction should be formed for the
   *         two specified maneuvers.
   */
  static bool IsVerbalMultiCuePossible(Maneuver* maneuver,
                                       Maneuver& next_maneuver);

};

}
}

#endif  // VALHALLA_ODIN_NARRATIVEBUILDER_H_
