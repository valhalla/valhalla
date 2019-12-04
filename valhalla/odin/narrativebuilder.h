#ifndef VALHALLA_ODIN_NARRATIVEBUILDER_H_
#define VALHALLA_ODIN_NARRATIVEBUILDER_H_

#include <cstdint>
#include <vector>

#include <valhalla/baldr/verbal_text_formatter.h>

#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/odin/maneuver.h>
#include <valhalla/odin/narrative_dictionary.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

namespace valhalla {
namespace odin {

const bool kLimitByConseuctiveCount = true;
constexpr uint32_t kElementMaxCount = 4;
constexpr uint32_t kVerbalAlertElementMaxCount = 1;
constexpr uint32_t kVerbalPreElementMaxCount = 2;
constexpr uint32_t kVerbalPostElementMaxCount = 2;
const std::string kVerbalDelim = ", ";

class NarrativeBuilder {
public:
  NarrativeBuilder(const Options& options,
                   const EnhancedTripLeg* trip_path,
                   const NarrativeDictionary& dictionary);

  virtual ~NarrativeBuilder() = default;

  NarrativeBuilder(NarrativeBuilder&&) = default;
  NarrativeBuilder& operator=(NarrativeBuilder&&) = default;

  NarrativeBuilder(const NarrativeBuilder&) = default;
  NarrativeBuilder& operator=(const NarrativeBuilder&) = default;

  void Build(const Options& options, const EnhancedTripLeg* etp, std::list<Maneuver>& maneuvers);

protected:
  /////////////////////////////////////////////////////////////////////////////
  std::string FormStartInstruction(Maneuver& maneuver);

  std::string FormVerbalStartInstruction(Maneuver& maneuver,
                                         uint32_t element_max_count = kVerbalPreElementMaxCount,
                                         const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormDestinationInstruction(Maneuver& maneuver);

  std::string FormVerbalAlertDestinationInstruction(Maneuver& maneuver);

  std::string FormVerbalDestinationInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormBecomesInstruction(Maneuver& maneuver, Maneuver* prev_maneuver);

  std::string FormVerbalBecomesInstruction(Maneuver& maneuver,
                                           Maneuver* prev_maneuver,
                                           uint32_t element_max_count = kVerbalPreElementMaxCount,
                                           const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormContinueInstruction(Maneuver& maneuver);

  std::string
  FormVerbalAlertContinueInstruction(Maneuver& maneuver,
                                     uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                     const std::string& delim = kVerbalDelim);

  std::string FormVerbalContinueInstruction(Maneuver& maneuver,
                                            Options_Units units,
                                            uint32_t element_max_count = kVerbalPreElementMaxCount,
                                            const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormTurnInstruction(Maneuver& maneuver);

  std::string FormVerbalAlertTurnInstruction(Maneuver& maneuver,
                                             uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                             const std::string& delim = kVerbalDelim);

  std::string FormVerbalTurnInstruction(Maneuver& maneuver,
                                        uint32_t element_max_count = kVerbalPreElementMaxCount,
                                        const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormUturnInstruction(Maneuver& maneuver);

  std::string
  FormVerbalAlertUturnInstruction(Maneuver& maneuver,
                                  uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                  const std::string& delim = kVerbalDelim);

  std::string FormVerbalUturnInstruction(Maneuver& maneuver,
                                         uint32_t element_max_count = kVerbalPreElementMaxCount,
                                         const std::string& delim = kVerbalDelim);

  std::string FormVerbalUturnInstruction(uint8_t phrase_id,
                                         const std::string& relative_dir,
                                         const std::string& street_names,
                                         const std::string& cross_street_names);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormRampStraightInstruction(Maneuver& maneuver,
                                          bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                          uint32_t element_max_count = kElementMaxCount);

  std::string
  FormVerbalAlertRampStraightInstruction(Maneuver& maneuver,
                                         bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                         uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                         const std::string& delim = kVerbalDelim);

  std::string
  FormVerbalRampStraightInstruction(Maneuver& maneuver,
                                    bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                    uint32_t element_max_count = kVerbalPreElementMaxCount,
                                    const std::string& delim = kVerbalDelim);

  std::string FormVerbalRampStraightInstruction(uint8_t phrase_id,
                                                const std::string& exit_branch_sign,
                                                const std::string& exit_toward_sign,
                                                const std::string& exit_name_sign);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormRampInstruction(Maneuver& maneuver,
                                  bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                  uint32_t element_max_count = kElementMaxCount);

  std::string
  FormVerbalAlertRampInstruction(Maneuver& maneuver,
                                 bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                 uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                 const std::string& delim = kVerbalDelim);

  std::string FormVerbalRampInstruction(Maneuver& maneuver,
                                        bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                        uint32_t element_max_count = kVerbalPreElementMaxCount,
                                        const std::string& delim = kVerbalDelim);

  std::string FormVerbalRampInstruction(uint8_t phrase_id,
                                        const std::string& relative_dir,
                                        const std::string& exit_branch_sign,
                                        const std::string& exit_toward_sign,
                                        const std::string& exit_name_sign);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormExitInstruction(Maneuver& maneuver,
                                  bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                  uint32_t element_max_count = kElementMaxCount);

  std::string
  FormVerbalAlertExitInstruction(Maneuver& maneuver,
                                 bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                 uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                 const std::string& delim = kVerbalDelim);

  std::string FormVerbalExitInstruction(Maneuver& maneuver,
                                        bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                        uint32_t element_max_count = kVerbalPreElementMaxCount,
                                        const std::string& delim = kVerbalDelim);

  std::string FormVerbalExitInstruction(uint8_t phrase_id,
                                        const std::string& relative_dir,
                                        const std::string& exit_number_sign,
                                        const std::string& exit_branch_sign,
                                        const std::string& exit_toward_sign,
                                        const std::string& exit_name_sign);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormKeepInstruction(Maneuver& maneuver,
                                  bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                  uint32_t element_max_count = kElementMaxCount);

  std::string
  FormVerbalAlertKeepInstruction(Maneuver& maneuver,
                                 bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                 uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                 const std::string& delim = kVerbalDelim);

  std::string FormVerbalKeepInstruction(Maneuver& maneuver,
                                        bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                        uint32_t element_max_count = kVerbalPreElementMaxCount,
                                        const std::string& delim = kVerbalDelim);

  std::string FormVerbalKeepInstruction(uint8_t phrase_id,
                                        const std::string& relative_dir,
                                        const std::string& street_name,
                                        const std::string& exit_number_sign,
                                        const std::string& exit_toward_sign);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormKeepToStayOnInstruction(Maneuver& maneuver,
                                          bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                          uint32_t element_max_count = kElementMaxCount);

  std::string
  FormVerbalAlertKeepToStayOnInstruction(Maneuver& maneuver,
                                         bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                         uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                         const std::string& delim = kVerbalDelim);

  std::string
  FormVerbalKeepToStayOnInstruction(Maneuver& maneuver,
                                    bool limit_by_consecutive_count = kLimitByConseuctiveCount,
                                    uint32_t element_max_count = kVerbalPreElementMaxCount,
                                    const std::string& delim = kVerbalDelim);

  std::string FormVerbalKeepToStayOnInstruction(uint8_t phrase_id,
                                                const std::string& relative_dir,
                                                const std::string& street_name,
                                                const std::string& exit_number_sign = "",
                                                const std::string& exit_toward_sign = "");

  /////////////////////////////////////////////////////////////////////////////
  std::string FormMergeInstruction(Maneuver& maneuver);

  std::string
  FormVerbalAlertMergeInstruction(Maneuver& maneuver,
                                  uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                  const std::string& delim = kVerbalDelim);

  std::string FormVerbalMergeInstruction(Maneuver& maneuver,
                                         uint32_t element_max_count = kVerbalPreElementMaxCount,
                                         const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormEnterRoundaboutInstruction(Maneuver& maneuver);

  std::string
  FormVerbalAlertEnterRoundaboutInstruction(Maneuver& maneuver,
                                            uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                            const std::string& delim = kVerbalDelim);

  std::string
  FormVerbalEnterRoundaboutInstruction(Maneuver& maneuver,
                                       uint32_t element_max_count = kVerbalPreElementMaxCount,
                                       const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormExitRoundaboutInstruction(Maneuver& maneuver);

  std::string
  FormVerbalExitRoundaboutInstruction(Maneuver& maneuver,
                                      uint32_t element_max_count = kVerbalPreElementMaxCount,
                                      const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormEnterFerryInstruction(Maneuver& maneuver);

  std::string
  FormVerbalAlertEnterFerryInstruction(Maneuver& maneuver,
                                       uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                       const std::string& delim = kVerbalDelim);

  std::string FormVerbalEnterFerryInstruction(Maneuver& maneuver,
                                              uint32_t element_max_count = kVerbalPreElementMaxCount,
                                              const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormExitFerryInstruction(Maneuver& maneuver);

  std::string
  FormVerbalAlertExitFerryInstruction(Maneuver& maneuver,
                                      uint32_t element_max_count = kVerbalAlertElementMaxCount,
                                      const std::string& delim = kVerbalDelim);

  std::string FormVerbalExitFerryInstruction(Maneuver& maneuver,
                                             uint32_t element_max_count = kVerbalPreElementMaxCount,
                                             const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormTransitConnectionStartInstruction(Maneuver& maneuver);

  std::string FormVerbalTransitConnectionStartInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormTransitConnectionTransferInstruction(Maneuver& maneuver);

  std::string FormVerbalTransitConnectionTransferInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormTransitConnectionDestinationInstruction(Maneuver& maneuver);

  std::string FormVerbalTransitConnectionDestinationInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormDepartInstruction(Maneuver& maneuver);

  std::string FormVerbalDepartInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormArriveInstruction(Maneuver& maneuver);

  std::string FormVerbalArriveInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormTransitInstruction(Maneuver& maneuver);

  std::string FormVerbalTransitInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormTransitRemainOnInstruction(Maneuver& maneuver);

  std::string FormVerbalTransitRemainOnInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormTransitTransferInstruction(Maneuver& maneuver);

  std::string FormVerbalTransitTransferInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormPostTransitConnectionDestinationInstruction(Maneuver& maneuver);

  std::string FormVerbalPostTransitConnectionDestinationInstruction(
      Maneuver& maneuver,
      uint32_t element_max_count = kVerbalPreElementMaxCount,
      const std::string& delim = kVerbalDelim);

  /////////////////////////////////////////////////////////////////////////////
  std::string
  FormVerbalPostTransitionInstruction(Maneuver& maneuver,
                                      bool include_street_names = false,
                                      uint32_t element_max_count = kVerbalPostElementMaxCount,
                                      const std::string& delim = kVerbalDelim);

  std::string FormVerbalPostTransitionTransitInstruction(Maneuver& maneuver);

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the transit stop count label based on the value of the specified
   * stop count and language rules.
   *
   * @param stop_count Specified stop count of transit line.
   * @param transit_platform_count_labels Map of stop count labels.
   *
   * @return the transit platform count label based on the value of the specified
   * stop count and language rules.
   */
  std::string FormTransitPlatformCountLabel(
      size_t stop_count,
      const std::unordered_map<std::string, std::string>& transit_platform_count_labels);

  /**
   * Returns the plural category based on the value of the specified
   * count and the language rules.
   *
   * @param count Specified value to determine plural category.
   *
   * @return the plural category based on the value of the specified
   * count and the language rules.
   */
  virtual std::string GetPluralCategory(size_t count);

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the length string of the specified maneuver.
   *
   * @param maneuver The maneuver to process.
   *
   * @return the length string of the specified maneuver.
   */
  std::string FormLength(Maneuver& maneuver,
                         const std::vector<std::string>& metric_lengths,
                         const std::vector<std::string>& us_customary_lengths);

  /**
   * Returns the metric length string of the specified kilometer value.
   *
   * @param kilometers The length value to process.
   *
   * @return the metric length string of the specified length value.
   */
  std::string FormMetricLength(float kilometers, const std::vector<std::string>& metric_lengths);

  /**
   * Returns the US customary length string of the specified miles value.
   *
   * @param miles The length value to process.
   *
   * @return the US customary length string of the specified length value.
   */
  std::string FormUsCustomaryLength(float miles,
                                    const std::vector<std::string>& us_customary_lengths);

  /////////////////////////////////////////////////////////////////////////////
  std::string FormRelativeTwoDirection(DirectionsLeg_Maneuver_Type type,
                                       const std::vector<std::string>& relative_directions);

  std::string FormRelativeThreeDirection(DirectionsLeg_Maneuver_Type type,
                                         const std::vector<std::string>& relative_directions);

  std::string FormRelativeTurnDirection(DirectionsLeg_Maneuver_Type type,
                                        const std::vector<std::string>& relative_directions);

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the transit name in this precedence - depending on which one exists:
   *    1) Short name
   * or 2) Long name
   * or 3) Generic name based on transit travel type
   *
   * @param maneuver The maneuver with the transit names to process.
   * @param empty_transit_name_labels A list of empty transit name labels.
   *
   * @return the transit name.
   */
  std::string FormTransitName(const Maneuver& maneuver,
                              const std::vector<std::string>& empty_transit_name_labels);

  /////////////////////////////////////////////////////////////////////////////
  // TODO make virtual
  bool HasLabel(const std::string& name, const std::string& label);

  /**
   * Returns the street names string for the specified street name list.
   * The format is controlled via the optional parameters.
   *
   * @param maneuver The maneuver with the street names to process.
   * @param street_names The list of street names to process.
   * @param empty_street_name_labels A pointer to a list of empty street name
   *                                labels.
   * @param enhance_empty_street_names If true, enhance the empty street name
   *                                   string with label depending on travel
   *                                   mode. (e.g., walkway)
   *                                   The default value is false.
   *                                   This parameter is optional.
   * @param max_count The maximum number of street names to process.
   *                  The default value is zero - meaning no limit.
   *                  This parameter is optional.
   * @param delim The specified delimiter to use between each street name.
   *              The default delimiter is a slash.
   *              This parameter is optional.
   * @param verbal_formatter A pointer to a verbal text formatter that prepares
   *                         strings for use with a text-to-speech engine.
   *                         The default is a nullptr.
   *                         This parameter is optional.
   *
   * @return the street names string for the specified street name list.
   */
  std::string FormStreetNames(const Maneuver& maneuver,
                              const StreetNames& street_names,
                              const std::vector<std::string>* empty_street_name_labels = nullptr,
                              bool enhance_empty_street_names = false,
                              uint32_t max_count = 0,
                              const std::string& delim = "/",
                              const VerbalTextFormatter* verbal_formatter = nullptr);

  /**
   * Returns the street names string for the specified street name list.
   * The format is controlled via the optional parameters.
   *
   * @param street_names The list of street names to process.
   * @param max_count The maximum number of street names to process.
   *                  The default value is zero - meaning no limit.
   *                  This parameter is optional.
   * @param delim The specified delimiter to use between each street name.
   *              The default delimiter is a slash.
   *              This parameter is optional.
   * @param verbal_formatter A pointer to a verbal text formatter that prepares
   *                         strings for use with a text-to-speech engine.
   *                         The default is a nullptr.
   *                         This parameter is optional.
   *
   * @return the street names string for the specified street name list.
   */
  std::string FormStreetNames(const StreetNames& street_names,
                              uint32_t max_count = 0,
                              const std::string& delim = "/",
                              const VerbalTextFormatter* verbal_formatter = nullptr);

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Processes the specified maneuver list and creates verbal multi-cue
   * instructions based on quick maneuvers.
   *
   * @param maneuvers The maneuver list to process.
   */
  void FormVerbalMultiCue(std::list<Maneuver>& maneuvers);

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
  std::string FormVerbalMultiCue(Maneuver* maneuver, Maneuver& next_maneuver);

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
  bool IsVerbalMultiCuePossible(Maneuver* maneuver, Maneuver& next_maneuver);

  /**
   * Combines a simple preposition and a definite article for certain languages.
   */
  virtual void FormArticulatedPrepositions(std::string& instruction) {
  }

  /////////////////////////////////////////////////////////////////////////////
  const Options& options_;
  const EnhancedTripLeg* trip_path_;
  const NarrativeDictionary& dictionary_;
  bool articulated_preposition_enabled_;
};

///////////////////////////////////////////////////////////////////////////////
class NarrativeBuilder_csCZ : public NarrativeBuilder {

public:
  NarrativeBuilder_csCZ(const Options& options,
                        const EnhancedTripLeg* trip_path,
                        const NarrativeDictionary& dictionary)
      : NarrativeBuilder(options, trip_path, dictionary) {
  }

protected:
  /**
   * Returns the plural category based on the value of the specified
   * count and the language rules.
   *
   * @param count Specified value to determine plural category.
   *
   * @return the plural category based on the value of the specified
   * count and the language rules.
   */
  std::string GetPluralCategory(size_t count) override;
};

///////////////////////////////////////////////////////////////////////////////
class NarrativeBuilder_hiIN : public NarrativeBuilder {

public:
  NarrativeBuilder_hiIN(const Options& options,
                        const EnhancedTripLeg* trip_path,
                        const NarrativeDictionary& dictionary)
      : NarrativeBuilder(options, trip_path, dictionary) {
  }

protected:
  /**
   * Returns the plural category based on the value of the specified
   * count and the language rules.
   *
   * @param count Specified value to determine plural category.
   *
   * @return the plural category based on the value of the specified
   * count and the language rules.
   */
  std::string GetPluralCategory(size_t count) override;
};

///////////////////////////////////////////////////////////////////////////////
class NarrativeBuilder_itIT : public NarrativeBuilder {

public:
  NarrativeBuilder_itIT(const Options& options,
                        const EnhancedTripLeg* trip_path,
                        const NarrativeDictionary& dictionary)
      : NarrativeBuilder(options, trip_path, dictionary) {
    // Enable articulated prepositions for Itailian
    articulated_preposition_enabled_ = true;
  }

protected:
  /**
   * Combines a simple preposition and a definite article for certain languages.
   */
  void FormArticulatedPrepositions(std::string& instruction) override;

private:
  static const std::unordered_map<std::string, std::string> articulated_prepositions_;
};

///////////////////////////////////////////////////////////////////////////////
class NarrativeBuilder_ruRU : public NarrativeBuilder {

public:
  NarrativeBuilder_ruRU(const Options& options,
                        const EnhancedTripLeg* trip_path,
                        const NarrativeDictionary& dictionary)
      : NarrativeBuilder(options, trip_path, dictionary) {
  }

protected:
  /**
   * Returns the plural category based on the value of the specified
   * count and the language rules.
   *
   * @param count Specified value to determine plural category.
   *
   * @return the plural category based on the value of the specified
   * count and the language rules.
   */
  std::string GetPluralCategory(size_t count) override;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_NARRATIVEBUILDER_H_
