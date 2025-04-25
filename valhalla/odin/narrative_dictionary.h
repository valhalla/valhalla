#ifndef VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_
#define VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_

#include <locale>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree.hpp>

namespace {

// Subset keys
inline constexpr auto kStartKey = "instructions.start";
inline constexpr auto kStartVerbalKey = "instructions.start_verbal";
inline constexpr auto kDestinationKey = "instructions.destination";
inline constexpr auto kDestinationVerbalAlertKey = "instructions.destination_verbal_alert";
inline constexpr auto kDestinationVerbalKey = "instructions.destination_verbal";
inline constexpr auto kBecomesKey = "instructions.becomes";
inline constexpr auto kBecomesVerbalKey = "instructions.becomes_verbal";
inline constexpr auto kContinueKey = "instructions.continue";
inline constexpr auto kContinueVerbalAlertKey = "instructions.continue_verbal_alert";
inline constexpr auto kContinueVerbalKey = "instructions.continue_verbal";
inline constexpr auto kBearKey = "instructions.bear";
inline constexpr auto kBearVerbalKey = "instructions.bear_verbal";
inline constexpr auto kTurnKey = "instructions.turn";
inline constexpr auto kTurnVerbalKey = "instructions.turn_verbal";
inline constexpr auto kSharpKey = "instructions.sharp";
inline constexpr auto kSharpVerbalKey = "instructions.sharp_verbal";
inline constexpr auto kUturnKey = "instructions.uturn";
inline constexpr auto kUturnVerbalKey = "instructions.uturn_verbal";
inline constexpr auto kRampStraightKey = "instructions.ramp_straight";
inline constexpr auto kRampStraightVerbalKey = "instructions.ramp_straight_verbal";
inline constexpr auto kRampKey = "instructions.ramp";
inline constexpr auto kRampVerbalKey = "instructions.ramp_verbal";
inline constexpr auto kExitKey = "instructions.exit";
inline constexpr auto kExitVerbalKey = "instructions.exit_verbal";
inline constexpr auto kExitVisualKey = "instructions.exit_visual";
inline constexpr auto kKeepKey = "instructions.keep";
inline constexpr auto kKeepVerbalKey = "instructions.keep_verbal";
inline constexpr auto kKeepToStayOnKey = "instructions.keep_to_stay_on";
inline constexpr auto kKeepToStayOnVerbalKey = "instructions.keep_to_stay_on_verbal";
inline constexpr auto kMergeKey = "instructions.merge";
inline constexpr auto kMergeVerbalKey = "instructions.merge_verbal";
inline constexpr auto kEnterRoundaboutKey = "instructions.enter_roundabout";
inline constexpr auto kEnterRoundaboutVerbalKey = "instructions.enter_roundabout_verbal";
inline constexpr auto kExitRoundaboutKey = "instructions.exit_roundabout";
inline constexpr auto kExitRoundaboutVerbalKey = "instructions.exit_roundabout_verbal";
inline constexpr auto kEnterFerryKey = "instructions.enter_ferry";
inline constexpr auto kEnterFerryVerbalKey = "instructions.enter_ferry_verbal";
inline constexpr auto kExitFerryKey = "instructions.exit_ferry";
inline constexpr auto kExitFerryVerbalKey = "instructions.exit_ferry_verbal";
inline constexpr auto kTransitConnectionStartKey = "instructions.transit_connection_start";
inline constexpr auto kTransitConnectionStartVerbalKey = "instructions.transit_connection_start_verbal";
inline constexpr auto kTransitConnectionTransferKey = "instructions.transit_connection_transfer";
inline constexpr auto kTransitConnectionTransferVerbalKey =
    "instructions.transit_connection_transfer_verbal";
inline constexpr auto kTransitConnectionDestinationKey = "instructions.transit_connection_destination";
inline constexpr auto kTransitConnectionDestinationVerbalKey =
    "instructions.transit_connection_destination_verbal";
inline constexpr auto kDepartKey = "instructions.depart";
inline constexpr auto kDepartVerbalKey = "instructions.depart_verbal";
inline constexpr auto kArriveKey = "instructions.arrive";
inline constexpr auto kArriveVerbalKey = "instructions.arrive_verbal";
inline constexpr auto kTransitKey = "instructions.transit";
inline constexpr auto kTransitVerbalKey = "instructions.transit_verbal";
inline constexpr auto kTransitRemainOnKey = "instructions.transit_remain_on";
inline constexpr auto kTransitRemainOnVerbalKey = "instructions.transit_remain_on_verbal";
inline constexpr auto kTransitTransferKey = "instructions.transit_transfer";
inline constexpr auto kTransitTransferVerbalKey = "instructions.transit_transfer_verbal";
inline constexpr auto kPostTransitConnectionDestinationKey =
    "instructions.post_transit_connection_destination";
inline constexpr auto kPostTransitConnectionDestinationVerbalKey =
    "instructions.post_transit_connection_destination_verbal";
inline constexpr auto kPostTransitionVerbalKey = "instructions.post_transition_verbal";
inline constexpr auto kPostTransitTransitionVerbalKey = "instructions.post_transition_transit_verbal";
inline constexpr auto kVerbalMultiCueKey = "instructions.verbal_multi_cue";
inline constexpr auto kApproachVerbalAlertKey = "instructions.approach_verbal_alert";
inline constexpr auto kPassKey = "instructions.pass";
inline constexpr auto kElevatorKey = "instructions.elevator";
inline constexpr auto kStepsKey = "instructions.steps";
inline constexpr auto kEscalatorKey = "instructions.escalator";
inline constexpr auto kEnterBuildingKey = "instructions.enter_building";
inline constexpr auto kExitBuildingKey = "instructions.exit_building";
inline constexpr auto kPosixLocaleKey = "posix_locale";

// Variable keys
inline constexpr auto kPhrasesKey = "phrases";
inline constexpr auto kCardinalDirectionsKey = "cardinal_directions";
inline constexpr auto kRelativeDirectionsKey = "relative_directions";
inline constexpr auto kOrdinalValuesKey = "ordinal_values";
inline constexpr auto kEmptyStreetNameLabelsKey = "empty_street_name_labels";
inline constexpr auto kMetricLengthsKey = "metric_lengths";
inline constexpr auto kUsCustomaryLengthsKey = "us_customary_lengths";
inline constexpr auto kFerryLabelKey = "ferry_label";
inline constexpr auto kStationLabelKey = "station_label";
inline constexpr auto kEmptyTransitNameLabelsKey = "empty_transit_name_labels";
inline constexpr auto kTransitStopCountLabelsKey = "transit_stop_count_labels";
inline constexpr auto kObjectLabelsKey = "object_labels";

inline constexpr auto kPluralCategoryZeroKey = "zero";
inline constexpr auto kPluralCategoryOneKey = "one";
inline constexpr auto kPluralCategoryTwoKey = "two";
inline constexpr auto kPluralCategoryFewKey = "few";
inline constexpr auto kPluralCategoryManyKey = "many";
inline constexpr auto kPluralCategoryOtherKey = "other";

// Empty street names label indexes
inline constexpr auto kWalkwayIndex = 0;
inline constexpr auto kCyclewayIndex = 1;
inline constexpr auto kMountainBikeTrailIndex = 2;
inline constexpr auto kPedestrianCrossingIndex = 3;
inline constexpr auto kStepsIndex = 4;
inline constexpr auto kBridgeIndex = 5;
inline constexpr auto kTunnelIndex = 6;

// object label indexes
inline constexpr auto kGateIndex = 0;
inline constexpr auto kBollardIndex = 1;
inline constexpr auto kStreetIntersectionIndex = 2;

// Metric length indexes
inline constexpr auto kKilometersIndex = 0;
inline constexpr auto kOneKilometerIndex = 1;
inline constexpr auto kMetersIndex = 2;
inline constexpr auto kSmallMetersIndex = 3;

// US Customary length indexes
inline constexpr auto kMilesIndex = 0;
inline constexpr auto kOneMileIndex = 1;
inline constexpr auto kHalfMileIndex = 2;
inline constexpr auto kQuarterMileIndex = 3;
inline constexpr auto kFeetIndex = 4;
inline constexpr auto kSmallFeetIndex = 5;

// Phrase tags
inline constexpr auto kCardinalDirectionTag = "<CARDINAL_DIRECTION>";
inline constexpr auto kRelativeDirectionTag = "<RELATIVE_DIRECTION>";
inline constexpr auto kOrdinalValueTag = "<ORDINAL_VALUE>";
inline constexpr auto kStreetNamesTag = "<STREET_NAMES>";
inline constexpr auto kPreviousStreetNamesTag = "<PREVIOUS_STREET_NAMES>";
inline constexpr auto kBeginStreetNamesTag = "<BEGIN_STREET_NAMES>";
inline constexpr auto kCrossStreetNamesTag = "<CROSS_STREET_NAMES>";
inline constexpr auto kRoundaboutExitStreetNamesTag = "<ROUNDABOUT_EXIT_STREET_NAMES>";
inline constexpr auto kRoundaboutExitBeginStreetNamesTag = "<ROUNDABOUT_EXIT_BEGIN_STREET_NAMES>";
inline constexpr auto kRampExitNumbersVisualTag = "<EXIT_NUMBERS>";
inline constexpr auto kObjectLabelTag = "<OBJECT_LABEL>";
inline constexpr auto kLengthTag = "<LENGTH>";
inline constexpr auto kDestinationTag = "<DESTINATION>";
inline constexpr auto kCurrentVerbalCueTag = "<CURRENT_VERBAL_CUE>";
inline constexpr auto kNextVerbalCueTag = "<NEXT_VERBAL_CUE>";
inline constexpr auto kKilometersTag = "<KILOMETERS>";
inline constexpr auto kMetersTag = "<METERS>";
inline constexpr auto kMilesTag = "<MILES>";
inline constexpr auto kTenthsOfMilesTag = "<TENTHS_OF_MILE>";
inline constexpr auto kFeetTag = "<FEET>";
inline constexpr auto kNumberSignTag = "<NUMBER_SIGN>";
inline constexpr auto kBranchSignTag = "<BRANCH_SIGN>";
inline constexpr auto kTowardSignTag = "<TOWARD_SIGN>";
inline constexpr auto kNameSignTag = "<NAME_SIGN>";
inline constexpr auto kJunctionNameTag = "<JUNCTION_NAME>";
inline constexpr auto kFerryLabelTag = "<FERRY_LABEL>";
inline constexpr auto kTransitPlatformTag = "<TRANSIT_STOP>";
inline constexpr auto kStationLabelTag = "<STATION_LABEL>";
inline constexpr auto kTimeTag = "<TIME>";
inline constexpr auto kTransitNameTag = "<TRANSIT_NAME>";
inline constexpr auto kTransitHeadSignTag = "<TRANSIT_HEADSIGN>";
inline constexpr auto kTransitPlatformCountTag = "<TRANSIT_STOP_COUNT>";
inline constexpr auto kTransitPlatformCountLabelTag = "<TRANSIT_STOP_COUNT_LABEL>";
inline constexpr auto kLevelTag = "<LEVEL>";

} // namespace

namespace valhalla {
namespace odin {

struct PhraseSet {
  std::unordered_map<std::string, std::string> phrases;
};

struct StartSubset : PhraseSet {
  std::vector<std::string> cardinal_directions;
  std::vector<std::string> empty_street_name_labels;
};

struct StartVerbalSubset : StartSubset {
  std::vector<std::string> metric_lengths;
  std::vector<std::string> us_customary_lengths;
};

struct DestinationSubset : PhraseSet {
  std::vector<std::string> relative_directions;
};

struct ContinueSubset : PhraseSet {
  std::vector<std::string> empty_street_name_labels;
};

struct ContinueVerbalSubset : ContinueSubset {
  std::vector<std::string> metric_lengths;
  std::vector<std::string> us_customary_lengths;
};

struct TurnSubset : PhraseSet {
  std::vector<std::string> relative_directions;
  std::vector<std::string> empty_street_name_labels;
};

struct RampSubset : PhraseSet {
  std::vector<std::string> relative_directions;
};

struct KeepSubset : RampSubset {
  std::vector<std::string> empty_street_name_labels;
};

struct EnterFerrySubset : PhraseSet {
  std::vector<std::string> empty_street_name_labels;
  std::string ferry_label;
};

struct EnterRoundaboutSubset : PhraseSet {
  std::vector<std::string> ordinal_values;
  std::vector<std::string> empty_street_name_labels;
};

struct TransitConnectionSubset : PhraseSet {
  std::string station_label;
};

struct TransitSubset : PhraseSet {
  std::vector<std::string> empty_transit_name_labels;
};

struct TransitStopSubset : TransitSubset {
  std::unordered_map<std::string, std::string> transit_stop_count_labels;
};

struct PostTransitionVerbalSubset : PhraseSet {
  std::vector<std::string> metric_lengths;
  std::vector<std::string> us_customary_lengths;
  std::vector<std::string> empty_street_name_labels;
};

struct PostTransitionTransitVerbalSubset : PhraseSet {
  std::unordered_map<std::string, std::string> transit_stop_count_labels;
};

struct VerbalMultiCueSubset : PhraseSet {
  std::vector<std::string> metric_lengths;
  std::vector<std::string> us_customary_lengths;
};

struct ApproachVerbalAlertSubset : PhraseSet {
  std::vector<std::string> metric_lengths;
  std::vector<std::string> us_customary_lengths;
};

struct PassSubset : PhraseSet {
  std::vector<std::string> object_labels;
};

struct EnterBuildingSubset : PhraseSet {
  std::vector<std::string> empty_street_name_labels;
};

struct ExitBuildingSubset : PhraseSet {
  std::vector<std::string> empty_street_name_labels;
};

/**
 * A class that stores the localized narrative instructions.
 */
class NarrativeDictionary {
public:
  NarrativeDictionary(const std::string& language_tag,
                      const boost::property_tree::ptree& narrative_pt);

  // Start
  StartSubset start_subset;
  StartVerbalSubset start_verbal_subset;

  // Destination
  DestinationSubset destination_subset;
  DestinationSubset destination_verbal_alert_subset;
  DestinationSubset destination_verbal_subset;

  // Becomes
  PhraseSet becomes_subset;
  PhraseSet becomes_verbal_subset;

  // Continue
  ContinueSubset continue_subset;
  ContinueSubset continue_verbal_alert_subset;
  ContinueVerbalSubset continue_verbal_subset;

  // Bear
  TurnSubset bear_subset;
  TurnSubset bear_verbal_subset;

  // Turn
  TurnSubset turn_subset;
  TurnSubset turn_verbal_subset;

  // Sharp
  TurnSubset sharp_subset;
  TurnSubset sharp_verbal_subset;

  // Uturn
  TurnSubset uturn_subset;
  TurnSubset uturn_verbal_subset;

  // RampStraight
  PhraseSet ramp_straight_subset;
  PhraseSet ramp_straight_verbal_subset;

  // Ramp
  RampSubset ramp_subset;
  RampSubset ramp_verbal_subset;

  // Exit
  RampSubset exit_subset;
  RampSubset exit_verbal_subset;
  PhraseSet exit_visual_subset;

  // Keep
  KeepSubset keep_subset;
  KeepSubset keep_verbal_subset;

  // KeepToStayOn
  KeepSubset keep_to_stay_on_subset;
  KeepSubset keep_to_stay_on_verbal_subset;

  // Merge
  TurnSubset merge_subset;
  TurnSubset merge_verbal_subset;

  // EnterRoundabout
  EnterRoundaboutSubset enter_roundabout_subset;
  EnterRoundaboutSubset enter_roundabout_verbal_subset;

  // ExitRoundabout
  ContinueSubset exit_roundabout_subset;
  ContinueSubset exit_roundabout_verbal_subset;

  // EnterFerry
  EnterFerrySubset enter_ferry_subset;
  EnterFerrySubset enter_ferry_verbal_subset;

  // TransitConnectionStart
  TransitConnectionSubset transit_connection_start_subset;
  TransitConnectionSubset transit_connection_start_verbal_subset;

  // TransitConnectionTransfer
  TransitConnectionSubset transit_connection_transfer_subset;
  TransitConnectionSubset transit_connection_transfer_verbal_subset;

  // TransitConnectionDestination
  TransitConnectionSubset transit_connection_destination_subset;
  TransitConnectionSubset transit_connection_destination_verbal_subset;

  // Depart
  PhraseSet depart_subset;
  PhraseSet depart_verbal_subset;

  // Arrive
  PhraseSet arrive_subset;
  PhraseSet arrive_verbal_subset;

  // Transit
  TransitStopSubset transit_subset;
  TransitSubset transit_verbal_subset;

  // TransitRemainOn
  TransitStopSubset transit_remain_on_subset;
  TransitSubset transit_remain_on_verbal_subset;

  // TransitTransfer
  TransitStopSubset transit_transfer_subset;
  TransitSubset transit_transfer_verbal_subset;

  // Post transition verbal
  PostTransitionVerbalSubset post_transition_verbal_subset;

  // Post transition transit verbal
  PostTransitionTransitVerbalSubset post_transition_transit_verbal_subset;

  // Verbal multi-cue
  VerbalMultiCueSubset verbal_multi_cue_subset;

  // Approach verbal alert
  ApproachVerbalAlertSubset approach_verbal_alert_subset;

  // Pass
  PassSubset pass_subset;
  // Elevator
  PhraseSet elevator_subset;

  // Steps
  PhraseSet steps_subset;

  // Escalator
  PhraseSet escalator_subset;

  // Enter Building
  EnterBuildingSubset enter_building_subset;

  // Exit Building
  ExitBuildingSubset exit_building_subset;

  // Posix locale
  std::string posix_locale;

  /**
   * Returns the locale based on the posix_locale string from language file.
   *
   * @return the locale based on the posix_locale string from language file.
   */
  const std::locale& GetLocale() const;

  /**
   * Returns the language tag of this dictionary.
   *
   * @return the language tag of this dictionary.
   */
  const std::string& GetLanguageTag() const;

protected:
  /**
   * Loads this dictionary object with the localized narrative instructions
   * contained in the specified property tree.
   *
   * @param  narrative_pt  The narrative property tree with the localized
   *                       narrative instructions.
   */
  void Load(const boost::property_tree::ptree& narrative_pt);

  /**
   * Loads the phrases with the localized narrative instructions
   * contained in the specified property tree.
   *
   * @param  phrase_handle  The 'phrase' structure to populate.
   * @param  phrase_pt  The 'phrase' property tree.
   */
  void Load(PhraseSet& phrase_handle, const boost::property_tree::ptree& phrase_pt);

  /**
   * Loads the specified 'start' instruction subset with the localized narrative
   * instructions contained in the specified property tree.
   *
   * @param  start_handle  The 'start' structure to populate.
   * @param  start_subset_pt  The 'start' property tree.
   */
  void Load(StartSubset& start_handle, const boost::property_tree::ptree& start_subset_pt);
  /**
   * Loads the specified 'start verbal' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  start_verbal_handle  The 'start verbal' structure to populate.
   * @param  start_verbal_subset_pt  The 'start verbal' property tree.
   */
  void Load(StartVerbalSubset& start_verbal_handle,
            const boost::property_tree::ptree& start_verbal_subset_pt);

  /**
   * Loads the specified 'destination' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  destination_handle  The 'destination' structure to populate.
   * @param  destination_subset_pt  The 'destination' property tree.
   */
  void Load(DestinationSubset& destination_handle,
            const boost::property_tree::ptree& destination_subset_pt);

  /**
   * Loads the specified 'continue' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  continue_handle  The 'continue' structure to populate.
   * @param  continue_subset_pt  The 'continue' property tree.
   */
  void Load(ContinueSubset& continue_handle, const boost::property_tree::ptree& continue_subset_pt);

  /**
   * Loads the specified 'continue verbal' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  continue_verbal_handle  The 'continue verbal' structure to populate.
   * @param  continue_verbal_subset_pt  The 'continue verbal' property tree.
   */
  void Load(ContinueVerbalSubset& continue_verbal_handle,
            const boost::property_tree::ptree& continue_verbal_subset_pt);

  /**
   * Loads the specified 'turn' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  turn_handle  The 'turn' structure to populate.
   * @param  turn_subset_pt  The 'turn' property tree.
   */
  void Load(TurnSubset& turn_handle, const boost::property_tree::ptree& turn_subset_pt);

  /**
   * Loads the specified 'ramp' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  ramp_handle  The 'ramp' structure to populate.
   * @param  ramp_subset_pt  The 'ramp' property tree.
   */
  void Load(RampSubset& ramp_handle, const boost::property_tree::ptree& ramp_subset_pt);

  /**
   * Loads the specified 'keep' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  keep_handle  The 'keep' structure to populate.
   * @param  keep_subset_pt  The 'keep' property tree.
   */
  void Load(KeepSubset& keep_handle, const boost::property_tree::ptree& keep_subset_pt);

  /**
   * Loads the specified 'enter_roundabout' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  enter_roundabout_handle  The 'enter_roundabout' structure to populate.
   * @param  enter_roundabout_subset_pt  The 'enter_roundabout' property tree.
   */
  void Load(EnterRoundaboutSubset& enter_roundabout_handle,
            const boost::property_tree::ptree& enter_roundabout_subset_pt);

  /**
   * Loads the specified 'enter_ferry' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  enter_ferry_handle  The 'enter_ferry' structure to populate.
   * @param  enter_ferry_subset_pt  The 'enter_ferry' property tree.
   */
  void Load(EnterFerrySubset& enter_ferry_handle,
            const boost::property_tree::ptree& enter_ferry_subset_pt);

  /**
   * Loads the specified 'transit_connection' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  transit_connection_handle  The 'transit_connection' structure to populate.
   * @param  transit_connection_subset_pt  The 'transit_connection' property tree.
   */
  void Load(TransitConnectionSubset& transit_connection_handle,
            const boost::property_tree::ptree& transit_connection_subset_pt);

  /**
   * Loads the specified 'transit' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  transit_handle  The 'transit' structure to populate.
   * @param  transit_subset_pt  The 'transit' property tree.
   */
  void Load(TransitSubset& transit_handle, const boost::property_tree::ptree& transit_subset_pt);

  /**
   * Loads the specified 'transit_stop' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  transit_stop_handle  The 'transit_stop' structure to populate.
   * @param  transit_stop_subset_pt  The 'transit_stop' property tree.
   */
  void Load(TransitStopSubset& transit_stop_handle,
            const boost::property_tree::ptree& transit_stop_subset_pt);

  /**
   * Loads the specified 'post transition verbal' instruction subset with the
   * localized narrative instructions contained in the specified property tree.
   *
   * @param  post_transition_verbal_handle  The 'post transition verbal'
   *                                        structure to populate.
   * @param  post_transition_verbal_subset_pt  The 'post transition verbal'
   *                                           property tree.
   */
  void Load(PostTransitionVerbalSubset& post_transition_verbal_handle,
            const boost::property_tree::ptree& post_transition_verbal_subset_pt);

  /**
   * Loads the specified 'post transition_transit verbal' instruction subset with the
   * localized narrative instructions contained in the specified property tree.
   *
   * @param  post_transition_transit_verbal_handle  The 'post transition_transit verbal'
   *                                                structure to populate.
   * @param  post_transition_transit_verbal_subset_pt  The 'post transition_transit verbal'
   *                                                   property tree.
   */
  void Load(PostTransitionTransitVerbalSubset& post_transition_transit_verbal_handle,
            const boost::property_tree::ptree& post_transition_transit_verbal_subset_pt);

  /**
   * Loads the specified 'verbal multi cue' instruction subset with the
   * localized narrative instructions contained in the specified property tree.
   *
   * @param  verbal_multi_cue_handle  The 'verbal multi cue' structure to populate.
   * @param  verbal_multi_cue_subset_pt  The 'verbal multi cue' property tree.
   */
  void Load(VerbalMultiCueSubset& verbal_multi_cue_handle,
            const boost::property_tree::ptree& verbal_multi_cue_subset_pt);

  /**
   * Loads the specified 'approach verbal alert' instruction subset with the
   * localized narrative instructions contained in the specified property tree.
   *
   * @param  approach_verbal_alert_handle  The 'approach verbal alert' structure to populate.
   * @param  approach_verbal_alert_subset_pt  The 'approach verbal alert' property tree.
   */
  void Load(ApproachVerbalAlertSubset& approach_verbal_alert_handle,
            const boost::property_tree::ptree& approach_verbal_alert_subset_pt);

  /**
   * Loads the specified 'pass' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  pass_handle  The 'pass' structure to populate.
   * @param  pass_subset_pt  The 'pass' property tree.
   */
  void Load(PassSubset& pass_handle, const boost::property_tree::ptree& pass_subset_pt);

  /**
   * Loads the specified 'enter_building' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  enter_building_handle  The 'enter_building' structure to populate.
   * @param  enter_building_subset_pt  The 'enter_building' property tree.
   */
  void Load(EnterBuildingSubset& enter_building_handle,
            const boost::property_tree::ptree& enter_building_subset_pt);

  /**
   * Loads the specified 'exit_building' instruction subset with the localized
   * narrative instructions contained in the specified property tree.
   *
   * @param  exit_building_handle  The 'exit_building' structure to populate.
   * @param  exit_building_subset_pt  The 'exit_building' property tree.
   */
  void Load(ExitBuildingSubset& exit_building_handle,
            const boost::property_tree::ptree& exit_building_subset_pt);

  // Locale
  std::locale locale;

  // Language tag
  std::string language_tag;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_
