#ifndef VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_
#define VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_

#include <vector>
#include <string>
#include <unordered_map>

#include <boost/property_tree/ptree.hpp>

namespace {

// Subset keys
constexpr auto kStartKey = "instructions.start";
constexpr auto kStartVerbalKey = "instructions.start_verbal";
constexpr auto kDestinationKey = "instructions.destination";
constexpr auto kDestinationVerbalAlertKey = "instructions.destination_verbal_alert";
constexpr auto kDestinationVerbalKey = "instructions.destination_verbal";

// Variable keys
constexpr auto kPhrasesKey = "phrases";
constexpr auto kCardinalDirectionsKey = "cardinal_directions";
constexpr auto kRelativeDirectionsKey = "relative_directions";
constexpr auto kEmptyStreetNameLabelsKey = "empty_street_name_labels";
constexpr auto kEmptyBeginStreetNameLabelsKey = "empty_begin_street_name_labels";
constexpr auto kMetricLengthsKey = "metric_lengths";
constexpr auto kUsCustomaryLengthsKey = "us_customary_lengths";

// Phrase tags
constexpr auto kCardinalDirectionTag = "<CARDINAL_DIRECTION>";
constexpr auto kRelativelDirectionTag = "<RELATIVE_DIRECTION>";
constexpr auto kStreetNamesTag = "<STREET_NAMES>";
constexpr auto kBeginStreetNamesTag = "<BEGIN_STREET_NAMES>";
constexpr auto kLengthTag = "<LENGTH>";
constexpr auto kDestinationTag = "<DESTINATION>";

}

namespace valhalla {
namespace odin {

struct PhraseSet {
  std::unordered_map<std::string, std::string> phrases;
};

struct StartSubset : PhraseSet {
  std::vector<std::string> cardinal_directions;
  std::vector<std::string> empty_street_name_labels;
  std::vector<std::string> empty_begin_street_name_labels;
};

struct StartVerbalSubset : StartSubset {
  std::vector<std::string> metric_lengths;
  std::vector<std::string> us_customary_lengths;
};

struct DestinationSubset : PhraseSet {
  std::vector<std::string> relative_directions;
};

/**
 * A class that stores the localized narrative instructions.
 */
class NarrativeDictionary {
 public:
  NarrativeDictionary(const boost::property_tree::ptree& narrative_pt);

  StartSubset start_subset;
  StartVerbalSubset start_verbal_subset;

  DestinationSubset destination_subset;
  DestinationSubset destination_verbal_alert_subset;
  DestinationSubset destination_verbal_subset;

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
  void Load(StartSubset& start_handle,
            const boost::property_tree::ptree& start_subset_pt);
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

};

}
}

#endif  // VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_
