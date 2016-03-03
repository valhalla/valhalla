#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/logging.h>

#include "odin/narrative_dictionary.h"

namespace {

// Read array as vector
template<typename T>
std::vector<T> as_vector(boost::property_tree::ptree const& pt,
                         boost::property_tree::ptree::key_type const& key) {
  std::vector<T> items;
  for (const auto& item : pt.get_child(key)) {
    items.push_back(item.second.get_value<T>());
  }
  return items;
}

}

namespace valhalla {
namespace odin {

NarrativeDictionary::NarrativeDictionary(
    const boost::property_tree::ptree& narrative_pt) {
  Load(narrative_pt);
}

void NarrativeDictionary::Load(
    const boost::property_tree::ptree& narrative_pt) {

  /////////////////////////////////////////////////////////////////////////////
  LOG_TRACE("Populate start_subset...");
  // Populate start_subset
  Load(start_subset, narrative_pt.get_child(kStartKey));

  LOG_TRACE("Populate start_verbal_subset...");
  // Populate start_verbal_subset
  Load(start_verbal_subset, narrative_pt.get_child(kStartVerbalKey));

  /////////////////////////////////////////////////////////////////////////////
  LOG_TRACE("Populate destination_subset...");
  // Populate destination_subset
  Load(destination_subset, narrative_pt.get_child(kDestinationKey));

  LOG_TRACE("Populate destination_verbal_alert_subset...");
  // Populate destination_verbal_alert_subset
  Load(destination_verbal_alert_subset, narrative_pt.get_child(kDestinationVerbalAlertKey));

  LOG_TRACE("Populate destination_verbal_subset...");
  // Populate destination_verbal_subset
  Load(destination_verbal_subset, narrative_pt.get_child(kDestinationVerbalKey));

  /////////////////////////////////////////////////////////////////////////////
  LOG_TRACE("Populate continue_subset...");
  // Populate continue_subset
  Load(continue_subset, narrative_pt.get_child(kContinueKey));

  LOG_TRACE("Populate continue_verbal_alert_subset...");
  // Populate continue_verbal_alert_subset
  Load(continue_verbal_alert_subset, narrative_pt.get_child(kContinueVerbalAlertKey));

  LOG_TRACE("Populate continue_verbal_subset...");
  // Populate continue_verbal_subset
  Load(continue_verbal_subset, narrative_pt.get_child(kContinueVerbalKey));

  /////////////////////////////////////////////////////////////////////////////
  LOG_TRACE("Populate bear_subset...");
  // Populate bear_subset
  Load(bear_subset, narrative_pt.get_child(kBearKey));

  LOG_TRACE("Populate bear_verbal_subset...");
  // Populate bear_verbal_subset
  Load(bear_verbal_subset, narrative_pt.get_child(kBearVerbalKey));

  /////////////////////////////////////////////////////////////////////////////
  LOG_TRACE("Populate turn_subset...");
  // Populate turn_subset
  Load(turn_subset, narrative_pt.get_child(kTurnKey));

  LOG_TRACE("Populate turn_verbal_subset...");
  // Populate turn_verbal_subset
  Load(turn_verbal_subset, narrative_pt.get_child(kTurnVerbalKey));

  /////////////////////////////////////////////////////////////////////////////
  LOG_TRACE("Populate post_transition_verbal_subset...");
  // Populate post_transition_verbal_subset
  Load(post_transition_verbal_subset, narrative_pt.get_child(kPostTransitionVerbalKey));

  /////////////////////////////////////////////////////////////////////////////
  LOG_TRACE("Populate verbal_multi_cue_subset...");
  // Populate verbal_multi_cue_subset
  Load(verbal_multi_cue_subset, narrative_pt.get_child(kVerbalMultiCueKey));

}

void NarrativeDictionary::Load(PhraseSet& phrase_handle,
                               const boost::property_tree::ptree& phrase_pt) {

  for (const auto& item : phrase_pt.get_child(kPhrasesKey)) {
    phrase_handle.phrases.emplace(item.first,
                                  item.second.get_value<std::string>());
  }
}

void NarrativeDictionary::Load(
    StartSubset& start_handle,
    const boost::property_tree::ptree& start_subset_pt) {

  // Populate phrases
  Load(static_cast<PhraseSet&>(start_handle), start_subset_pt);

  // Populate cardinal_directions
  start_handle.cardinal_directions = as_vector<std::string>(
      start_subset_pt, kCardinalDirectionsKey);

  // Populate empty_street_name_labels
  start_handle.empty_street_name_labels = as_vector<std::string>(
      start_subset_pt, kEmptyStreetNameLabelsKey);
}

void NarrativeDictionary::Load(
    StartVerbalSubset& start_verbal_handle,
    const boost::property_tree::ptree& start_verbal_subset_pt) {

  // Populate start_subset items
  Load(static_cast<StartSubset&>(start_verbal_handle), start_verbal_subset_pt);

  // Populate metric_lengths
  start_verbal_handle.metric_lengths = as_vector<std::string>(
      start_verbal_subset_pt, kMetricLengthsKey);

  // Populate us_customary_lengths
  start_verbal_handle.us_customary_lengths = as_vector<std::string>(
      start_verbal_subset_pt, kUsCustomaryLengthsKey);
}

void NarrativeDictionary::Load(
    DestinationSubset& destination_handle,
    const boost::property_tree::ptree& destination_subset_pt) {

  // Populate phrases
  Load(static_cast<PhraseSet&>(destination_handle), destination_subset_pt);

  // Populate relative_directions
  destination_handle.relative_directions = as_vector<std::string>(
      destination_subset_pt, kRelativeDirectionsKey);
}

void NarrativeDictionary::Load(
    ContinueSubset& continue_handle,
    const boost::property_tree::ptree& continue_subset_pt) {

  // Populate phrases
  Load(static_cast<PhraseSet&>(continue_handle), continue_subset_pt);

  // Populate empty_street_name_labels
  continue_handle.empty_street_name_labels = as_vector<std::string>(
      continue_subset_pt, kEmptyStreetNameLabelsKey);
}

void NarrativeDictionary::Load(
    ContinueVerbalSubset& continue_verbal_handle,
    const boost::property_tree::ptree& continue_verbal_subset_pt) {

  // Populate continue_subset items
  Load(static_cast<ContinueSubset&>(continue_verbal_handle), continue_verbal_subset_pt);

  // Populate metric_lengths
  continue_verbal_handle.metric_lengths = as_vector<std::string>(
      continue_verbal_subset_pt, kMetricLengthsKey);

  // Populate us_customary_lengths
  continue_verbal_handle.us_customary_lengths = as_vector<std::string>(
      continue_verbal_subset_pt, kUsCustomaryLengthsKey);
}

void NarrativeDictionary::Load(
    TurnSubset& turn_handle,
    const boost::property_tree::ptree& turn_subset_pt) {

  // Populate phrases
  Load(static_cast<PhraseSet&>(turn_handle), turn_subset_pt);

  // Populate relative_directions
  turn_handle.relative_directions = as_vector<std::string>(
      turn_subset_pt, kRelativeDirectionsKey);

  // Populate empty_street_name_labels
  turn_handle.empty_street_name_labels = as_vector<std::string>(
      turn_subset_pt, kEmptyStreetNameLabelsKey);
}

void NarrativeDictionary::Load(
    PostTransitionVerbalSubset& post_transition_verbal_handle,
    const boost::property_tree::ptree& post_transition_verbal_subset_pt) {

  // Populate phrases
  Load(static_cast<PhraseSet&>(post_transition_verbal_handle),
       post_transition_verbal_subset_pt);

  // Populate metric_lengths
  post_transition_verbal_handle.metric_lengths = as_vector<std::string>(
      post_transition_verbal_subset_pt, kMetricLengthsKey);

  // Populate us_customary_lengths
  post_transition_verbal_handle.us_customary_lengths = as_vector<std::string>(
      post_transition_verbal_subset_pt, kUsCustomaryLengthsKey);

  // Populate empty_street_name_labels
  post_transition_verbal_handle.empty_street_name_labels = as_vector<std::string>(
      post_transition_verbal_subset_pt, kEmptyStreetNameLabelsKey);
}

}
}
