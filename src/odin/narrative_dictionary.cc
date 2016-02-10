#include <valhalla/midgard/logging.h>
#include <valhalla/odin/narrative_dictionary.h>
#include <boost/property_tree/ptree.hpp>

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

  LOG_TRACE("Populate start_subset...");
  // Populate start_subset
  Load(start_subset, narrative_pt.get_child(kStartKey));

  LOG_TRACE("Populate start_verbal_subset...");
  // Populate start_verbal_subset
  Load(start_verbal_subset, narrative_pt.get_child(kStartVerbalKey));

  LOG_TRACE("Populate destination_subset...");
  // Populate destination_subset
  Load(destination_subset, narrative_pt.get_child(kDestinationKey));

  LOG_TRACE("Populate destination_verbal_alert_subset...");
  // Populate destination_verbal_alert_subset
  Load(destination_verbal_alert_subset, narrative_pt.get_child(kDestinationVerbalAlertKey));

  LOG_TRACE("Populate destination_verbal_subset...");
  // Populate destination_verbal_subset
  Load(destination_verbal_subset, narrative_pt.get_child(kDestinationVerbalKey));
}

void NarrativeDictionary::Load(PhraseSet& phrase_handle,
                               const boost::property_tree::ptree& phrase_pt) {

  for (const auto& item : phrase_pt.get_child(kPhrasesKey)) {
    LOG_TRACE("read phrases...");
    LOG_TRACE("item.first=" + item.first);
    LOG_TRACE("item.second.get_value<std::string>())=" + item.second.get_value<std::string>());

    phrase_handle.phrases.emplace(item.first,
                               item.second.get_value<std::string>());
  }
}

void NarrativeDictionary::Load(
    StartSubset& start_handle,
    const boost::property_tree::ptree& start_subset_pt) {

  LOG_TRACE("Populate phrases...");
  // Populate phrases
  Load(static_cast<PhraseSet&>(start_handle), start_subset_pt);

  // Populate cardinal_directions
  start_handle.cardinal_directions = as_vector<std::string>(
      start_subset_pt, kCardinalDirectionsKey);

//  // Populate empty_street_name_labels
//  start_handle.empty_street_name_labels = as_vector<std::string>(
//      start_subset_pt, kEmptyStreetNameLabelsKey);
//
//  // Populate empty_begin_street_name_labels
//  start_handle.empty_begin_street_name_labels = as_vector<std::string>(
//      start_subset_pt, kEmptyBeginStreetNameLabelsKey);
}

void NarrativeDictionary::Load(
    StartVerbalSubset& start_verbal_handle,
    const boost::property_tree::ptree& start_verbal_subset_pt) {

  // Populate start_subset items
  Load(static_cast<StartSubset&>(start_verbal_handle), start_verbal_subset_pt);

//  // Populate metric_lengths
//  start_verbal_handle.metric_lengths = as_vector<std::string>(
//      start_verbal_subset_pt, kMetricLengthsKey);
//
//  // Populate us_customary_lengths
//  start_verbal_handle.us_customary_lengths = as_vector<std::string>(
//      start_verbal_subset_pt, kUsCustomaryLengthsKey);
}

void NarrativeDictionary::Load(
    DestinationSubset& destination_handle,
    const boost::property_tree::ptree& destination_subset_pt) {

  LOG_TRACE("Populate phrases...");
  // Populate phrases
  Load(static_cast<PhraseSet&>(destination_handle), destination_subset_pt);

  // Populate relative_directions
  destination_handle.relative_directions = as_vector<std::string>(
      destination_subset_pt, kRelativeDirectionsKey);
}

}
}
