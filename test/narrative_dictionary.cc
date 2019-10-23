#include <map>
#include <string>
#include <vector>

#include "odin/narrative_dictionary.h"
#include "odin/util.h"

#include "test.h"

using namespace valhalla::odin;

namespace {

// Expected strings
const std::vector<std::string> kExpectedEmptyStreetNameLabels = {"the walkway", "the cycleway",
                                                                 "the mountain bike trail"};
const std::vector<std::string> kExpectedCardinalDirections = {"north",     "northeast", "east",
                                                              "southeast", "south",     "southwest",
                                                              "west",      "northwest"};
const std::vector<std::string> kExpectedMetricLengths = {"<KILOMETERS> kilometers", "1 kilometer",
                                                         "a half kilometer", "<METERS> meters",
                                                         "less than 10 meters"};
const std::vector<std::string> kExpectedUsCustomaryLengths =
    {"<MILES> miles",     "1 mile",      "a half mile",      "<TENTHS_OF_MILE> tenths of a mile",
     "1 tenth of a mile", "<FEET> feet", "less than 10 feet"};
const std::vector<std::string> kExpectedRelativeTwoDirections = {"left", "right"};
const std::vector<std::string> kExpectedRelativeThreeDirections = {"left", "straight", "right"};
const std::vector<std::string> kExpectedOrdinalValues = {"1st", "2nd", "3rd", "4th", "5th",
                                                         "6th", "7th", "8th", "9th", "10th"};
const std::string kExpectedFerryLabel = "Ferry";
const std::string kExpectedStationLabel = "Station";
const std::vector<std::string> kExpectedEmptyTransitNameLabels = {"tram",    "metro",    "train",
                                                                  "bus",     "ferry",    "cable car",
                                                                  "gondola", "funicular"};
const std::map<std::string, std::string> kExpectedTransitStopCountLabels = {{"one", "stop"},
                                                                            {"other", "stops"}};

// Expected phrases
const std::map<std::string, std::string> kExpectedStartPhrases =
    {{"0", "Head <CARDINAL_DIRECTION>."},
     {"1", "Head <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"2", "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"4", "Drive <CARDINAL_DIRECTION>."},
     {"5", "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"6", "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"8", "Walk <CARDINAL_DIRECTION>."},
     {"9", "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"10", "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"16", "Bike <CARDINAL_DIRECTION>."},
     {"17", "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"18", "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedStartVerbalPhrases =
    {{"0", "Head <CARDINAL_DIRECTION> for <LENGTH>."},
     {"1", "Head <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>."},
     {"2", "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"4", "Drive <CARDINAL_DIRECTION> for <LENGTH>."},
     {"5", "Drive <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>."},
     {"6", "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"8", "Walk <CARDINAL_DIRECTION> for <LENGTH>."},
     {"9", "Walk <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>."},
     {"10", "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"16", "Bike <CARDINAL_DIRECTION> for <LENGTH>."},
     {"17", "Bike <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>."},
     {"18", "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedExitPhrases =
    {{"0", "Take the exit on the <RELATIVE_DIRECTION>."},
     {"1", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>."},
     {"2", "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>."},
     {"3", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>."},
     {"4", "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"5", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"6", "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"7", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward "
           "<TOWARD_SIGN>."},
     {"8", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>."},
     {"10", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>."},
     {"12", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"14", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward "
            "<TOWARD_SIGN>."},
     {"15", "Take the exit."},
     {"16", "Take exit <NUMBER_SIGN>."},
     {"17", "Take the <BRANCH_SIGN> exit."},
     {"18", "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN>."},
     {"19", "Take the exit toward <TOWARD_SIGN>."},
     {"20", "Take exit <NUMBER_SIGN> toward <TOWARD_SIGN>."},
     {"21", "Take the <BRANCH_SIGN> exit toward <TOWARD_SIGN>."},
     {"22", "Take exit <NUMBER_SIGN> onto <BRANCH_SIGN> toward <TOWARD_SIGN>."},
     {"23", "Take the <NAME_SIGN> exit."},
     {"25", "Take the <NAME_SIGN> exit onto <BRANCH_SIGN>."},
     {"27", "Take the <NAME_SIGN> exit toward <TOWARD_SIGN>."},
     {"29", "Take the <NAME_SIGN> exit onto <BRANCH_SIGN> toward <TOWARD_SIGN>."}};

const std::map<std::string, std::string> kExpectedExitVerbalPhrases =
    {{"0", "Take the exit on the <RELATIVE_DIRECTION>."},
     {"1", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION>."},
     {"2", "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION>."},
     {"3", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>."},
     {"4", "Take the exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"5", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"6", "Take the <BRANCH_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"7", "Take exit <NUMBER_SIGN> on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward "
           "<TOWARD_SIGN>."},
     {"8", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION>."},
     {"10", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN>."},
     {"12", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"14", "Take the <NAME_SIGN> exit on the <RELATIVE_DIRECTION> onto <BRANCH_SIGN> toward "
            "<TOWARD_SIGN>."}};

const std::map<std::string, std::string> kExpectedKeepPhrases =
    {{"0", "Keep <RELATIVE_DIRECTION> at the fork."},
     {"1", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>."},
     {"2", "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>."},
     {"3", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>."},
     {"4", "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"5", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>."},
     {"6", "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>."},
     {"7", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward "
           "<TOWARD_SIGN>."}};

const std::map<std::string, std::string> kExpectedKeepVerbalPhrases =
    {{"0", "Keep <RELATIVE_DIRECTION> at the fork."},
     {"1", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN>."},
     {"2", "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES>."},
     {"3", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES>."},
     {"4", "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."},
     {"5", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> toward <TOWARD_SIGN>."},
     {"6", "Keep <RELATIVE_DIRECTION> to take <STREET_NAMES> toward <TOWARD_SIGN>."},
     {"7", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> onto <STREET_NAMES> toward "
           "<TOWARD_SIGN>."}};

const std::map<std::string, std::string> kExpectedKeepToStayOnPhrases =
    {{"0", "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."},
     {"1", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>."},
     {"2", "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."},
     {"3", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward "
           "<TOWARD_SIGN>."}};

const std::map<std::string, std::string> kExpectedKeepToStayOnVerbalPhrases =
    {{"0", "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."},
     {"1", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES>."},
     {"2", "Keep <RELATIVE_DIRECTION> to stay on <STREET_NAMES> toward <TOWARD_SIGN>."},
     {"3", "Keep <RELATIVE_DIRECTION> to take exit <NUMBER_SIGN> to stay on <STREET_NAMES> toward "
           "<TOWARD_SIGN>."}};

const std::map<std::string, std::string> kExpectedMergePhrases =
    {{"0", "Merge."},
     {"1", "Merge <RELATIVE_DIRECTION>."},
     {"2", "Merge onto <STREET_NAMES>."},
     {"3", "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedMergeVerbalPhrases =
    {{"0", "Merge."},
     {"1", "Merge <RELATIVE_DIRECTION>."},
     {"2", "Merge onto <STREET_NAMES>."},
     {"3", "Merge <RELATIVE_DIRECTION> onto <STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedEnterRoundaboutPhrases =
    {{"0", "Enter the roundabout."},
     {"1", "Enter the roundabout and take the <ORDINAL_VALUE> exit."}};

const std::map<std::string, std::string> kExpectedEnterRoundaboutVerbalPhrases =
    {{"0", "Enter the roundabout."},
     {"1", "Enter the roundabout and take the <ORDINAL_VALUE> exit."}};

const std::map<std::string, std::string> kExpectedExitRoundaboutPhrases =
    {{"0", "Exit the roundabout."},
     {"1", "Exit the roundabout onto <STREET_NAMES>."},
     {"2", "Exit the roundabout onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedExitRoundaboutVerbalPhrases =
    {{"0", "Exit the roundabout."},
     {"1", "Exit the roundabout onto <STREET_NAMES>."},
     {"2", "Exit the roundabout onto <BEGIN_STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedEnterFerryPhrases =
    {{"0", "Take the Ferry."},
     {"1", "Take the <STREET_NAMES>."},
     {"2", "Take the <STREET_NAMES> <FERRY_LABEL>."}};

const std::map<std::string, std::string> kExpectedEnterFerryVerbalPhrases =
    {{"0", "Take the Ferry."},
     {"1", "Take the <STREET_NAMES>."},
     {"2", "Take the <STREET_NAMES> <FERRY_LABEL>."}};

const std::map<std::string, std::string> kExpectedExitFerryPhrases =
    {{"0", "Head <CARDINAL_DIRECTION>."},
     {"1", "Head <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"2", "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"4", "Drive <CARDINAL_DIRECTION>."},
     {"5", "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"6", "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"8", "Walk <CARDINAL_DIRECTION>."},
     {"9", "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"10", "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"16", "Bike <CARDINAL_DIRECTION>."},
     {"17", "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"18", "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedExitFerryVerbalPhrases =
    {{"0", "Head <CARDINAL_DIRECTION>."},
     {"1", "Head <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"2", "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"4", "Drive <CARDINAL_DIRECTION>."},
     {"5", "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"6", "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"8", "Walk <CARDINAL_DIRECTION>."},
     {"9", "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"10", "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"16", "Bike <CARDINAL_DIRECTION>."},
     {"17", "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"18", "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedTransitConnectionStartPhrases =
    {{"0", "Enter the station."},
     {"1", "Enter the <TRANSIT_STOP>."},
     {"2", "Enter the <TRANSIT_STOP> <STATION_LABEL>."}};

const std::map<std::string, std::string> kExpectedTransitConnectionStartVerbalPhrases =
    {{"0", "Enter the station."},
     {"1", "Enter the <TRANSIT_STOP>."},
     {"2", "Enter the <TRANSIT_STOP> <STATION_LABEL>."}};

const std::map<std::string, std::string> kExpectedTransitConnectionTransferPhrases =
    {{"0", "Transfer at the station."},
     {"1", "Transfer at the <TRANSIT_STOP>."},
     {"2", "Transfer at the <TRANSIT_STOP> <STATION_LABEL>."}};

const std::map<std::string, std::string> kExpectedTransitConnectionTransferVerbalPhrases =
    {{"0", "Transfer at the station."},
     {"1", "Transfer at the <TRANSIT_STOP>."},
     {"2", "Transfer at the <TRANSIT_STOP> <STATION_LABEL>."}};

const std::map<std::string, std::string> kExpectedTransitConnectionDestinationPhrases =
    {{"0", "Exit the station."},
     {"1", "Exit the <TRANSIT_STOP>."},
     {"2", "Exit the <TRANSIT_STOP> <STATION_LABEL>."}};

const std::map<std::string, std::string> kExpectedTransitConnectionDestinationVerbalPhrases =
    {{"0", "Exit the station."},
     {"1", "Exit the <TRANSIT_STOP>."},
     {"2", "Exit the <TRANSIT_STOP> <STATION_LABEL>."}};

const std::map<std::string, std::string> kExpectedDepartPhrases =
    {{"0", "Depart: <TIME>."}, {"1", "Depart: <TIME> from <TRANSIT_STOP>."}};

const std::map<std::string, std::string> kExpectedDepartVerbalPhrases =
    {{"0", "Depart at <TIME>."}, {"1", "Depart at <TIME> from <TRANSIT_STOP>."}};

const std::map<std::string, std::string> kExpectedArrivePhrases =
    {{"0", "Arrive: <TIME>."}, {"1", "Arrive: <TIME> at <TRANSIT_STOP>."}};

const std::map<std::string, std::string> kExpectedArriveVerbalPhrases =
    {{"0", "Arrive at <TIME>."}, {"1", "Arrive at <TIME> at <TRANSIT_STOP>."}};

const std::map<std::string, std::string> kExpectedPostTransitConnectionDestinationPhrases =
    {{"0", "Head <CARDINAL_DIRECTION>."},
     {"1", "Head <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"2", "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"4", "Drive <CARDINAL_DIRECTION>."},
     {"5", "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"6", "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"8", "Walk <CARDINAL_DIRECTION>."},
     {"9", "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"10", "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."},
     {"16", "Bike <CARDINAL_DIRECTION>."},
     {"17", "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"18", "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."}};

const std::map<std::string, std::string> kExpectedPostTransitConnectionDestinationVerbalPhrases =
    {{"0", "Head <CARDINAL_DIRECTION>."},
     {"1", "Head <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"2", "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"4", "Drive <CARDINAL_DIRECTION>."},
     {"5", "Drive <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"6", "Drive <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"8", "Walk <CARDINAL_DIRECTION>."},
     {"9", "Walk <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"10", "Walk <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."},
     {"16", "Bike <CARDINAL_DIRECTION>."},
     {"17", "Bike <CARDINAL_DIRECTION> on <STREET_NAMES>."},
     {"18", "Bike <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."}};

const NarrativeDictionary& GetNarrativeDictionary(const std::string& lang_tag) {
  // Get the locale dictionary
  const auto phrase_dictionary = get_locales().find(lang_tag);

  // If language tag is not found then throw error
  if (phrase_dictionary == get_locales().end()) {
    throw std::runtime_error("Invalid language tag.");
  }

  return *phrase_dictionary->second;
}

void validate(const std::string& test_target, const std::string& expected) {
  if (test_target != expected) {
    throw std::runtime_error("Invalid entry: " + test_target + "  |  expected: " + expected);
  }
}

void validate(const std::vector<std::string>& test_target, const std::vector<std::string>& expected) {
  if (test_target.size() != expected.size()) {
    throw std::runtime_error("Invalid item count: " + std::to_string(test_target.size()) +
                             "  |  expected: " + std::to_string(expected.size()));
  }

  for (auto test_target_item = test_target.begin(), expected_item = expected.begin();
       test_target_item != test_target.end(); ++test_target_item, ++expected_item) {
    if ((*test_target_item) != (*expected_item)) {
      throw std::runtime_error("Invalid entry: " + (*test_target_item) +
                               "  |  expected: " + (*expected_item));
    }
  }
}

void validate(const std::unordered_map<std::string, std::string>& test_target,
              const std::map<std::string, std::string>& expected) {

  for (const auto& expected_phrase : expected) {
    const auto& test_target_item = test_target.at(expected_phrase.first);
    if (test_target_item != expected_phrase.second) {
      throw std::runtime_error("Invalid entry: " + test_target_item +
                               "  |  expected: " + expected_phrase.second);
    }
  }
}

void test_en_US_start() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate start phrases
  validate(dictionary.start_subset.phrases, kExpectedStartPhrases);

  // cardinal_directions
  const auto& cardinal_directions = dictionary.start_subset.cardinal_directions;
  validate(cardinal_directions, kExpectedCardinalDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.start_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_start_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate start phrases
  validate(dictionary.start_verbal_subset.phrases, kExpectedStartVerbalPhrases);

  // cardinal_directions
  const auto& cardinal_directions = dictionary.start_verbal_subset.cardinal_directions;
  validate(cardinal_directions, kExpectedCardinalDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.start_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);

  // metric_lengths
  const auto& metric_lengths = dictionary.start_verbal_subset.metric_lengths;
  validate(metric_lengths, kExpectedMetricLengths);

  // us_customary_lengths
  const auto& us_customary_lengths = dictionary.start_verbal_subset.us_customary_lengths;
  validate(us_customary_lengths, kExpectedUsCustomaryLengths);
}

void test_en_US_destination() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "You have arrived at your destination.",
  const auto& phrase_0 = dictionary.destination_subset.phrases.at("0");
  validate(phrase_0, "You have arrived at your destination.");

  // "1": "You have arrived at <DESTINATION>.",
  const auto& phrase_1 = dictionary.destination_subset.phrases.at("1");
  validate(phrase_1, "You have arrived at <DESTINATION>.");

  // "2": "Your destination is on the <RELATIVE_DIRECTION>.",
  const auto& phrase_2 = dictionary.destination_subset.phrases.at("2");
  validate(phrase_2, "Your destination is on the <RELATIVE_DIRECTION>.");

  // "3": "<DESTINATION> is on the <RELATIVE_DIRECTION>."
  const auto& phrase_3 = dictionary.destination_subset.phrases.at("3");
  validate(phrase_3, "<DESTINATION> is on the <RELATIVE_DIRECTION>.");

  // relative_directions
  const auto& relative_directions = dictionary.destination_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);
}

void test_en_US_destination_verbal_alert() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "You will arrive at your destination.",
  const auto& phrase_0 = dictionary.destination_verbal_alert_subset.phrases.at("0");
  validate(phrase_0, "You will arrive at your destination.");

  // "1": "You will arrive at <DESTINATION>.",
  const auto& phrase_1 = dictionary.destination_verbal_alert_subset.phrases.at("1");
  validate(phrase_1, "You will arrive at <DESTINATION>.");

  // "2": "Your destination will be on the <RELATIVE_DIRECTION>.",
  const auto& phrase_2 = dictionary.destination_verbal_alert_subset.phrases.at("2");
  validate(phrase_2, "Your destination will be on the <RELATIVE_DIRECTION>.");

  // "3": "<DESTINATION> will be on the <RELATIVE_DIRECTION>."
  const auto& phrase_3 = dictionary.destination_verbal_alert_subset.phrases.at("3");
  validate(phrase_3, "<DESTINATION> will be on the <RELATIVE_DIRECTION>.");

  // relative_directions
  const auto& relative_directions = dictionary.destination_verbal_alert_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);
}

void test_en_US_destination_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "You have arrived at your destination.",
  const auto& phrase_0 = dictionary.destination_verbal_subset.phrases.at("0");
  validate(phrase_0, "You have arrived at your destination.");

  // "1": "You have arrived at <DESTINATION>.",
  const auto& phrase_1 = dictionary.destination_verbal_subset.phrases.at("1");
  validate(phrase_1, "You have arrived at <DESTINATION>.");

  // "2": "Your destination is on the <RELATIVE_DIRECTION>.",
  const auto& phrase_2 = dictionary.destination_verbal_subset.phrases.at("2");
  validate(phrase_2, "Your destination is on the <RELATIVE_DIRECTION>.");

  // "3": "<DESTINATION> is on the <RELATIVE_DIRECTION>."
  const auto& phrase_3 = dictionary.destination_verbal_subset.phrases.at("3");
  validate(phrase_3, "<DESTINATION> is on the <RELATIVE_DIRECTION>.");

  // relative_directions
  const auto& relative_directions = dictionary.destination_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);
}

void test_en_US_becomes() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "<PREVIOUS_STREET_NAMES> becomes <STREET_NAMES>.",
  const auto& phrase_0 = dictionary.becomes_subset.phrases.at("0");
  validate(phrase_0, "<PREVIOUS_STREET_NAMES> becomes <STREET_NAMES>.");
}

void test_en_US_becomes_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "<PREVIOUS_STREET_NAMES> becomes <STREET_NAMES>.",
  const auto& phrase_0 = dictionary.becomes_verbal_subset.phrases.at("0");
  validate(phrase_0, "<PREVIOUS_STREET_NAMES> becomes <STREET_NAMES>.");
}

void test_en_US_continue() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Continue.",
  const auto& phrase_0 = dictionary.continue_subset.phrases.at("0");
  validate(phrase_0, "Continue.");

  // "1": "Continue on <STREET_NAMES>."
  const auto& phrase_1 = dictionary.continue_subset.phrases.at("1");
  validate(phrase_1, "Continue on <STREET_NAMES>.");

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.continue_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_continue_verbal_alert() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Continue.",
  const auto& phrase_0 = dictionary.continue_verbal_alert_subset.phrases.at("0");
  validate(phrase_0, "Continue.");

  // "1": "Continue on <STREET_NAMES>."
  const auto& phrase_1 = dictionary.continue_verbal_alert_subset.phrases.at("1");
  validate(phrase_1, "Continue on <STREET_NAMES>.");

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels =
      dictionary.continue_verbal_alert_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_continue_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Continue for <LENGTH>.",
  const auto& phrase_0 = dictionary.continue_verbal_subset.phrases.at("0");
  validate(phrase_0, "Continue for <LENGTH>.");

  // "1": "Continue on <STREET_NAMES> for <LENGTH>."
  const auto& phrase_1 = dictionary.continue_verbal_subset.phrases.at("1");
  validate(phrase_1, "Continue on <STREET_NAMES> for <LENGTH>.");

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.continue_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);

  // metric_lengths
  const auto& metric_lengths = dictionary.continue_verbal_subset.metric_lengths;
  validate(metric_lengths, kExpectedMetricLengths);

  // us_customary_lengths
  const auto& us_customary_lengths = dictionary.continue_verbal_subset.us_customary_lengths;
  validate(us_customary_lengths, kExpectedUsCustomaryLengths);
}

void test_en_US_bear() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Bear <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.bear_subset.phrases.at("0");
  validate(phrase_0, "Bear <RELATIVE_DIRECTION>.");

  // "1": "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.bear_subset.phrases.at("1");
  validate(phrase_1, "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES>.");

  // "2": "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",
  const auto& phrase_2 = dictionary.bear_subset.phrases.at("2");
  validate(phrase_2,
           "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.");

  // "3": "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  const auto& phrase_3 = dictionary.bear_subset.phrases.at("3");
  validate(phrase_3, "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.bear_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.bear_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_bear_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Bear <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.bear_verbal_subset.phrases.at("0");
  validate(phrase_0, "Bear <RELATIVE_DIRECTION>.");

  // "1": "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.bear_verbal_subset.phrases.at("1");
  validate(phrase_1, "Bear <RELATIVE_DIRECTION> onto <STREET_NAMES>.");

  // "2": "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.",
  const auto& phrase_2 = dictionary.bear_verbal_subset.phrases.at("2");
  validate(phrase_2, "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.");

  // "3": "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  const auto& phrase_3 = dictionary.bear_verbal_subset.phrases.at("3");
  validate(phrase_3, "Bear <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.bear_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.bear_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_turn() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Turn <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.turn_subset.phrases.at("0");
  validate(phrase_0, "Turn <RELATIVE_DIRECTION>.");

  // "1": "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.turn_subset.phrases.at("1");
  validate(phrase_1, "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>.");

  // "2": "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",
  const auto& phrase_2 = dictionary.turn_subset.phrases.at("2");
  validate(phrase_2,
           "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.");

  // "3": "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  const auto& phrase_3 = dictionary.turn_subset.phrases.at("3");
  validate(phrase_3, "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.turn_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.turn_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_turn_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Turn <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.turn_verbal_subset.phrases.at("0");
  validate(phrase_0, "Turn <RELATIVE_DIRECTION>.");

  // "1": "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.turn_verbal_subset.phrases.at("1");
  validate(phrase_1, "Turn <RELATIVE_DIRECTION> onto <STREET_NAMES>.");

  // "2": "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.",
  const auto& phrase_2 = dictionary.turn_verbal_subset.phrases.at("2");
  validate(phrase_2, "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.");

  // "3": "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  const auto& phrase_3 = dictionary.turn_verbal_subset.phrases.at("3");
  validate(phrase_3, "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.turn_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.turn_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_sharp() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Turn sharp <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.sharp_subset.phrases.at("0");
  validate(phrase_0, "Turn sharp <RELATIVE_DIRECTION>.");

  // "1": "Turn sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.sharp_subset.phrases.at("1");
  validate(phrase_1, "Turn sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.");

  // "2": "Turn sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.",
  const auto& phrase_2 = dictionary.sharp_subset.phrases.at("2");
  validate(phrase_2,
           "Turn sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.");

  // "3": "Turn sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  const auto& phrase_3 = dictionary.sharp_subset.phrases.at("3");
  validate(phrase_3, "Turn sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.sharp_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.sharp_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_sharp_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Turn sharp <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.sharp_verbal_subset.phrases.at("0");
  validate(phrase_0, "Turn sharp <RELATIVE_DIRECTION>.");

  // "1": "Turn sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.sharp_verbal_subset.phrases.at("1");
  validate(phrase_1, "Turn sharp <RELATIVE_DIRECTION> onto <STREET_NAMES>.");

  // "2": "Turn sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.",
  const auto& phrase_2 = dictionary.sharp_verbal_subset.phrases.at("2");
  validate(phrase_2, "Turn sharp <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>.");

  // "3": "Turn sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  const auto& phrase_3 = dictionary.sharp_verbal_subset.phrases.at("3");
  validate(phrase_3, "Turn sharp <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.sharp_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.sharp_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_uturn() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  const auto& phrase_0 = dictionary.uturn_subset.phrases.at("0");
  validate(phrase_0, "Make a <RELATIVE_DIRECTION> U-turn.");

  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.uturn_subset.phrases.at("1");
  validate(phrase_1, "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.");

  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  const auto& phrase_2 = dictionary.uturn_subset.phrases.at("2");
  validate(phrase_2, "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.");

  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.",
  const auto& phrase_3 = dictionary.uturn_subset.phrases.at("3");
  validate(phrase_3, "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.");

  // "4": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.",
  const auto& phrase_4 = dictionary.uturn_subset.phrases.at("4");
  validate(phrase_4,
           "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.");

  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
  const auto& phrase_5 = dictionary.uturn_subset.phrases.at("5");
  validate(phrase_5,
           "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.uturn_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.uturn_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_uturn_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  const auto& phrase_0 = dictionary.uturn_verbal_subset.phrases.at("0");
  validate(phrase_0, "Make a <RELATIVE_DIRECTION> U-turn.");

  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.uturn_verbal_subset.phrases.at("1");
  validate(phrase_1, "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.");

  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  const auto& phrase_2 = dictionary.uturn_verbal_subset.phrases.at("2");
  validate(phrase_2, "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.");

  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.",
  const auto& phrase_3 = dictionary.uturn_verbal_subset.phrases.at("3");
  validate(phrase_3, "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.");

  // "4": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.",
  const auto& phrase_4 = dictionary.uturn_verbal_subset.phrases.at("4");
  validate(phrase_4,
           "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.");

  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
  const auto& phrase_5 = dictionary.uturn_verbal_subset.phrases.at("5");
  validate(phrase_5,
           "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.uturn_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.uturn_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_ramp_straight() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  //  "0": "Stay straight to take the ramp.",
  const auto& phrase_0 = dictionary.ramp_straight_subset.phrases.at("0");
  validate(phrase_0, "Stay straight to take the ramp.");

  //  "1": "Stay straight to take the <BRANCH_SIGN> ramp.",
  const auto& phrase_1 = dictionary.ramp_straight_subset.phrases.at("1");
  validate(phrase_1, "Stay straight to take the <BRANCH_SIGN> ramp.");

  //  "2": "Stay straight to take the ramp toward <TOWARD_SIGN>.",
  const auto& phrase_2 = dictionary.ramp_straight_subset.phrases.at("2");
  validate(phrase_2, "Stay straight to take the ramp toward <TOWARD_SIGN>.");

  //  "3": "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  const auto& phrase_3 = dictionary.ramp_straight_subset.phrases.at("3");
  validate(phrase_3, "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  //  "4": "Stay straight to take the <NAME_SIGN> ramp."
  const auto& phrase_4 = dictionary.ramp_straight_subset.phrases.at("4");
  validate(phrase_4, "Stay straight to take the <NAME_SIGN> ramp.");
}

void test_en_US_ramp_straight_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  //  "0": "Stay straight to take the ramp.",
  const auto& phrase_0 = dictionary.ramp_straight_verbal_subset.phrases.at("0");
  validate(phrase_0, "Stay straight to take the ramp.");

  //  "1": "Stay straight to take the <BRANCH_SIGN> ramp.",
  const auto& phrase_1 = dictionary.ramp_straight_verbal_subset.phrases.at("1");
  validate(phrase_1, "Stay straight to take the <BRANCH_SIGN> ramp.");

  //  "2": "Stay straight to take the ramp toward <TOWARD_SIGN>.",
  const auto& phrase_2 = dictionary.ramp_straight_verbal_subset.phrases.at("2");
  validate(phrase_2, "Stay straight to take the ramp toward <TOWARD_SIGN>.");

  //  "3": "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  const auto& phrase_3 = dictionary.ramp_straight_verbal_subset.phrases.at("3");
  validate(phrase_3, "Stay straight to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  //  "4": "Stay straight to take the <NAME_SIGN> ramp."
  const auto& phrase_4 = dictionary.ramp_straight_verbal_subset.phrases.at("4");
  validate(phrase_4, "Stay straight to take the <NAME_SIGN> ramp.");
}

void test_en_US_ramp() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.ramp_subset.phrases.at("0");
  validate(phrase_0, "Take the ramp on the <RELATIVE_DIRECTION>.");

  // "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  const auto& phrase_1 = dictionary.ramp_subset.phrases.at("1");
  validate(phrase_1, "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.");

  // "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  const auto& phrase_2 = dictionary.ramp_subset.phrases.at("2");
  validate(phrase_2, "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.");

  // "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  const auto& phrase_3 = dictionary.ramp_subset.phrases.at("3");
  validate(phrase_3, "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.");

  // "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  const auto& phrase_4 = dictionary.ramp_subset.phrases.at("4");
  validate(phrase_4, "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.");

  // "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
  const auto& phrase_5 = dictionary.ramp_subset.phrases.at("5");
  validate(phrase_5, "Turn <RELATIVE_DIRECTION> to take the ramp.");

  // "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
  const auto& phrase_6 = dictionary.ramp_subset.phrases.at("6");
  validate(phrase_6, "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.");

  // "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
  const auto& phrase_7 = dictionary.ramp_subset.phrases.at("7");
  validate(phrase_7, "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.");

  // "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  const auto& phrase_8 = dictionary.ramp_subset.phrases.at("8");
  validate(phrase_8,
           "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  // "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
  const auto& phrase_9 = dictionary.ramp_subset.phrases.at("9");
  validate(phrase_9, "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp.");

  // "10": "Take the ramp."
  const auto& phrase_10 = dictionary.ramp_subset.phrases.at("10");
  validate(phrase_10, "Take the ramp.");

  // "11": "Take the <BRANCH_SIGN> ramp."
  const auto& phrase_11 = dictionary.ramp_subset.phrases.at("11");
  validate(phrase_11, "Take the <BRANCH_SIGN> ramp.");

  // "12": "Take the ramp toward <TOWARD_SIGN>."
  const auto& phrase_12 = dictionary.ramp_subset.phrases.at("12");
  validate(phrase_12, "Take the ramp toward <TOWARD_SIGN>.");

  // "13": "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
  const auto& phrase_13 = dictionary.ramp_subset.phrases.at("13");
  validate(phrase_13, "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  // "14": "Take the <NAME_SIGN> ramp."
  const auto& phrase_14 = dictionary.ramp_subset.phrases.at("14");
  validate(phrase_14, "Take the <NAME_SIGN> ramp.");

  // relative_directions
  const auto& relative_directions = dictionary.ramp_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);
}

void test_en_US_ramp_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Take the ramp on the <RELATIVE_DIRECTION>.",
  const auto& phrase_0 = dictionary.ramp_verbal_subset.phrases.at("0");
  validate(phrase_0, "Take the ramp on the <RELATIVE_DIRECTION>.");

  // "1": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  const auto& phrase_1 = dictionary.ramp_verbal_subset.phrases.at("1");
  validate(phrase_1, "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION>.");

  // "2": "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  const auto& phrase_2 = dictionary.ramp_verbal_subset.phrases.at("2");
  validate(phrase_2, "Take the ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.");

  // "3": "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.",
  const auto& phrase_3 = dictionary.ramp_verbal_subset.phrases.at("3");
  validate(phrase_3, "Take the <BRANCH_SIGN> ramp on the <RELATIVE_DIRECTION> toward <TOWARD_SIGN>.");

  // "4": "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.",
  const auto& phrase_4 = dictionary.ramp_verbal_subset.phrases.at("4");
  validate(phrase_4, "Take the <NAME_SIGN> ramp on the <RELATIVE_DIRECTION>.");

  // "5": "Turn <RELATIVE_DIRECTION> to take the ramp.",
  const auto& phrase_5 = dictionary.ramp_verbal_subset.phrases.at("5");
  validate(phrase_5, "Turn <RELATIVE_DIRECTION> to take the ramp.");

  // "6": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.",
  const auto& phrase_6 = dictionary.ramp_verbal_subset.phrases.at("6");
  validate(phrase_6, "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp.");

  // "7": "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.",
  const auto& phrase_7 = dictionary.ramp_verbal_subset.phrases.at("7");
  validate(phrase_7, "Turn <RELATIVE_DIRECTION> to take the ramp toward <TOWARD_SIGN>.");

  // "8": "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.",
  const auto& phrase_8 = dictionary.ramp_verbal_subset.phrases.at("8");
  validate(phrase_8,
           "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  // "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
  const auto& phrase_9 = dictionary.ramp_verbal_subset.phrases.at("9");
  validate(phrase_9, "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp.");

  // "10": "Take the ramp."
  const auto& phrase_10 = dictionary.ramp_verbal_subset.phrases.at("10");
  validate(phrase_10, "Take the ramp.");

  // "11": "Take the <BRANCH_SIGN> ramp."
  const auto& phrase_11 = dictionary.ramp_verbal_subset.phrases.at("11");
  validate(phrase_11, "Take the <BRANCH_SIGN> ramp.");

  // "12": "Take the ramp toward <TOWARD_SIGN>."
  const auto& phrase_12 = dictionary.ramp_verbal_subset.phrases.at("12");
  validate(phrase_12, "Take the ramp toward <TOWARD_SIGN>.");

  // "13": "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>."
  const auto& phrase_13 = dictionary.ramp_verbal_subset.phrases.at("13");
  validate(phrase_13, "Take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  // "14": "Take the <NAME_SIGN> ramp."
  const auto& phrase_14 = dictionary.ramp_verbal_subset.phrases.at("14");
  validate(phrase_14, "Take the <NAME_SIGN> ramp.");

  // relative_directions
  const auto& relative_directions = dictionary.ramp_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);
}

void test_en_US_exit() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate exit phrases
  validate(dictionary.exit_subset.phrases, kExpectedExitPhrases);

  // relative_directions
  const auto& relative_directions = dictionary.exit_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);
}

void test_en_US_exit_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate exit_verbal phrases
  validate(dictionary.exit_verbal_subset.phrases, kExpectedExitVerbalPhrases);

  // relative_directions
  const auto& relative_directions = dictionary.exit_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);
}

void test_en_US_keep() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate keep phrases
  validate(dictionary.keep_subset.phrases, kExpectedKeepPhrases);

  // relative_directions
  const auto& relative_directions = dictionary.keep_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeThreeDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.keep_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_keep_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate keep_verbal phrases
  validate(dictionary.keep_verbal_subset.phrases, kExpectedKeepVerbalPhrases);

  // relative_directions
  const auto& relative_directions = dictionary.keep_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeThreeDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.keep_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_keep_to_stay_on() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate keep_to_stay_on phrases
  validate(dictionary.keep_to_stay_on_subset.phrases, kExpectedKeepToStayOnPhrases);

  // relative_directions
  const auto& relative_directions = dictionary.keep_to_stay_on_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeThreeDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.keep_to_stay_on_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_keep_to_stay_on_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate keep_to_stay_on_verbal phrases
  validate(dictionary.keep_to_stay_on_verbal_subset.phrases, kExpectedKeepToStayOnVerbalPhrases);

  // relative_directions
  const auto& relative_directions = dictionary.keep_to_stay_on_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeThreeDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels =
      dictionary.keep_to_stay_on_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_merge() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate merge phrases
  validate(dictionary.merge_subset.phrases, kExpectedMergePhrases);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.merge_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_merge_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate merge_verbal phrases
  validate(dictionary.merge_verbal_subset.phrases, kExpectedMergeVerbalPhrases);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.merge_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_enter_roundabout() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate enter_roundabout phrases
  validate(dictionary.enter_roundabout_subset.phrases, kExpectedEnterRoundaboutPhrases);

  // ordinal_values: "1st", "2nd", "3rd", "4th", "5th", "6th", "7th", "8th", "9th", "10th"
  const auto& ordinal_values = dictionary.enter_roundabout_subset.ordinal_values;
  validate(ordinal_values, kExpectedOrdinalValues);
}

void test_en_US_enter_roundabout_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate enter_roundabout_verbal phrases
  validate(dictionary.enter_roundabout_verbal_subset.phrases, kExpectedEnterRoundaboutVerbalPhrases);

  // ordinal_values: "1st", "2nd", "3rd", "4th", "5th", "6th", "7th", "8th", "9th", "10th"
  const auto& ordinal_values = dictionary.enter_roundabout_verbal_subset.ordinal_values;
  validate(ordinal_values, kExpectedOrdinalValues);
}

void test_en_US_exit_roundabout() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate exit_roundabout phrases
  validate(dictionary.exit_roundabout_subset.phrases, kExpectedExitRoundaboutPhrases);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.exit_roundabout_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_exit_roundabout_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate exit_roundabout_verbal phrases
  validate(dictionary.exit_roundabout_verbal_subset.phrases, kExpectedExitRoundaboutVerbalPhrases);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels =
      dictionary.exit_roundabout_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_enter_ferry() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate enter_ferry phrases
  validate(dictionary.enter_ferry_subset.phrases, kExpectedEnterFerryPhrases);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.enter_ferry_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);

  // Ferry label
  validate(dictionary.enter_ferry_subset.ferry_label, kExpectedFerryLabel);
}

void test_en_US_enter_ferry_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate enter_ferry_verbal phrases
  validate(dictionary.enter_ferry_verbal_subset.phrases, kExpectedEnterFerryVerbalPhrases);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels =
      dictionary.enter_ferry_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);

  // Ferry label
  validate(dictionary.enter_ferry_verbal_subset.ferry_label, kExpectedFerryLabel);
}

void test_en_US_exit_ferry() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate exit_ferry phrases
  validate(dictionary.exit_ferry_subset.phrases, kExpectedExitFerryPhrases);

  // cardinal_directions
  const auto& cardinal_directions = dictionary.exit_ferry_subset.cardinal_directions;
  validate(cardinal_directions, kExpectedCardinalDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.exit_ferry_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_exit_ferry_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate exit_ferry_verbal phrases
  validate(dictionary.exit_ferry_verbal_subset.phrases, kExpectedExitFerryVerbalPhrases);

  // cardinal_directions
  const auto& cardinal_directions = dictionary.exit_ferry_verbal_subset.cardinal_directions;
  validate(cardinal_directions, kExpectedCardinalDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.exit_ferry_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_transit_connection_start() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate transit_connection_start phrases
  validate(dictionary.transit_connection_start_subset.phrases,
           kExpectedTransitConnectionStartPhrases);

  // Station label
  validate(dictionary.transit_connection_start_subset.station_label, kExpectedStationLabel);
}

void test_en_US_transit_connection_start_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate transit_connection_start_verbal phrases
  validate(dictionary.transit_connection_start_verbal_subset.phrases,
           kExpectedTransitConnectionStartVerbalPhrases);

  // Station label
  validate(dictionary.transit_connection_start_verbal_subset.station_label, kExpectedStationLabel);
}

void test_en_US_transit_connection_transfer() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate transit_connection_start phrases
  validate(dictionary.transit_connection_transfer_subset.phrases,
           kExpectedTransitConnectionTransferPhrases);

  // Station label
  validate(dictionary.transit_connection_transfer_subset.station_label, kExpectedStationLabel);
}

void test_en_US_transit_connection_transfer_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate transit_connection_start_verbal phrases
  validate(dictionary.transit_connection_transfer_verbal_subset.phrases,
           kExpectedTransitConnectionTransferVerbalPhrases);

  // Station label
  validate(dictionary.transit_connection_transfer_verbal_subset.station_label, kExpectedStationLabel);
}

void test_en_US_transit_connection_destination() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate transit_destination_start phrases
  validate(dictionary.transit_connection_destination_subset.phrases,
           kExpectedTransitConnectionDestinationPhrases);

  // Station label
  validate(dictionary.transit_connection_destination_subset.station_label, kExpectedStationLabel);
}

void test_en_US_transit_connection_destination_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate transit_destination_start_verbal phrases
  validate(dictionary.transit_connection_destination_verbal_subset.phrases,
           kExpectedTransitConnectionDestinationVerbalPhrases);

  // Station label
  validate(dictionary.transit_connection_destination_verbal_subset.station_label,
           kExpectedStationLabel);
}

void test_en_US_depart() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate depart phrases
  validate(dictionary.depart_subset.phrases, kExpectedDepartPhrases);
}

void test_en_US_depart_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate depart_verbal phrases
  validate(dictionary.depart_verbal_subset.phrases, kExpectedDepartVerbalPhrases);
}

void test_en_US_arrive() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate arrive phrases
  validate(dictionary.arrive_subset.phrases, kExpectedArrivePhrases);
}

void test_en_US_arrive_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate arrive_verbal phrases
  validate(dictionary.arrive_verbal_subset.phrases, kExpectedArriveVerbalPhrases);
}

void test_en_US_transit() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  const auto& phrase_0 = dictionary.transit_subset.phrases.at("0");
  validate(phrase_0, "Take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)");

  const auto& phrase_1 = dictionary.transit_subset.phrases.at("1");
  validate(phrase_1, "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. (<TRANSIT_STOP_COUNT> "
                     "<TRANSIT_STOP_COUNT_LABEL>)");

  // empty_transit_name_labels
  const auto& empty_transit_name_labels = dictionary.transit_subset.empty_transit_name_labels;
  validate(empty_transit_name_labels, kExpectedEmptyTransitNameLabels);

  // transit_stop_count_labels
  const auto& transit_stop_count_labels = dictionary.transit_subset.transit_stop_count_labels;
  validate(transit_stop_count_labels, kExpectedTransitStopCountLabels);
}

void test_en_US_transit_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  const auto& phrase_0 = dictionary.transit_verbal_subset.phrases.at("0");
  validate(phrase_0, "Take the <TRANSIT_NAME>.");

  const auto& phrase_1 = dictionary.transit_verbal_subset.phrases.at("1");
  validate(phrase_1, "Take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>.");

  // empty_transit_name_labels
  const auto& empty_transit_name_labels = dictionary.transit_subset.empty_transit_name_labels;
  validate(empty_transit_name_labels, kExpectedEmptyTransitNameLabels);
}

void test_en_US_transit_remain_on() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  const auto& phrase_0 = dictionary.transit_remain_on_subset.phrases.at("0");
  validate(phrase_0,
           "Remain on the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)");

  const auto& phrase_1 = dictionary.transit_remain_on_subset.phrases.at("1");
  validate(phrase_1, "Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. "
                     "(<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)");

  // empty_transit_name_labels
  const auto& empty_transit_name_labels = dictionary.transit_subset.empty_transit_name_labels;
  validate(empty_transit_name_labels, kExpectedEmptyTransitNameLabels);

  // transit_stop_count_labels
  const auto& transit_stop_count_labels =
      dictionary.transit_remain_on_subset.transit_stop_count_labels;
  validate(transit_stop_count_labels, kExpectedTransitStopCountLabels);
}

void test_en_US_transit_remain_on_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  const auto& phrase_0 = dictionary.transit_remain_on_verbal_subset.phrases.at("0");
  validate(phrase_0, "Remain on the <TRANSIT_NAME>.");

  const auto& phrase_1 = dictionary.transit_remain_on_verbal_subset.phrases.at("1");
  validate(phrase_1, "Remain on the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>.");

  // empty_transit_name_labels
  const auto& empty_transit_name_labels = dictionary.transit_subset.empty_transit_name_labels;
  validate(empty_transit_name_labels, kExpectedEmptyTransitNameLabels);
}

void test_en_US_transit_transfer() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  const auto& phrase_0 = dictionary.transit_transfer_subset.phrases.at("0");
  validate(phrase_0,
           "Transfer to take the <TRANSIT_NAME>. (<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)");

  const auto& phrase_1 = dictionary.transit_transfer_subset.phrases.at("1");
  validate(phrase_1, "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>. "
                     "(<TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>)");

  // empty_transit_name_labels
  const auto& empty_transit_name_labels = dictionary.transit_subset.empty_transit_name_labels;
  validate(empty_transit_name_labels, kExpectedEmptyTransitNameLabels);

  // transit_stop_count_labels
  const auto& transit_stop_count_labels =
      dictionary.transit_transfer_subset.transit_stop_count_labels;
  validate(transit_stop_count_labels, kExpectedTransitStopCountLabels);
}

void test_en_US_transit_transfer_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  const auto& phrase_0 = dictionary.transit_transfer_verbal_subset.phrases.at("0");
  validate(phrase_0, "Transfer to take the <TRANSIT_NAME>.");

  const auto& phrase_1 = dictionary.transit_transfer_verbal_subset.phrases.at("1");
  validate(phrase_1, "Transfer to take the <TRANSIT_NAME> toward <TRANSIT_HEADSIGN>.");

  // empty_transit_name_labels
  const auto& empty_transit_name_labels = dictionary.transit_subset.empty_transit_name_labels;
  validate(empty_transit_name_labels, kExpectedEmptyTransitNameLabels);
}

void test_en_US_post_transit_connection_destination() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate post_transit_connection_destination phrases
  validate(dictionary.post_transit_connection_destination_subset.phrases,
           kExpectedPostTransitConnectionDestinationPhrases);

  // cardinal_directions
  const auto& cardinal_directions =
      dictionary.post_transit_connection_destination_subset.cardinal_directions;
  validate(cardinal_directions, kExpectedCardinalDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels =
      dictionary.post_transit_connection_destination_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_post_transit_connection_destination_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // Validate post_transit_connection_destination_verbal phrases
  validate(dictionary.post_transit_connection_destination_verbal_subset.phrases,
           kExpectedPostTransitConnectionDestinationVerbalPhrases);

  // cardinal_directions
  const auto& cardinal_directions =
      dictionary.post_transit_connection_destination_verbal_subset.cardinal_directions;
  validate(cardinal_directions, kExpectedCardinalDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels =
      dictionary.post_transit_connection_destination_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_post_transition_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Continue for <LENGTH>.",
  const auto& phrase_0 = dictionary.post_transition_verbal_subset.phrases.at("0");
  validate(phrase_0, "Continue for <LENGTH>.");

  // "1": "Continue on <STREET_NAMES> for <LENGTH>."
  const auto& phrase_1 = dictionary.post_transition_verbal_subset.phrases.at("1");
  validate(phrase_1, "Continue on <STREET_NAMES> for <LENGTH>.");

  // metric_lengths
  const auto& metric_lengths = dictionary.post_transition_verbal_subset.metric_lengths;
  validate(metric_lengths, kExpectedMetricLengths);

  // us_customary_lengths
  const auto& us_customary_lengths = dictionary.post_transition_verbal_subset.us_customary_lengths;
  validate(us_customary_lengths, kExpectedUsCustomaryLengths);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels =
      dictionary.post_transition_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);
}

void test_en_US_post_transition_transit_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Continue for <LENGTH>.",
  const auto& phrase_0 = dictionary.post_transition_transit_verbal_subset.phrases.at("0");
  validate(phrase_0, "Travel <TRANSIT_STOP_COUNT> <TRANSIT_STOP_COUNT_LABEL>.");

  // transit_stop_count_labels
  const auto& transit_stop_count_labels =
      dictionary.post_transition_transit_verbal_subset.transit_stop_count_labels;
  validate(transit_stop_count_labels, kExpectedTransitStopCountLabels);
}

void test_en_US_verbal_multi_cue() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>"
  const auto& phrase_0 = dictionary.verbal_multi_cue_subset.phrases.at("0");
  validate(phrase_0, "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>");
}

} // namespace

int main() {
  test::suite suite("narrative_dictionary");

  // test the en-US start phrases
  suite.test(TEST_CASE(test_en_US_start));

  // test the en-US start verbal phrases
  suite.test(TEST_CASE(test_en_US_start_verbal));

  // test the en-US destination phrases
  suite.test(TEST_CASE(test_en_US_destination));

  // test the en-US destination verbal alert_phrases
  suite.test(TEST_CASE(test_en_US_destination_verbal_alert));

  // test the en-US destination verbal phrases
  suite.test(TEST_CASE(test_en_US_destination_verbal));

  // test the en-US becomes phrases
  suite.test(TEST_CASE(test_en_US_becomes));

  // test the en-US becomes verbal phrases
  suite.test(TEST_CASE(test_en_US_becomes_verbal));

  // test the en-US continue phrases
  suite.test(TEST_CASE(test_en_US_continue));

  // test the en-US continue verbal alert_phrases
  suite.test(TEST_CASE(test_en_US_continue_verbal_alert));

  // test the en-US continue verbal phrases
  suite.test(TEST_CASE(test_en_US_continue_verbal));

  // test the en-US bear phrases
  suite.test(TEST_CASE(test_en_US_bear));

  // test the en-US bear verbal phrases
  suite.test(TEST_CASE(test_en_US_bear_verbal));

  // test the en-US turn phrases
  suite.test(TEST_CASE(test_en_US_turn));

  // test the en-US turn verbal phrases
  suite.test(TEST_CASE(test_en_US_turn_verbal));

  // test the en-US sharp phrases
  suite.test(TEST_CASE(test_en_US_sharp));

  // test the en-US sharp verbal phrases
  suite.test(TEST_CASE(test_en_US_sharp_verbal));

  // test the en-US uturn phrases
  suite.test(TEST_CASE(test_en_US_uturn));

  // test the en-US uturn verbal phrases
  suite.test(TEST_CASE(test_en_US_uturn_verbal));

  // test the en-US ramp_straight phrases
  suite.test(TEST_CASE(test_en_US_ramp_straight));

  // test the en-US ramp_straight verbal phrases
  suite.test(TEST_CASE(test_en_US_ramp_straight_verbal));

  // test the en-US ramp phrases
  suite.test(TEST_CASE(test_en_US_ramp));

  // test the en-US ramp verbal phrases
  suite.test(TEST_CASE(test_en_US_ramp_verbal));

  // test the en-US exit phrases
  suite.test(TEST_CASE(test_en_US_exit));

  // test the en-US exit_verbal phrases
  suite.test(TEST_CASE(test_en_US_exit_verbal));

  // test the en-US keep phrases
  suite.test(TEST_CASE(test_en_US_keep));

  // test the en-US keep_verbal phrases
  suite.test(TEST_CASE(test_en_US_keep_verbal));

  // test the en-US keep_to_stay_on phrases
  suite.test(TEST_CASE(test_en_US_keep_to_stay_on));

  // test the en-US keep_to_stay_on_verbal phrases
  suite.test(TEST_CASE(test_en_US_keep_to_stay_on_verbal));

  // test the en-US merge phrases
  suite.test(TEST_CASE(test_en_US_merge));

  // test the en-US merge_verbal phrases
  suite.test(TEST_CASE(test_en_US_merge_verbal));

  // test the en-US enter_roundabout phrases
  suite.test(TEST_CASE(test_en_US_enter_roundabout));

  // test the en-US enter_roundabout_verbal phrases
  suite.test(TEST_CASE(test_en_US_enter_roundabout_verbal));

  // test the en-US exit_roundabout phrases
  suite.test(TEST_CASE(test_en_US_exit_roundabout));

  // test the en-US exit_roundabout_verbal phrases
  suite.test(TEST_CASE(test_en_US_exit_roundabout_verbal));

  // test the en-US enter_ferry phrases
  suite.test(TEST_CASE(test_en_US_enter_ferry));

  // test the en-US enter_ferry_verbal phrases
  suite.test(TEST_CASE(test_en_US_enter_ferry_verbal));

  // test the en-US exit_ferry phrases
  suite.test(TEST_CASE(test_en_US_exit_ferry));

  // test the en-US exit_ferry_verbal phrases
  suite.test(TEST_CASE(test_en_US_exit_ferry_verbal));

  // test the en-US transit_connection_start phrases
  suite.test(TEST_CASE(test_en_US_transit_connection_start));

  // test the en-US transit_connection_start_verbal phrases
  suite.test(TEST_CASE(test_en_US_transit_connection_start_verbal));

  // test the en-US transit_connection_transfer phrases
  suite.test(TEST_CASE(test_en_US_transit_connection_transfer));

  // test the en-US transit_connection_transfer_verbal phrases
  suite.test(TEST_CASE(test_en_US_transit_connection_transfer_verbal));

  // test the en-US transit_connection_destination phrases
  suite.test(TEST_CASE(test_en_US_transit_connection_destination));

  // test the en-US transit_connection_destination_verbal phrases
  suite.test(TEST_CASE(test_en_US_transit_connection_destination_verbal));

  // test the en-US depart phrases
  suite.test(TEST_CASE(test_en_US_depart));

  // test the en-US depart_verbal phrases
  suite.test(TEST_CASE(test_en_US_depart_verbal));

  // test the en-US arrive phrases
  suite.test(TEST_CASE(test_en_US_arrive));

  // test the en-US arrive_verbal phrases
  suite.test(TEST_CASE(test_en_US_arrive_verbal));

  // test the en-US transit phrases
  suite.test(TEST_CASE(test_en_US_transit));

  // test the en-US transit_verbal phrases
  suite.test(TEST_CASE(test_en_US_transit_verbal));

  // test the en-US transit_remain_on phrases
  suite.test(TEST_CASE(test_en_US_transit_remain_on));

  // test the en-US transit_remain_on_verbal phrases
  suite.test(TEST_CASE(test_en_US_transit_remain_on_verbal));

  // test the en-US transit_transfer phrases
  suite.test(TEST_CASE(test_en_US_transit_transfer));

  // test the en-US transit_transfer_verbal phrases
  suite.test(TEST_CASE(test_en_US_transit_transfer_verbal));

  // test the en-US post_transit_connection_destination phrases
  suite.test(TEST_CASE(test_en_US_post_transit_connection_destination));

  // test the en-US post_transit_connection_destination_verbal phrases
  suite.test(TEST_CASE(test_en_US_post_transit_connection_destination_verbal));

  // test the en-US post_transition_verbal_subset phrases
  suite.test(TEST_CASE(test_en_US_post_transition_verbal));

  // test the en-US post_transition_transit_verbal phrases
  suite.test(TEST_CASE(test_en_US_post_transition_transit_verbal));

  // test the en-US verbal_multi_cue phrases
  suite.test(TEST_CASE(test_en_US_verbal_multi_cue));

  return suite.tear_down();
}
