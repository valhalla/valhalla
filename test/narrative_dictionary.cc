#include <string>

#include "odin/util.h"
#include "odin/narrative_dictionary.h"

#include "test.h"

using namespace valhalla::odin;


namespace {

// Expected strings
const std::vector<std::string> kExpectedEmptyStreetNameLabels = { "walkway", "cycleway", "mountain bike trail" };
const std::vector<std::string> kExpectedCardinalDirections = { "north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest" };
const std::vector<std::string> kExpectedMetricLengths = { "<KILOMETERS> kilometers", "1 kilometer", "a half kilometer", "<METERS> meters", "less than 10 meters" };
const std::vector<std::string> kExpectedUsCustomaryLengths = { "<MILES> miles", "1 mile", "a half mile", "<TENTHS_OF_MILE> tenths of a mile", "1 tenth of a mile", "<FEET> feet", "less than 10 feet" };
const std::vector<std::string> kExpectedRelativeTwoDirections = { "left", "right" };
const std::vector<std::string> kExpectedRelativeTurnDirections = { "left", "sharp left", "right", "sharp right" };

const NarrativeDictionary& GetNarrativeDictionary(const std::string& lang_tag) {
  // Get the locale dictionary
  const auto phrase_dictionary = get_locales().find(lang_tag);

  // If language tag is not found then throw error
  if (phrase_dictionary == get_locales().end()) {
    throw std::runtime_error("Invalid language tag.");
  }

  return phrase_dictionary->second;
}

void validate(const std::string& test_target, const std::string& expected) {
  if (test_target != expected) {
    throw std::runtime_error(
        "Invalid entry: " + test_target + "  |  expected: " + expected);
  }

}

void validate(const std::vector<std::string>& test_target,
              const std::vector<std::string>& expected) {
  if (test_target.size() != expected.size()) {
    throw std::runtime_error(
        "Invalid item count: " + std::to_string(test_target.size())
            + "  |  expected: " + std::to_string(expected.size()));
  }

  for (auto test_target_item = test_target.begin(), expected_item = expected.begin();
      test_target_item != test_target.end();
      ++test_target_item, ++expected_item) {
    if ((*test_target_item) != (*expected_item)) {
      throw std::runtime_error(
          "Invalid entry: " + (*test_target_item) + "  |  expected: " + (*expected_item));
    }
  }
}

void test_en_US_start() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Head <CARDINAL_DIRECTION>.",
  const auto& phrase_0 = dictionary.start_subset.phrases.at("0");
  validate(phrase_0, "Head <CARDINAL_DIRECTION>.");

  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.start_subset.phrases.at("1");
  validate(phrase_1, "Head <CARDINAL_DIRECTION> on <STREET_NAMES>.");

  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>."
  const auto& phrase_2 = dictionary.start_subset.phrases.at("2");
  validate(phrase_2, "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.");

  // cardinal_directions
  const auto& cardinal_directions = dictionary.start_subset.cardinal_directions;
  validate(cardinal_directions, kExpectedCardinalDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.start_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);

}

void test_en_US_start_verbal() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Head <CARDINAL_DIRECTION> for <LENGTH>.",
  const auto& phrase_0 = dictionary.start_verbal_subset.phrases.at("0");
  validate(phrase_0, "Head <CARDINAL_DIRECTION> for <LENGTH>.");

  // "1": "Head <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>.",
  const auto& phrase_1 = dictionary.start_verbal_subset.phrases.at("1");
  validate(phrase_1, "Head <CARDINAL_DIRECTION> on <STREET_NAMES> for <LENGTH>.");

  // "2": "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>."
  const auto& phrase_2 = dictionary.start_verbal_subset.phrases.at("2");
  validate(phrase_2, "Head <CARDINAL_DIRECTION> on <BEGIN_STREET_NAMES>.");

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
  const auto& empty_street_name_labels = dictionary.continue_verbal_alert_subset.empty_street_name_labels;
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
  validate(phrase_2, "Bear <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.");

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
  validate(phrase_2, "Turn <RELATIVE_DIRECTION> onto <BEGIN_STREET_NAMES>. Continue on <STREET_NAMES>.");

  // "3": "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>."
  const auto& phrase_3 = dictionary.turn_subset.phrases.at("3");
  validate(phrase_3, "Turn <RELATIVE_DIRECTION> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.turn_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTurnDirections);

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
  validate(relative_directions, kExpectedRelativeTurnDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.turn_verbal_subset.empty_street_name_labels;
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
  validate(phrase_4, "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.");

  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
  const auto& phrase_5 = dictionary.uturn_subset.phrases.at("5");
  validate(phrase_5, "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.uturn_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.uturn_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);

}

void test_en_US_uturn_verbal_alert() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "Make a <RELATIVE_DIRECTION> U-turn.",
  const auto& phrase_0 = dictionary.uturn_verbal_alert_subset.phrases.at("0");
  validate(phrase_0, "Make a <RELATIVE_DIRECTION> U-turn.");

  // "1": "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.",
  const auto& phrase_1 = dictionary.uturn_verbal_alert_subset.phrases.at("1");
  validate(phrase_1, "Make a <RELATIVE_DIRECTION> U-turn onto <STREET_NAMES>.");

  // "2": "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.",
  const auto& phrase_2 = dictionary.uturn_verbal_alert_subset.phrases.at("2");
  validate(phrase_2, "Make a <RELATIVE_DIRECTION> U-turn to stay on <STREET_NAMES>.");

  // "3": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>."
  const auto& phrase_3 = dictionary.uturn_verbal_alert_subset.phrases.at("3");
  validate(phrase_3, "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES>.");

  // relative_directions
  const auto& relative_directions = dictionary.uturn_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.uturn_verbal_alert_subset.empty_street_name_labels;
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
  validate(phrase_4, "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> onto <STREET_NAMES>.");

  // "5": "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>."
  const auto& phrase_5 = dictionary.uturn_verbal_subset.phrases.at("5");
  validate(phrase_5, "Make a <RELATIVE_DIRECTION> U-turn at <CROSS_STREET_NAMES> to stay on <STREET_NAMES>.");

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

void test_en_US_ramp_straight_verbal_alert() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  //  "0": "Stay straight to take the ramp.",
  const auto& phrase_0 = dictionary.ramp_straight_verbal_alert_subset.phrases.at("0");
  validate(phrase_0, "Stay straight to take the ramp.");

  //  "1": "Stay straight to take the <BRANCH_SIGN> ramp.",
  const auto& phrase_1 = dictionary.ramp_straight_verbal_alert_subset.phrases.at("1");
  validate(phrase_1, "Stay straight to take the <BRANCH_SIGN> ramp.");

  //  "2": "Stay straight to take the ramp toward <TOWARD_SIGN>.",
  const auto& phrase_2 = dictionary.ramp_straight_verbal_alert_subset.phrases.at("2");
  validate(phrase_2, "Stay straight to take the ramp toward <TOWARD_SIGN>.");

  //  "3": "Stay straight to take the <NAME_SIGN> ramp."
  const auto& phrase_3 = dictionary.ramp_straight_verbal_alert_subset.phrases.at("3");
  validate(phrase_3, "Stay straight to take the <NAME_SIGN> ramp.");

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
  validate(phrase_8, "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  // "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
  const auto& phrase_9 = dictionary.ramp_subset.phrases.at("9");
  validate(phrase_9, "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp.");

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
  validate(phrase_8, "Turn <RELATIVE_DIRECTION> to take the <BRANCH_SIGN> ramp toward <TOWARD_SIGN>.");

  // "9": "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp."
  const auto& phrase_9 = dictionary.ramp_verbal_subset.phrases.at("9");
  validate(phrase_9, "Turn <RELATIVE_DIRECTION> to take the <NAME_SIGN> ramp.");

  // relative_directions
  const auto& relative_directions = dictionary.ramp_verbal_subset.relative_directions;
  validate(relative_directions, kExpectedRelativeTwoDirections);

}

void test_en_US_post_transition_verbal_subset() {
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
  const auto& empty_street_name_labels = dictionary.post_transition_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, kExpectedEmptyStreetNameLabels);

}

void test_en_US_verbal_multi_cue() {
  const NarrativeDictionary& dictionary = GetNarrativeDictionary("en-US");

  // "0": "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>"
  const auto& phrase_0 = dictionary.verbal_multi_cue_subset.phrases.at("0");
  validate(phrase_0, "<CURRENT_VERBAL_CUE> Then <NEXT_VERBAL_CUE>");
}

}

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

  // test the en-US uturn phrases
  suite.test(TEST_CASE(test_en_US_uturn));

  // test the en-US uturn verbal alert_phrases
  suite.test(TEST_CASE(test_en_US_uturn_verbal_alert));

  // test the en-US uturn verbal phrases
  suite.test(TEST_CASE(test_en_US_uturn_verbal));

  // test the en-US ramp_straight phrases
  suite.test(TEST_CASE(test_en_US_ramp_straight));

  // test the en-US ramp_straight verbal alert_phrases
  suite.test(TEST_CASE(test_en_US_ramp_straight_verbal_alert));

  // test the en-US ramp_straight verbal phrases
  suite.test(TEST_CASE(test_en_US_ramp_straight_verbal));

  // test the en-US ramp phrases
  suite.test(TEST_CASE(test_en_US_ramp));

  // test the en-US ramp verbal phrases
  suite.test(TEST_CASE(test_en_US_ramp_verbal));

  // test the en-US post_transition_verbal_subset phrases
  suite.test(TEST_CASE(test_en_US_post_transition_verbal_subset));

  // test the en-US verbal_multi_cue phrases
  suite.test(TEST_CASE(test_en_US_verbal_multi_cue));

  return suite.tear_down();
}
