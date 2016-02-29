#include <string>

#include "odin/util.h"
#include "odin/narrative_dictionary.h"

#include "test.h"

using namespace valhalla::odin;


namespace {

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
  validate(cardinal_directions, { "north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest" });

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.start_subset.empty_street_name_labels;
  validate(empty_street_name_labels, { "walkway", "cycleway", "mountain bike trail" });

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
  validate(cardinal_directions, { "north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest" });

  // empty_street_name_labels "walkway", "cycleway", "mountain bike trail"
  const auto& empty_street_name_labels = dictionary.start_verbal_subset.empty_street_name_labels;
  validate(empty_street_name_labels, { "walkway", "cycleway", "mountain bike trail" });

  // metric_lengths
  const auto& metric_lengths = dictionary.start_verbal_subset.metric_lengths;
  validate(metric_lengths, { "<KILOMETERS> kilometers", "1 kilometer", "a half kilometer", "<METERS> meters", "less than 10 meters" });

  // us_customary_lengths
  const auto& us_customary_lengths = dictionary.start_verbal_subset.us_customary_lengths;
  validate(us_customary_lengths, { "<MILES> miles", "1 mile", "a half mile", "<TENTHS_OF_MILE> tenths of a mile", "1 tenth of a mile", "<FEET> feet", "less than 10 feet" });

}

void test_en_US_destination_phrases() {
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
}

void test_en_US_destination_verbal_alert_phrases() {
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
}

void test_en_US_destination_verbal_phrases() {
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
}

}

int main() {
  test::suite suite("narrative_dictionary");

  // test the en-US start phrases
  suite.test(TEST_CASE(test_en_US_start));

  // test the en-US start verbal phrases
  suite.test(TEST_CASE(test_en_US_start_verbal));

  // test the en-US destination phrases
  suite.test(TEST_CASE(test_en_US_destination_phrases));

  // test the en-US destination verbal alert_phrases
  suite.test(TEST_CASE(test_en_US_destination_verbal_alert_phrases));

  // test the en-US destination verbal phrases
  suite.test(TEST_CASE(test_en_US_destination_verbal_phrases));

  return suite.tear_down();
}
