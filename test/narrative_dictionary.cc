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

void test_en_US_start_phrases() {
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
}

void test_en_US_start_verbal_phrases() {
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
  suite.test(TEST_CASE(test_en_US_start_phrases));

  // test the en-US start verbal phrases
  suite.test(TEST_CASE(test_en_US_start_verbal_phrases));

  // test the en-US destination phrases
  suite.test(TEST_CASE(test_en_US_destination_phrases));

  // test the en-US destination verbal alert_phrases
  suite.test(TEST_CASE(test_en_US_destination_verbal_alert_phrases));

  // test the en-US destination verbal phrases
  suite.test(TEST_CASE(test_en_US_destination_verbal_phrases));

  return suite.tear_down();
}
