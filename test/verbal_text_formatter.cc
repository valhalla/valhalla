#include <regex>

#include "test.h"
#include "valhalla/baldr/verbal_text_formatter.h"
#include "valhalla/baldr/verbal_text_formatter_us.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

// Sub class to test protected methods
class VerbalTextFormatterTest : public VerbalTextFormatter {
 public:
  VerbalTextFormatterTest(const std::string& country_code,
                          const std::string& state_code)
      : VerbalTextFormatter(country_code, state_code) {
  }

  std::string FormThousandTts(const std::string& source) const {
    return VerbalTextFormatter::FormThousandTts(source);
  }

  std::string FormHundredTts(const std::string& source) const {
    return VerbalTextFormatter::FormHundredTts(source);
  }

  std::string FormNumberSplitTts(const std::string& source) const {
    return VerbalTextFormatter::FormNumberSplitTts(source);
  }

  std::string FormLeadingOhTts(const std::string& source) const{
    return VerbalTextFormatter::FormLeadingOhTts(source);
  }

};

void TryFormThousandTtsString(string source, string expected) {
  VerbalTextFormatterTest formatter_test("US", "PA");
  string tts = formatter_test.FormThousandTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormThousandTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormThousandTtsString() {
  TryFormThousandTtsString("1", "1");
  TryFormThousandTtsString("10", "10");
  TryFormThousandTtsString("1000", "1 thousand");
  TryFormThousandTtsString("MD 1000", "MD 1 thousand");
  TryFormThousandTtsString("MD 1000 West", "MD 1 thousand West");
  TryFormThousandTtsString("24000", "24 thousand");
  TryFormThousandTtsString("MD 24000", "MD 24 thousand");
  TryFormThousandTtsString("MD 24000 West", "MD 24 thousand West");
}

void TryFormHundredTtsString(string source, string expected) {
  VerbalTextFormatterTest formatter_test("US", "MD");
  string tts = formatter_test.FormHundredTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormHundredTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormHundredTtsString() {
  TryFormHundredTtsString("1", "1");
  TryFormHundredTtsString("10", "10");
  TryFormHundredTtsString("100", "1 hundred");
  TryFormHundredTtsString("MD 100", "MD 1 hundred");
  TryFormHundredTtsString("MD 100 West", "MD 1 hundred West");
  TryFormHundredTtsString("2400", "24 hundred");
  TryFormHundredTtsString("MD 2400", "MD 24 hundred");
  TryFormHundredTtsString("MD 2400 West", "MD 24 hundred West");
}

void TryFormNumberSplitTtsString(string source, string expected) {
  VerbalTextFormatterTest formatter_test("US", "PA");
  string tts = formatter_test.FormNumberSplitTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormNumberSplitTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormNumberSplitTtsString() {
  TryFormNumberSplitTtsString("1", "1");
  TryFormNumberSplitTtsString("12", "12");
  TryFormNumberSplitTtsString("123", "1 23");
  TryFormNumberSplitTtsString("1234", "12 34");
  TryFormNumberSplitTtsString("12345", "1 23 45");
  TryFormNumberSplitTtsString("123456", "12 34 56");
  TryFormNumberSplitTtsString("1234567", "1 23 45 67");
  TryFormNumberSplitTtsString("12345678", "12 34 56 78");
  TryFormNumberSplitTtsString("123456789", "1 23 45 67 89");
  TryFormNumberSplitTtsString("1234567890", "12 34 56 78 90");
  TryFormNumberSplitTtsString("105", "1 05");
  TryFormNumberSplitTtsString("20304", "2 03 04");
  TryFormNumberSplitTtsString("3010203", "3 01 02 03");
  TryFormNumberSplitTtsString("PA 220", "PA 2 20");
  TryFormNumberSplitTtsString("PA 283 West", "PA 2 83 West");
  TryFormNumberSplitTtsString("CR 1155 North", "CR 11 55 North");
  TryFormNumberSplitTtsString("146B", "1 46B");
  TryFormNumberSplitTtsString("1 123456", "1 12 34 56");
  TryFormNumberSplitTtsString("12 123456", "12 12 34 56");
  TryFormNumberSplitTtsString("1 123456 1", "1 12 34 56 1");
  TryFormNumberSplitTtsString("12 123456 1", "12 12 34 56 1");
  TryFormNumberSplitTtsString("123 123456", "1 23 12 34 56");
  TryFormNumberSplitTtsString("123456, 123456", "12 34 56, 12 34 56");
  TryFormNumberSplitTtsString("123456, 123456, 123456", "12 34 56, 12 34 56, 12 34 56");
  TryFormNumberSplitTtsString("County Road 00-122", "County Road 00-1 22");
  TryFormNumberSplitTtsString("Road 0110", "Road 01 10");
}

void TryFormLeadingOhTtsString(string source, string expected) {
  VerbalTextFormatterTest formatter_test("US", "PA");
  string tts = formatter_test.FormLeadingOhTts(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect FormLeadingOhTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormLeadingOhTtsString() {
  TryFormLeadingOhTtsString("1", "1");
  TryFormLeadingOhTtsString("1 05", "1 o5");
  TryFormLeadingOhTtsString("West 1 05 1st Avenue", "West 1 o5 1st Avenue");
  TryFormLeadingOhTtsString("10 05 03", "10 o5 o3");
  TryFormLeadingOhTtsString("County Road 00-1 22", "County Road 00-1 22");
  TryFormLeadingOhTtsString("County Road 0-30", "County Road 0-30");
  TryFormLeadingOhTtsString("State Highway 0", "State Highway 0");
}

void TryFormat(string source, string expected) {
  VerbalTextFormatterTest formatter_test("US", "PA");
  string tts = formatter_test.Format(source);
  if (tts != expected) {
    throw std::runtime_error(
        "Incorrect Format - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormat() {
  TryFormat("PA 23", "PA 23");
  TryFormat("PA 283", "PA 2 83");
  TryFormat("PA 100", "PA 1 hundred");
  TryFormat("US 202", "US 2 o2");
  TryFormat("PA 1080", "PA 10 80");
  TryFormat("PA 40001", "PA 4 00 o1");
  TryFormat("PA 4007", "PA 40 o7");
  TryFormat("SR 1021", "SR 10 21");
  TryFormat("SR-1021", "SR-10 21");
  TryFormat("Sr 1021", "Sr 10 21");
  TryFormat("T609", "T6 o9");
  TryFormat("US 422 Business Alternate", "US 4 22 Business Alternate");
}

}

int main() {
  test::suite suite("verbal_text_formatter");

  // FormThousandTtsString
  suite.test(TEST_CASE(TestFormThousandTtsString));

  // FormHundredTtsString
  suite.test(TEST_CASE(TestFormHundredTtsString));

  // FormNumberSplitTtsString
  suite.test(TEST_CASE(TestFormNumberSplitTtsString));

  // FormLeadingOhTtsString
  suite.test(TEST_CASE(TestFormLeadingOhTtsString));

  // Format
  suite.test(TEST_CASE(TestFormat));

  return suite.tear_down();
}
