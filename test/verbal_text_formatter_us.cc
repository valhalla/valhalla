#include "baldr/verbal_text_formatter_us.h"
#include "baldr/verbal_text_formatter.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

// Sub class to test protected methods
class VerbalTextFormatterUsTest : public VerbalTextFormatterUs {
public:
  VerbalTextFormatterUsTest(const std::string& country_code, const std::string& state_code)
      : VerbalTextFormatterUs(country_code, state_code) {
  }

  std::string FormInterstateTts(const std::string& source) const {
    return VerbalTextFormatterUs::FormInterstateTts(source);
  }

  std::string FormUsHighwayTts(const std::string& source) const {
    return VerbalTextFormatterUs::FormUsHighwayTts(source);
  }

  std::string ProcessStatesTts(const std::string& source) const {
    return VerbalTextFormatterUs::ProcessStatesTts(source);
  }

  std::string ProcessCountysTts(const std::string& source) const {
    return VerbalTextFormatterUs::ProcessCountysTts(source);
  }

  std::string ProcessThousandTts(const std::string& source) const {
    return VerbalTextFormatterUs::ProcessThousandTts(source);
  }

  std::string ProcessHundredTts(const std::string& source) const {
    return VerbalTextFormatterUs::ProcessHundredTts(source);
  }

  std::string FormNumberSplitTts(const std::string& source) const {
    return VerbalTextFormatterUs::FormNumberSplitTts(source);
  }

  std::string FormLeadingOhTts(const std::string& source) const {
    return VerbalTextFormatterUs::FormLeadingOhTts(source);
  }
};

void TryFormInterstateTtsString(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "PA");
  string tts = formatter_test.FormInterstateTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormInterstateTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
  }
}

void TestFormInterstateTtsString() {
  TryFormInterstateTtsString("I H1", "Interstate H1");
  TryFormInterstateTtsString("I 5", "Interstate 5");
  TryFormInterstateTtsString("I 35", "Interstate 35");
  TryFormInterstateTtsString("I 35E", "Interstate 35E");
  TryFormInterstateTtsString("I 35W", "Interstate 35W");
  TryFormInterstateTtsString("I 95 South", "Interstate 95 South");
  TryFormInterstateTtsString("I 270 Spur", "Interstate 270 Spur");
  TryFormInterstateTtsString("I 495 West", "Interstate 495 West");
  TryFormInterstateTtsString("I-695 West", "Interstate 695 West");
  TryFormInterstateTtsString("I-895", "Interstate 895");
  TryFormInterstateTtsString("i-205 East", "Interstate 205 East");
  TryFormInterstateTtsString("WI 129", "WI 129");
}

void TryFormUsHighwayTtsString(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "PA");
  string tts = formatter_test.FormUsHighwayTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormUsHighwayTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
  }
}

void TestFormUsHighwayTtsString() {
  TryFormUsHighwayTtsString("US 1", "U.S. 1");
  TryFormUsHighwayTtsString("US 22", "U.S. 22");
  TryFormUsHighwayTtsString("US 220 North", "U.S. 220 North");
  TryFormUsHighwayTtsString("US-220 South", "U.S. 220 South");
  TryFormUsHighwayTtsString("Us 422 Business Alternate", "U.S. 422 Business Alternate");
  TryFormUsHighwayTtsString("Us-522", "U.S. 522");
}

void TryProcessStatesTts(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "");
  string tts = formatter_test.ProcessStatesTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect ProcessStatesTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
  }
}

void TestProcessStatesTts() {
  TryProcessStatesTts("AL 261", "Alabama 261");
  TryProcessStatesTts("AL-261", "Alabama 261");
  TryProcessStatesTts("Al 261", "Alabama 261");
  TryProcessStatesTts("AK 1", "Alaska 1");
  TryProcessStatesTts("AR 107", "Arkansas 107");
  TryProcessStatesTts("CA 480", "California 480");
  TryProcessStatesTts("CO 265", "CO 265");
  TryProcessStatesTts("CT 14A", "Connecticut 14A");
  TryProcessStatesTts("CT 15", "Connecticut 15");
  TryProcessStatesTts("DE 10", "Delaware 10");
  TryProcessStatesTts("DC 295", "D C 295");
  TryProcessStatesTts("FL 535", "Florida 535");
  TryProcessStatesTts("FL A1A", "Florida A1A");
  TryProcessStatesTts("GA 400", "Georgia 400");
  TryProcessStatesTts("HI 92", "Hawaii 92");
  TryProcessStatesTts("ID 21", "Idaho 21");
  TryProcessStatesTts("IL 97", "Illinois 97");
  TryProcessStatesTts("IN 135", "Indiana 135");
  TryProcessStatesTts("IA 5", "Iowa 5");
  TryProcessStatesTts("KS 4", "Kansas 4");
  TryProcessStatesTts("KY 676", "Kentucky 676");
  TryProcessStatesTts("LA 73", "Louisiana 73");
  TryProcessStatesTts("ME 27", "Maine 27");
  TryProcessStatesTts("MD 450", "Maryland 450");
  TryProcessStatesTts("MA 2", "Massachusetts 2");
  TryProcessStatesTts("M 43", "Michigan 43");
  TryProcessStatesTts("MN 55", "Minnesota 55");
  TryProcessStatesTts("MS 468", "Mississippi 468");
  TryProcessStatesTts("MO 180", "Missouri 180");
  TryProcessStatesTts("MO A", "Missouri A");
  TryProcessStatesTts("Mo JJ", "Missouri JJ");
  TryProcessStatesTts("MO JJ West", "Missouri JJ West");
  TryProcessStatesTts("Mo Money Road", "Mo Money Road");
  TryProcessStatesTts("MT 282", "Montana 282");
  TryProcessStatesTts("NE 2", "Nebraska 2");
  TryProcessStatesTts("NE 55W Link", "Nebraska 55W Link");
  TryProcessStatesTts("NV 592", "Nevada 592");
  TryProcessStatesTts("NH 13", "New Hampshire 13");
  TryProcessStatesTts("NJ 33", "New Jersey 33");
  TryProcessStatesTts("NM 599", "New Mexico 599");
  TryProcessStatesTts("NY 5", "New York 5");
  TryProcessStatesTts("NC 50", "North Carolina 50");
  TryProcessStatesTts("ND 810", "North Dakota 810");
  TryProcessStatesTts("OH 73", "Ohio 73");
  TryProcessStatesTts("OK 3", "Oklahoma 3");
  TryProcessStatesTts("OR 99E Business", "Oregon 99E Business");
  TryProcessStatesTts("PA 39", "Pennsylvania 39");
  TryProcessStatesTts("RI 146", "Rhode Island 146");
  TryProcessStatesTts("SC 12", "South Carolina 12");
  TryProcessStatesTts("SD 34", "South Dakota 34");
  TryProcessStatesTts("SD 1806", "South Dakota 1806");
  TryProcessStatesTts("TN 155", "Tennessee 155");
  TryProcessStatesTts("TX 165", "Texas 165");
  TryProcessStatesTts("UT 186", "Utah 186");
  TryProcessStatesTts("VT 12", "Vermont 12");
  TryProcessStatesTts("VA 195", "Virginia 195");
  TryProcessStatesTts("WA 8", "Washington 8");
  TryProcessStatesTts("WV 7", "West Virginia 7");
  TryProcessStatesTts("WI 30", "Wisconsin 30");
  TryProcessStatesTts("WY 212", "Wyoming 212");
}

void TryProcessCountysTts(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "");
  string tts = formatter_test.ProcessCountysTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect ProcessCountysTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
  }
}

void TestProcessCountysTts() {
  TryProcessCountysTts("CR 0", "County Route 0");
  TryProcessCountysTts("CR 1.00", "County Route 1.00");
  TryProcessCountysTts("CR 100", "County Route 100");
  TryProcessCountysTts("CR 100 S", "County Route 100 S");
  TryProcessCountysTts("CR 539", "County Route 539");
  TryProcessCountysTts("CR-852", "County Route 852");
  TryProcessCountysTts("C R A13", "County Route A13");
  TryProcessCountysTts("C R 13B", "County Route 13B");
  TryProcessCountysTts("C R 404", "County Route 404");
  TryProcessCountysTts("CR-4003", "County Route 4003");
  TryProcessCountysTts("CR116", "County Route 116");
  TryProcessCountysTts("CR A", "County Route A");
  TryProcessCountysTts("CR JJ", "County Route JJ");
  TryProcessCountysTts("CR J29", "County Route J29");
  TryProcessCountysTts("CR 18VV", "County Route 18VV");
  TryProcessCountysTts("Cr 8542 Marmot Trail", "County Route 8542 Marmot Trail");
  TryProcessCountysTts("ACR 123", "ACR 123");
  TryProcessCountysTts("A CR 123", "A County Route 123");
  TryProcessCountysTts("CR Ettinger", "CR Ettinger");
  TryProcessCountysTts("Crap", "Crap");
  TryProcessCountysTts("Co 7", "County Road 7");
  TryProcessCountysTts("Co-17", "County Road 17");
  TryProcessCountysTts("Co39", "County Road 39");
  TryProcessCountysTts("Co 200", "County Road 200");
  TryProcessCountysTts("Co 200A", "County Road 200A");
  TryProcessCountysTts("Co 200BAD", "Co 200BAD");
}

void TryFormThousandTtsString(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "PA");
  string tts = formatter_test.ProcessThousandTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormThousandTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
  }
}

void TestFormThousandTtsString() {
  TryFormThousandTtsString("1", "1");
  TryFormThousandTtsString("10", "10");
  TryFormThousandTtsString("1000", "1 thousand");
  TryFormThousandTtsString("1000D", "1 thousand D");
  TryFormThousandTtsString("D1000", "D1 thousand");
  TryFormThousandTtsString("D1000D", "D1 thousand D");
  TryFormThousandTtsString("3000-4000", "3 thousand 4 thousand");
  TryFormThousandTtsString("53000-54000", "53 thousand 54 thousand");
  TryFormThousandTtsString("Co 2000", "Co 2 thousand");
  TryFormThousandTtsString("Co 2000A", "Co 2 thousand A");
  TryFormThousandTtsString("Co A2000", "Co A2 thousand");
  TryFormThousandTtsString("Co A2000A", "Co A2 thousand A");
  TryFormThousandTtsString("MD 1000", "MD 1 thousand");
  TryFormThousandTtsString("MD 1000 West", "MD 1 thousand West");
  TryFormThousandTtsString("24000", "24 thousand");
  TryFormThousandTtsString("MD 24000", "MD 24 thousand");
  TryFormThousandTtsString("MD 24000 West", "MD 24 thousand West");
}

void TryFormHundredTtsString(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "MD");
  string tts = formatter_test.ProcessHundredTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormHundredTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
  }
}

void TestFormHundredTtsString() {
  TryFormHundredTtsString("1", "1");
  TryFormHundredTtsString("10", "10");
  TryFormHundredTtsString("100", "1 hundred");
  TryFormHundredTtsString("100B", "1 hundred B");
  TryFormHundredTtsString("B100", "B1 hundred");
  TryFormHundredTtsString("B100B", "B1 hundred B");
  TryFormHundredTtsString("300-400", "3 hundred 4 hundred");
  TryFormHundredTtsString("5300-5400", "53 hundred 54 hundred");
  TryFormHundredTtsString("Co 200", "Co 2 hundred");
  TryFormHundredTtsString("Co 200A", "Co 2 hundred A");
  TryFormHundredTtsString("Co A200", "Co A2 hundred");
  TryFormHundredTtsString("Co A200A", "Co A2 hundred A");
  TryFormHundredTtsString("MD 100", "MD 1 hundred");
  TryFormHundredTtsString("MD 100 West", "MD 1 hundred West");
  TryFormHundredTtsString("2400", "24 hundred");
  TryFormHundredTtsString("MD 2400", "MD 24 hundred");
  TryFormHundredTtsString("MD 2400 West", "MD 24 hundred West");
}

void TryFormNumberSplitTtsString(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "PA");
  string tts = formatter_test.FormNumberSplitTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormNumberSplitTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
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
  TryFormNumberSplitTtsString("101st Street", "101st Street");
  TryFormNumberSplitTtsString("102nd Street", "102nd Street");
  TryFormNumberSplitTtsString("103rd Street", "103rd Street");
  TryFormNumberSplitTtsString("104th Street", "104th Street");
  TryFormNumberSplitTtsString("101St Street", "101St Street");
  TryFormNumberSplitTtsString("102ND Street", "102ND Street");
}

void TryFormLeadingOhTtsString(string source, string expected) {
  VerbalTextFormatterUsTest formatter_test("US", "PA");
  string tts = formatter_test.FormLeadingOhTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormLeadingOhTts - EXPECTED: " + expected +
                             "  |  FORMED: " + tts);
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
  VerbalTextFormatterUsTest formatter_test("US", "PA");
  string tts = formatter_test.Format(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect Format - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormat() {
  TryFormat("I H1", "Interstate H1");
  TryFormat("I 5", "Interstate 5");
  TryFormat("I 35", "Interstate 35");
  TryFormat("I 35E", "Interstate 35E");
  TryFormat("I 35W", "Interstate 35W");
  TryFormat("I 95 South", "Interstate 95 South");
  TryFormat("I 270 Spur", "Interstate 2 70 Spur");
  TryFormat("I 495 West", "Interstate 4 95 West");
  TryFormat("I-695 West", "Interstate 6 95 West");
  TryFormat("I-895", "Interstate 8 95");
  TryFormat("i-205 East", "Interstate 2 o5 East");

  TryFormat("US 1", "U.S. 1");
  TryFormat("US 22", "U.S. 22");
  TryFormat("US 220 North", "U.S. 2 20 North");
  TryFormat("US-220 South", "U.S. 2 20 South");
  TryFormat("US 202", "U.S. 2 o2");
  TryFormat("Us 422 Business Alternate", "U.S. 4 22 Business Alternate");
  TryFormat("Us-522", "U.S. 5 22");
  TryFormat("US Highway 222", "U.S. Highway 2 22");

  TryFormat("AL 233", "Alabama 2 33");
  TryFormat("AR 107", "Arkansas 1 o7");
  TryFormat("PA 23", "Pennsylvania 23");
  TryFormat("PA 283", "Pennsylvania 2 83");
  TryFormat("PA 100", "Pennsylvania 1 hundred");
  TryFormat("PA 1080", "Pennsylvania 10 80");
  TryFormat("PA 4007", "Pennsylvania 40 o7");
  TryFormat("SR 1021", "State Route 10 21");
  TryFormat("SR-1021", "State Route 10 21");
  TryFormat("Sr 1021", "State Route 10 21");
  TryFormat("SR3032", "State Route 30 32");
  TryFormat("SH 1021", "State Highway 10 21");
  TryFormat("SH-1021", "State Highway 10 21");
  TryFormat("Sh 1021", "State Highway 10 21");
  TryFormat("SH3032", "State Highway 30 32");
  TryFormat("CT 14A", "Connecticut 14A");

  TryFormat("CR 0", "County Route 0");
  TryFormat("CR 1.00", "County Route 1.00");
  TryFormat("CR 100", "County Route 1 hundred");
  TryFormat("CR 100 S", "County Route 1 hundred S");
  TryFormat("CR 539", "County Route 5 39");
  TryFormat("CR-852", "County Route 8 52");
  TryFormat("C R A13", "County Route A13");
  TryFormat("C R 13B", "County Route 13B");
  TryFormat("C R 404", "County Route 4 o4");
  TryFormat("CR-4003", "County Route 40 o3");
  TryFormat("CR116", "County Route 1 16");
  TryFormat("CR A", "County Route A");
  TryFormat("CR JJ", "County Route JJ");
  TryFormat("CR J29", "County Route J29");
  TryFormat("CR 18VV", "County Route 18VV");
  TryFormat("Cr 8542 Marmot Trail", "County Route 85 42 Marmot Trail");
  TryFormat("CR Ettinger", "CR Ettinger");
  TryFormat("Crap", "Crap");
  TryFormat("Co 7", "County Road 7");
  TryFormat("Co-17", "County Road 17");
  TryFormat("Co39", "County Road 39");
  TryFormat("Co 200", "County Road 2 hundred");
  TryFormat("Co 200A", "County Road 2 hundred A");
  TryFormat("Co 200BAD", "Co 2 hundred BAD");
  TryFormat("Co 2000", "County Road 2 thousand");
  TryFormat("Co 2000A", "County Road 2 thousand A");
  TryFormat("Co 2000BAD", "Co 2 thousand BAD");

  TryFormat("North 2800th", "North 28 hundredth");
  TryFormat("North 2800th Avenue", "North 28 hundredth Avenue");
  TryFormat("North 2800Th Avenue", "North 28 hundredth Avenue");

  TryFormat("North 28000th", "North 28 thousandth");
  TryFormat("North 28000th Avenue", "North 28 thousandth Avenue");
  TryFormat("North 28000TH Avenue", "North 28 thousandth Avenue");

  TryFormat("905 203rd Street", "9 o5 203rd Street");

  TryFormat("T609", "T6 o9");
}

} // namespace

int main() {
  test::suite suite("verbal_text_formatter_us");

  // FormInterstateTts
  suite.test(TEST_CASE(TestFormInterstateTtsString));

  // FormUsHighwayTtsString
  suite.test(TEST_CASE(TestFormUsHighwayTtsString));

  // ProcessStatesTts
  suite.test(TEST_CASE(TestProcessStatesTts));

  // ProcessCountysTts
  suite.test(TEST_CASE(TestProcessCountysTts));

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
