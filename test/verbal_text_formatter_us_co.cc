#include "baldr/verbal_text_formatter_us_co.h"
#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

// Sub class to test protected methods
class VerbalTextFormatterUsCoTest : public VerbalTextFormatterUsCo {
public:
  VerbalTextFormatterUsCoTest(const std::string& country_code, const std::string& state_code)
      : VerbalTextFormatterUsCo(country_code, state_code) {
  }

  std::string ProcessStatesTts(const std::string& source) const {
    return VerbalTextFormatterUsCo::ProcessStatesTts(source);
  }
};

void TryProcessStatesTts(string source, string expected) {
  VerbalTextFormatterUsCoTest formatter_test("US", "CO");
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
  TryProcessStatesTts("CO 265", "Colorado 265");
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

void TryFormat(string source, string expected) {
  VerbalTextFormatterUsCoTest formatter_test("US", "CO");
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
  TryFormat("CT 14A", "Connecticut 14A");
  TryFormat("Co 7", "Colorado 7");
  TryFormat("Co 200", "Colorado 2 hundred");

  TryFormat("CR 539", "County Route 5 39");
  TryFormat("CR-4003", "County Route 40 o3");

  TryFormat("T609", "T6 o9");
}

} // namespace

int main() {
  test::suite suite("verbal_text_formatter_us_co");

  // ProcessStatesTts
  suite.test(TEST_CASE(TestProcessStatesTts));

  // Format
  suite.test(TEST_CASE(TestFormat));

  return suite.tear_down();
}
