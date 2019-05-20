#include "baldr/verbal_text_formatter_us_tx.h"
#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

// Sub class to test protected methods
class VerbalTextFormatterUsTxTest : public VerbalTextFormatterUsTx {
public:
  VerbalTextFormatterUsTxTest(const std::string& country_code, const std::string& state_code)
      : VerbalTextFormatterUsTx(country_code, state_code) {
  }

  std::string FormFmTts(const std::string& source) const {
    return VerbalTextFormatterUsTx::FormFmTts(source);
  }

  std::string FormRmTts(const std::string& source) const {
    return VerbalTextFormatterUsTx::FormRmTts(source);
  }
};

void TryFormFmTts(string source, string expected) {
  VerbalTextFormatterUsTxTest formatter_test("US", "TX");
  string tts = formatter_test.FormFmTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormFmTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormFmTts() {
  TryFormFmTts("FM1018", "Farm to Market Road 1018");
  TryFormFmTts("FM 1018", "Farm to Market Road 1018");
  TryFormFmTts("FM-1018", "Farm to Market Road 1018");
  TryFormFmTts("F M 1018", "Farm to Market Road 1018");
  TryFormFmTts("F-M 1018", "Farm to Market Road 1018");
  TryFormFmTts("F-M-1018", "Farm to Market Road 1018");
}

void TryFormRmTts(string source, string expected) {
  VerbalTextFormatterUsTxTest formatter_test("US", "TX");
  string tts = formatter_test.FormRmTts(source);
  if (tts != expected) {
    throw std::runtime_error("Incorrect FormRmTts - EXPECTED: " + expected + "  |  FORMED: " + tts);
  }
}

void TestFormRmTts() {
  TryFormRmTts("RM1018", "Ranch to Market Road 1018");
  TryFormRmTts("RM 1018", "Ranch to Market Road 1018");
  TryFormRmTts("FM-1018", "Ranch to Market Road 1018");
  TryFormRmTts("R M 1018", "Ranch to Market Road 1018");
  TryFormRmTts("R-M 1018", "Ranch to Market Road 1018");
  TryFormRmTts("R-M-1018", "Ranch to Market Road 1018");
}

void TryFormat(string source, string expected) {
  VerbalTextFormatterUsTxTest formatter_test("US", "TX");
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

  TryFormat("CR 539", "County Route 5 39");
  TryFormat("CR-4003", "County Route 40 o3");

  TryFormat("FM1018", "Farm to Market Road 10 18");
  TryFormat("FM 1018", "Farm to Market Road 10 18");
  TryFormat("FM-1018", "Farm to Market Road 10 18");
  TryFormat("F M 1018", "Farm to Market Road 10 18");
  TryFormat("F-M 1018", "Farm to Market Road 10 18");
  TryFormat("F-M-1018", "Farm to Market Road 10 18");

  TryFormat("RM1018", "Ranch to Market Road 10 18");
  TryFormat("RM 1018", "Ranch to Market Road 10 18");
  TryFormat("RM-1018", "Ranch to Market Road 10 18");
  TryFormat("R M 1018", "Ranch to Market Road 10 18");
  TryFormat("R-M 1018", "Ranch to Market Road 10 18");
  TryFormat("R-M-1018", "Ranch to Market Road 10 18");

  TryFormat("T609", "T6 o9");
}

} // namespace

int main() {
  test::suite suite("verbal_text_formatter_us_tx");

  // FormFmTts
  suite.test(TEST_CASE(TestFormFmTts));

  // Format
  suite.test(TEST_CASE(TestFormat));

  return suite.tear_down();
}
