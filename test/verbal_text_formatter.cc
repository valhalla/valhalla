#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

// Sub class to test protected methods
class VerbalTextFormatterTest : public VerbalTextFormatter {
public:
  VerbalTextFormatterTest(const std::string& country_code, const std::string& state_code)
      : VerbalTextFormatter(country_code, state_code) {
  }

  std::string FormNumberSplitTts(const std::string& source) const {
    return VerbalTextFormatter::FormNumberSplitTts(source);
  }
};

void TryFormNumberSplitTtsString(const string& source, const string& expected) {
  VerbalTextFormatterTest formatter_test("US", "PA");
  string tts = formatter_test.FormNumberSplitTts(source);
  EXPECT_EQ(tts, expected);
}

TEST(VeralTextFormatter, TestFormNumberSplitTtsString) {
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
  TryFormNumberSplitTtsString("PA 23", "PA 23");
  TryFormNumberSplitTtsString("PA 283", "PA 2 83");
  TryFormNumberSplitTtsString("US 202", "US 2 02");
  TryFormNumberSplitTtsString("PA 1080", "PA 10 80");
  TryFormNumberSplitTtsString("PA 40001", "PA 4 00 01");
  TryFormNumberSplitTtsString("PA 4007", "PA 40 07");
  TryFormNumberSplitTtsString("SR 1021", "SR 10 21");
  TryFormNumberSplitTtsString("SR-1021", "SR-10 21");
  TryFormNumberSplitTtsString("Sr 1021", "Sr 10 21");
  TryFormNumberSplitTtsString("T609", "T6 09");
  TryFormNumberSplitTtsString("US 422 Business Alternate", "US 4 22 Business Alternate");
}

void TryFormat(const string& source, const string& expected) {
  VerbalTextFormatterTest formatter_test("FR", "IDF");
  string tts = formatter_test.Format(source);
  EXPECT_EQ(tts, expected);
}

TEST(VeralTextFormatter, TestFormat) {
  TryFormat("E411", "E411");
  TryFormat("A104", "A104");
  TryFormat("A154", "A154");
  TryFormat("A320", "A320");
  TryFormat("A406", "A406");
  TryFormat("A844", "A844");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
