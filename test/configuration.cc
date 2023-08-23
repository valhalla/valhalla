#include "test.h"
#include <string>

#include "configuration.h"

namespace {

TEST(Configuration, UseBeforeConfiguration) {
  EXPECT_ANY_THROW(valhalla::config());
}

TEST(Configuration, ReadInlineConfig) {
  using namespace valhalla;

  auto inline_config = R"(
    {
      "obj1": {
        "val": "example"
      },
      "obj2": {
        "inner": {
          "val": 4
        }
      }
    }
    )";

  auto conf = config(inline_config);

  EXPECT_EQ(conf.get<std::string>("obj1.val"), "example");
  EXPECT_EQ(conf.get<uint32_t>("obj2.inner.val"), 4);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
