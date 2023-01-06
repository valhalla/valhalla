#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class LitTest : public ::testing::TestWithParam<std::tuple<std::string, bool>> {
protected:
  static gurka::nodelayout layout;
  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A---------------------------B
    )";
    layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  }
};

gurka::nodelayout LitTest::layout = {};

//=======================================================================================
TEST_P(LitTest, tagged_lit) {

  const std::string& lit_value = std::get<0>(GetParam());

  gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
  };

  if (!lit_value.empty()) {
    ways["AB"]["lit"] = lit_value;
  }

  const gurka::map& map = gurka::buildtiles(layout, ways, {}, {}, "test/data/tagged_lit");
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto& edge_tuple = gurka::findEdgeByNodes(*reader, layout, "A", "B");
  const baldr::DirectedEdge* edge = std::get<1>(edge_tuple);
  const bool& evaluates_to = std::get<1>(GetParam());
  ASSERT_EQ(edge->lit(), evaluates_to);
}

INSTANTIATE_TEST_SUITE_P(LitTest,
                         LitTest,
                         ::testing::Values(std::make_tuple("limited", false),
                                           std::make_tuple("yes", true),
                                           std::make_tuple("sunset-sunrise", true),
                                           std::make_tuple("no", false),
                                           std::make_tuple("dusk-dawn", true),
                                           std::make_tuple("automatic", true),
                                           std::make_tuple("24/7", true),
                                           std::make_tuple("disused", false),
                                           std::make_tuple("", false)) // no lit tag
);
