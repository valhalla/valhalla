// following the formating of other tests here
#include <gtest/gtest.h>
#include "gurka.h"

using namespace valhalla;

class Testing : public testing::Test{
    protected:
        static gurka::map map;
        static void SetUpTestSuite() {
            const std::string ascii_map = R"(
                A----B
                |    |
                C----D
            )";
            const gurka::ways ways = {
                {"AB", {{"highway", "via_ferrata"}}}
                {"BC", {{"highway", "residential"}}},
                {"CD", {{"highway", "residential"}}},
                {"DA", {{"highway", "residential"}}}
            };
            
            const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
            auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/test_via_ferrata");

        }
};

gurka::map Testing::map = {}

TEST_F(Testing, CheckTesting){
    auto result = gurka::route(map, "A", "B", "pedestrian");
    gurka::assert::osrm::expect_steps(result, {"A", "B"});

}