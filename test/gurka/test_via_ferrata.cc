// following the formating of other tests here
#include <gtest/gtest.h>
#include "gurka.h"

using namespace valhalla;

// Creating the test case 
class Testing : public testing::Test{
    // Following the example of the other test cases present
    protected:
        static gurka::map map;
        static void SetUpTestSuite() {
            // give a map layout to test
            const std::string ascii_map = R"(
                A----B
                |    |
                C----D
            )";
            // Define each road or way
            const gurka::ways ways = {
                {"AB", {{"highway", "via_ferrata"}}}
                {"BC", {{"highway", "residential"}}},
                {"CD", {{"highway", "residential"}}},
                {"DA", {{"highway", "residential"}}}
            };
            // Convert the ASCII map to a layout
            const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
            // Build the map 
            auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/test_via_ferrata");

        }
};

gurka::map Testing::map = {}

// Define the test case 
TEST_F(Testing, CheckTesting){
    auto result = gurka::route(map, "A", "B", "pedestrian");
    // verify the result
    gurka::assert::osrm::expect_steps(result, {"A", "B"});

}