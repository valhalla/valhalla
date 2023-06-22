#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

TEST(Standalone, NotThruPruning) {
  constexpr double gridsize_metres = 100;

  const std::string ascii_map = R"(
            A   
            | 
            |
            |
        B---C---D
        |       |
        |       |
        |   7   |                                      E 
        F-------G                                       \
        |       |                                        \1
        H-------I---J                                     \
        |       |                                          \
        |       |                                           \3
        K-------L                                            \
        |                          M                          \
        |                          |                           \
        N----O                     |                            \
        |                          P              Q              |
        |                           \             |              |
        R                            \            |              | 
         \      4                    |            |              |
          S-----------T------2-------U------------V--------------W--------X
           \         /       
            \       /
             \--Y--/6
               /
     Z---------
          5
    )";

  const gurka::ways ways = {
      {"AC", {{"highway", "service"}}},
      {"BCD", {{"highway", "service"}}},
      {"DG", {{"highway", "service"}}},
      {"FG", {{"highway", "service"}}},
      {"FB", {{"highway", "service"}}},
      {"FH", {{"highway", "service"}}},
      {"HIJ", {{"highway", "service"}}},
      {"IG", {{"highway", "service"}}},
      {"KL", {{"highway", "service"}}},
      {"LI", {{"highway", "footway"}}},
      {"HKNR", {{"highway", "residential"}}},
      {"NO", {{"highway", "service"}, {"access", "private"}}},
      {"RS", {{"highway", "residential"}}},
      {"TS", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"SYT", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"YZ", {{"highway", "residential"}}},
      {"WVU", {{"highway", "residential"}}},
      {"UT", {{"highway", "residential"}}},
      {"WE", {{"highway", "tertiary"}}},
      {"XW", {{"highway", "tertiary"}}},
      {"UPM", {{"highway", "residential"}}},
      {"VQ", {{"highway", "residential"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "TS", "from"},
           {gurka::way_member, "RS", "to"},
           {gurka::node_member, "S", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_straight_on"},
       }},
      {{
           {gurka::way_member, "SYT", "from"},
           {gurka::way_member, "UT", "to"},
           {gurka::node_member, "T", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_straight_on"},
       }},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_not_thru_pruning",
                               build_config);

  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  // mark these edges with not_thru true...make the test look like real world data
  std::vector<GraphId> not_thru_edgeids;
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "S", "R")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "R", "N")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "N", "K")));

  test::customize_edges(map.config, [&not_thru_edgeids](const GraphId& edgeid, DirectedEdge& edge) {
    if (std::find(not_thru_edgeids.begin(), not_thru_edgeids.end(), edgeid) !=
        not_thru_edgeids.end()) {
      edge.set_not_thru(true);
    }
  });

  not_thru_edgeids.clear();

  // mark these edges with not_thru false...make the test look like real world data
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "R", "S")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "N", "R")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "K", "N")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "U", "T")));
  not_thru_edgeids.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, "T", "U")));

  test::customize_edges(map.config, [&not_thru_edgeids](const GraphId& edgeid, DirectedEdge& edge) {
    if (std::find(not_thru_edgeids.begin(), not_thru_edgeids.end(), edgeid) !=
        not_thru_edgeids.end()) {
      edge.set_not_thru(false);
    }
  });

  // trivial path test.  uses time dep forward A*
  auto result = gurka::do_action(valhalla::Options::route, map, {"4", "6"}, "auto");
  gurka::assert::raw::expect_path(result, {"TS", "RS", "HKNR", "HKNR", "HKNR", "FH", "FG", "IG",
                                           "HIJ", "HKNR", "HKNR", "HKNR", "RS", "SYT", "SYT"});

  // multi-waypoint test.  uses bidir A*
  result = gurka::do_action(valhalla::Options::route, map, {"1", "2", "3"}, "auto",
                            {{"/locations/1/type", "break_through"}});
  gurka::assert::raw::expect_path(result, {"WE",   "WVU",  "WVU", "UT",  "UT", "TS",  "RS",   "HKNR",
                                           "HKNR", "HKNR", "FH",  "FG",  "IG", "HIJ", "HKNR", "HKNR",
                                           "HKNR", "RS",   "SYT", "SYT", "UT", "WVU", "WVU",  "WE"});

  // dest on not_thru edge test.  uses bidir A*
  result = gurka::do_action(valhalla::Options::route, map, {"2", "5"}, "auto");
  gurka::assert::raw::expect_path(result, {"UT", "TS", "RS", "HKNR", "HKNR", "HKNR", "FH", "FG", "IG",
                                           "HIJ", "HKNR", "HKNR", "HKNR", "RS", "SYT", "YZ"});

  // dest at not_thru area test.  uses bidir A*
  result = gurka::do_action(valhalla::Options::route, map, {"2", "7"}, "auto");
  gurka::assert::raw::expect_path(result, {"UT", "TS", "RS", "HKNR", "HKNR", "HKNR", "FH", "FG"});

  // dest at not_thru area test.  uses bidir A*
  result = gurka::do_action(valhalla::Options::route, map, {"1", "7"}, "auto");
  gurka::assert::raw::expect_path(result, {"WE", "WVU", "WVU", "UT", "TS", "RS", "HKNR", "HKNR",
                                           "HKNR", "FH", "FG"});
}
