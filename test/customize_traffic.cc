#include <algorithm>
#include <array>
#include <iostream>
#include <random>
#include <string>
#include <utility>

#include "baldr/graphreader.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "sif/autocost.h"
#include "sif/costfactory.h"
#include "test.h"
#include "thor/bidirectional_astar.h"
#include "thor/unidirectional_astar.h"
#include <valhalla/proto/options.pb.h>

#include <boost/tokenizer.hpp>

#include <chrono>
#include <cstdint>
#include <thread>

// #include <boost/filesystem.hpp>
// #include <boost/filesystem/operations.hpp>
// #include <boost/filesystem/path.hpp>
// namespace fs = boost::filesystem;

using namespace valhalla;
using namespace test;
namespace {

// TODO: only support one edge id each time. To be improve performance, List shall be supported
void customize_traffic(const boost::property_tree::ptree& config,
                       baldr::GraphId& target_edge_id,
                       const int target_speed) {
  // Make some updates to the traffic .tar file.
  // Generate traffic data
  std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
      generate_traffic = [&target_edge_id, &target_speed](baldr::GraphReader& reader,
                                                          baldr::TrafficTile& tile, int index,
                                                          baldr::TrafficSpeed* current) -> void {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto edge_id = baldr::GraphId(tile_id.tileid(), tile_id.level(), index);
    if (edge_id == target_edge_id) {
      current->breakpoint1 = 255;
      current->overall_encoded_speed = target_speed >> 1;
      current->encoded_speed1 = target_speed >> 1;
    }
  };
  test::customize_live_traffic_data(config, generate_traffic);
}

// TODO: only support one edge id each time. To be improve performance, List shall be supported
void customize_traffic(const boost::property_tree::ptree& config, const std::vector<CSV_Speed>& ts) {
  test::customize_live_traffic_data(config, ts);
}

// TODO: only support one edge id each time. To be improve performance, List shall be supported
void show_traffic_fake(const boost::property_tree::ptree& config) {
  // loop over all tiles in the tileset
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));
  auto tile_dir = config.get<std::string>("mjolnir.tile_dir");
  for (const auto& tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(baldr::GraphId(tile_id));
    LOG_INFO("tile id = " + std::to_string(tile_id));
    GraphId edgeid = tile_id;
    for (size_t j = 0; j < tile->header()->directededgecount(); ++j, ++edgeid) {
      auto edge = tile->directededge(edgeid);

      if (tile->GetSpeed(edge, 255, 1) !=
          tile->GetSpeed(tile->directededge(j), kConstrainedFlowMask, 1))
        LOG_INFO(std::to_string(j) + ": traffic speed: " +
                 std::to_string(tile->GetSpeed(edge, 255, 1)) + "; graph speed: " +
                 std::to_string(tile->GetSpeed(tile->directededge(j), kConstrainedFlowMask, 1)));
    }
  }
}

// create fake traffic for (100/step) % of edges
void customize_traffic_fake(const boost::property_tree::ptree& config, const int step) {
  std::function<void(baldr::GraphReader&, baldr::TrafficTile&, int, baldr::TrafficSpeed*)>
      generate_traffic = [&step](baldr::GraphReader& reader, baldr::TrafficTile& tile, int index,
                                 baldr::TrafficSpeed* current) -> void {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto graph_tile = reader.GetGraphTile(baldr::GraphId(tile_id));
    auto edge_id = baldr::GraphId(tile_id.tileid(), tile_id.level(), index);
    auto edge = graph_tile->directededge(edge_id);

    // reduce the speed to 50% of graph tile speed
    if (index % step == 0) {
      auto speed = static_cast<int>(graph_tile->GetSpeed(edge, 255, 1) * 0.5);
      current->breakpoint1 = 255;
      current->overall_encoded_speed = speed >> 1;
      current->encoded_speed1 = speed >> 1;
    }
    // create blockage
    else if (index % 100000 == 0) {
      auto speed = static_cast<int>(graph_tile->GetSpeed(edge, 255, 1) * 0);
      current->breakpoint1 = 255;
      current->overall_encoded_speed = speed >> 1;
      current->encoded_speed1 = speed >> 1;
    }
  };
  test::customize_live_traffic_data(config, generate_traffic);
}

std::vector<CSV_Speed> ParseTrafficFile(const std::string filename,
                                        const boost::property_tree::ptree& pt) {

  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};
  std::vector<CSV_Speed> ts;
  std::string line;
  std::ifstream file(filename);
  uint32_t line_num = 0;

  std::vector<std::pair<uint32_t, uint32_t>> speed_data;
  baldr::GraphId tileid;
  baldr::GraphId prev_tileid{baldr::kInvalidGraphId};
  if (file.is_open()) {
    // for each row in the file
    while (getline(file, line) && ++line_num) {
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      uint32_t index = 0, speed = 0;

      // parse each column
      for (const auto& t : tok) {
        switch (field_num) {
          case 0:
            tileid = baldr::GraphId{std::stoull(t)};
            break;
          case 1:
            index = std::stoi(t);
            break;
          case 2:
            speed = std::stoi(t);
            break;
          default:
            break;
        }
        field_num++;
      } // for

      if (prev_tileid.value == kInvalidGraphId) {
        prev_tileid = tileid;
      }
      if (tileid.value != prev_tileid.value && !file.eof()) {
        std::vector<std::pair<uint32_t, uint32_t>> temp;
        temp.swap(speed_data);
        ts.emplace_back(CSV_Speed{prev_tileid, std::move(temp)});

        prev_tileid = tileid;
        speed_data.emplace_back(std::make_pair(index, speed));
      } else {
        speed_data.emplace_back(std::make_pair(index, speed));
      }
    } // while

    // insert the last tile
    ts.emplace_back(CSV_Speed{tileid, std::move(speed_data)});
  } // if
  file.close();
  return ts;
}

} // namespace

int main(int argc, char** argv) {

  boost::property_tree::ptree pt;
  rapidjson::read_json("/SDD_datadrive/valhalla/deu/valhalla_deu.json", pt);
  auto ts = ParseTrafficFile("/SDD_datadrive/here-traffic/traffic_speed_deu.csv", pt);
  LOG_INFO("ParseTrafficFile finished");

  // # 24848 , 332
  // auto tile_id = baldr::GraphId{24848};
  // auto tgt_edge_id = baldr::GraphId{tile_id.tileid(), tile_id.level(), 3189};
  // // Read the information
  // GraphReader reader(pt.get_child("mjolnir"));
  // auto tile = reader.GetGraphTile(baldr::GraphId(tile_id));
  // auto edge = tile->directededge(tgt_edge_id);

  // LOG_INFO("speed: " + std::to_string(tile->GetSpeed(edge, 255, 1)));

  // test::build_live_traffic_data(pt);
  // LOG_INFO("Build the default traffic data with dummy values");

  // fs::path data_dir(pt.get<std::string>("mjolnir.traffic_extract").c_str());
  // if (fs::exists(data_dir)) {
  //   LOG_INFO("traffic available");
  // } else {
  //   LOG_INFO("traffic not available");
  // }

  // int interval = 60;
  // while (true) {

  //   auto start = std::chrono::steady_clock::now();
  //   customize_traffic(pt, ts);
  //   LOG_INFO("customize_traffic finished");
  //   auto stop = std::chrono::steady_clock::now();

  //   auto duration =
  //       0.000001 * std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count();

  //   int64_t sleep = interval * 1000 - duration;
  //   LOG_INFO("Sleep for " + std::to_string(sleep) + " ms");
  //   std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
  // }

  return 0;
  // for (auto x : ts) {
  //   LOG_INFO(std::to_string(x.tileid.value) + ", size=" + std::to_string(x.speed_data.size()));
  //   for (auto y : x.speed_data) {
  //     std::cout << y.first << ", " << y.second << std::endl;
  //   }
  // }

  // customize_traffic_fake(pt, 20);
  // LOG_INFO("update the fake traffic for 5% of edges");

  // show the difference between traffic speed and graph speed
  // show_traffic_fake(pt);

  // customize traffic for single edge
  //   // Boebingen, Lessingstraße: way id: 13865735: 0,621568686498,1,621770013090
  //   auto tgt_edge_id = baldr::GraphId{621568686498};
  //   customize_traffic(pt, tgt_edge_id, 30); // original: 30

  //   tgt_edge_id = baldr::GraphId{621770013090};
  //   customize_traffic(pt, tgt_edge_id, 30); // original: 30

  //   // STR > Sifi A81: 309175327,1,3802388259096
  //   tgt_edge_id = baldr::GraphId{3802388259096};
  //   customize_traffic(pt, tgt_edge_id, 0); // original: 100

  //   // auto tgt_edge_id = baldr::GraphId{3802388259096};

  //   // Read the information
  //   GraphReader reader(pt.get_child("mjolnir"));
  //   auto tile = reader.GetGraphTile(baldr::GraphId(tgt_edge_id));
  //   auto edge = tile->directededge(tgt_edge_id);

  //   LOG_INFO("speed: " + std::to_string(tile->GetSpeed(edge, 255, 1)));
  //   LOG_INFO("wayid: " + std::to_string(tile->edgeinfo(edge).wayid()));
}
