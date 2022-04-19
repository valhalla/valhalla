#include <cstdint>

#include "baldr/rapidjson_utils.h"
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <unordered_map>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "config.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/servicedays.h"
#include "valhalla/proto/transit.pb.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

Transit read_pbf(const std::string& file_name) {
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if (!file) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(
      static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  auto limit = std::max(static_cast<size_t>(1), buffer.size() * 2);
#if GOOGLE_PROTOBUF_VERSION >= 3006000
  cs.SetTotalBytesLimit(limit);
#else
  cs.SetTotalBytesLimit(limit, limit);
#endif
  Transit transit;
  if (!transit.ParseFromCodedStream(&cs)) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  return transit;
}

// Get PBF transit data given a GraphId / tile
Transit read_pbf(const GraphId& id, const std::string& transit_dir, std::string& file_name) {
  std::string fname = GraphTile::FileSuffix(id);
  fname = fname.substr(0, fname.size() - 3) + "pbf";
  file_name = transit_dir + filesystem::path::preferred_separator + fname;
  Transit transit;
  transit = read_pbf(file_name);
  return transit;
}

// Log all scheduled departures from a stop
void LogDepartures(const Transit& transit, const GraphId& stopid, std::string& file) {

  std::size_t slash_found = file.find_last_of("/\\");
  std::string directory = file.substr(0, slash_found);

  filesystem::recursive_directory_iterator transit_file_itr(directory);
  filesystem::recursive_directory_iterator end_file_itr;

  LOG_INFO("Departures:");

  // for each tile.
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (filesystem::is_regular_file(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      std::string ext = transit_file_itr->path().extension().string();
      std::string file_name = fname.substr(0, fname.size() - ext.size());

      // make sure we are looking at a pbf file
      if ((ext == ".pbf" && fname == file) ||
          (file_name.substr(file_name.size() - 4) == ".pbf" && file_name == file)) {

        Transit spp;
        {
          // already loaded
          if (ext == ".pbf") {
            spp = transit;
          } else {
            spp = read_pbf(fname);
          }
        }

        if (spp.stop_pairs_size() == 0) {
          if (transit.nodes_size() > 0) {
            LOG_ERROR("Tile " + fname + " has 0 schedule stop pairs but has " +
                      std::to_string(transit.nodes_size()) + " stops");
          }
        }

        // Iterate through the stop pairs in this tile and form Valhalla departure
        // records
        uint32_t tileid;
        for (uint32_t i = 0; i < spp.stop_pairs_size(); i++) {
          const Transit_StopPair& sp = spp.stop_pairs(i);

          // Skip stop pair if either stop graph Id is invalid
          GraphId orig_graphid = GraphId(sp.origin_graphid());
          if (!orig_graphid.Is_Valid() || !GraphId(sp.destination_graphid()).Is_Valid()) {
            continue;
          }

          if (orig_graphid == stopid) {

            int total_seconds = sp.origin_departure_time();
            int seconds = total_seconds % 60;
            int minutes = (total_seconds / 60) % 60;
            int hours = total_seconds / 3600;

            std::stringstream ss;
            ss << std::setfill('0') << std::setw(2) << hours;
            ss << ":";
            ss << std::setfill('0') << std::setw(2) << minutes;
            ss << ":";
            ss << std::setfill('0') << std::setw(2) << seconds;

            std::string dow;
            uint32_t counter = 0;
            for (const auto& service_day : sp.service_days_of_week()) {

              switch (counter) {
                case 0:
                  if (service_day) {
                    if (!dow.empty()) {
                      dow += ", ";
                    }
                    dow += "Sunday";
                  }
                  break;
                case 1:
                  if (service_day) {
                    if (!dow.empty()) {
                      dow += ", ";
                    }
                    dow += "Monday";
                  }
                  break;
                case 2:
                  if (service_day) {
                    if (!dow.empty()) {
                      dow += ", ";
                    }
                    dow += "Tuesday";
                  }
                  break;
                case 3:
                  if (service_day) {
                    if (!dow.empty()) {
                      dow += ", ";
                    }
                    dow += "Wednesday";
                  }
                  break;
                case 4:
                  if (service_day) {
                    if (!dow.empty()) {
                      dow += ", ";
                    }
                    dow += "Thursday";
                  }
                  break;
                case 5:
                  if (service_day) {
                    if (!dow.empty()) {
                      dow += ", ";
                    }
                    dow += "Friday";
                  }
                  break;
                case 6:
                  if (service_day) {
                    if (!dow.empty()) {
                      dow += ", ";
                    }
                    dow += "Saturday";
                  }
                  break;
              }
              counter++;
            }

            auto d = date::floor<date::days>(DateTime::pivot_date_);
            std::string added_dates;
            for (const auto& day : sp.service_added_dates()) {

              if (!added_dates.empty()) {
                added_dates += ", ";
              }
              date::sys_days adddate = date::sys_days(date::year_month_day(d + date::days(day)));
              added_dates += to_iso_extended_string(adddate);
            }

            date::sys_days start_date =
                date::sys_days(date::year_month_day(d + date::days(sp.service_start_date())));
            date::sys_days end_date =
                date::sys_days(date::year_month_day(d + date::days(sp.service_end_date())));

            LOG_INFO(" Route: " + std::to_string(sp.route_index()) +
                     " Trip: " + std::to_string(sp.trip_id()) + " Dep Time: " + ss.str() +
                     " DOW: " + dow + " Added dates: " + added_dates +
                     " Start Date: " + to_iso_extended_string(start_date) +
                     " End Date: " + to_iso_extended_string(end_date));
          }
        }
      }
    }
  }
}

void LogSchedule(const std::string& transit_dir,
                 GraphId& originid,
                 const GraphId& destid,
                 const uint32_t tripid,
                 const std::string& time,
                 const Transit& transit,
                 std::string& file,
                 const uint8_t local_level) {

  std::size_t slash_found = file.find_last_of("/\\");
  std::string directory = file.substr(0, slash_found);

  filesystem::recursive_directory_iterator transit_file_itr(directory);
  filesystem::recursive_directory_iterator end_file_itr;

  // for each tile.
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (filesystem::is_regular_file(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      std::string ext = transit_file_itr->path().extension().string();
      std::string file_name = fname.substr(0, fname.size() - ext.size());

      // make sure we are looking at a pbf file
      if ((ext == ".pbf" && fname == file) ||
          (file_name.substr(file_name.size() - 4) == ".pbf" && file_name == file)) {

        Transit spp;
        {
          // already loaded
          if (ext == ".pbf") {
            spp = transit;
          } else {
            spp = read_pbf(fname);
          }
        }

        if (spp.stop_pairs_size() == 0) {
          if (transit.nodes_size() > 0) {
            LOG_ERROR("Tile " + fname + " has 0 schedule stop pairs but has " +
                      std::to_string(transit.nodes_size()) + " stops");
          }
        }

        // Check if there are no schedule stop pairs in this tile
        if (spp.stop_pairs_size() == 0) {
          LOG_ERROR("No stop pairs in the PBF tile");
          return;
        }
        // Iterate through the stop pairs in this tile and form Valhalla departure
        // records
        uint32_t tileid;
        GraphId orig_graphid;
        std::string origin_time;
        for (uint32_t i = 0; i < spp.stop_pairs_size(); i++) {
          const Transit_StopPair& sp = spp.stop_pairs(i);

          // Skip stop pair if either stop graph Id is invalid
          orig_graphid = GraphId(sp.origin_graphid());
          if (!orig_graphid.Is_Valid() || !GraphId(sp.destination_graphid()).Is_Valid()) {
            continue;
          }
          // do we have the correct stop?
          if (orig_graphid == originid) {
            // do we have the correct trip?
            if (sp.trip_id() == tripid) {

              int total_seconds = sp.origin_departure_time();
              int seconds = total_seconds % 60;
              int minutes = (total_seconds / 60) % 60;
              int hours = total_seconds / 3600;

              std::stringstream ss;
              ss << std::setfill('0') << std::setw(2) << hours;
              ss << ":";
              ss << std::setfill('0') << std::setw(2) << minutes;
              ss << ":";
              ss << std::setfill('0') << std::setw(2) << seconds;

              // do we have the correct time?
              if (time == ss.str()) {
                // now lets save the destination time and id...this is our new origin
                total_seconds = sp.destination_arrival_time();
                seconds = total_seconds % 60;
                minutes = (total_seconds / 60) % 60;
                hours = total_seconds / 3600;

                ss.str("");
                ss << std::setfill('0') << std::setw(2) << hours;
                ss << ":";
                ss << std::setfill('0') << std::setw(2) << minutes;
                ss << ":";
                ss << std::setfill('0') << std::setw(2) << seconds;
                origin_time = ss.str();
                originid = GraphId(sp.destination_graphid());

                LOG_INFO("Tile : " + std::to_string(orig_graphid.tileid()) + "\tTrip:\t" +
                         sp.trip_headsign() + "\tDep Time:\t" + time + "\tArr Time:\t" + origin_time +
                         "\tOrigin ----> Dest\t" + sp.origin_onestop_id() + " ----> " +
                         sp.destination_onestop_id());
                if (destid == originid) { // we are done.
                  originid = GraphId();
                  origin_time = "";
                }

                break;
              }
            }
          }
        }

        while (originid.Is_Valid()) {

          GraphId tile(originid.tileid(), local_level, 0);
          std::string file_name;
          Transit transit = read_pbf(tile, transit_dir, file_name);
          LogSchedule(transit_dir, originid, destid, tripid, origin_time, transit, file_name,
                      local_level);
        }
      }
    }
  }
}
// Log the list of routes within the tile
void LogRoutes(const Transit& transit) {
  LOG_INFO("Routes:");
  for (uint32_t i = 0; i < transit.routes_size(); i++) {
    const Transit_Route& r = transit.routes(i);
    LOG_INFO("Route idx = " + std::to_string(i) + ": " + r.name() + "," + r.route_long_name());
  }
}

GraphId GetGraphId(Transit& transit, const std::string& onestop_id) {
  for (uint32_t i = 0; i < transit.nodes_size(); i++) {
    const Transit_Node& node = transit.nodes(i);
    if (node.onestop_id() == onestop_id) {
      LOG_INFO("Node: " + node.name());
      return GraphId(node.graphid());
    }
  }
  return GraphId();
}

// Main method for testing a single path
int main(int argc, char* argv[]) {
  // args
  std::string config;
  float o_lng, o_lat, d_lng, d_lat;
  std::string o_onestop_id, d_onestop_id, time;
  int tripid;

  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_query_transit",
      "valhalla_query_transit " VALHALLA_VERSION "\n\n"
      "valhalla_query_transit is a simple command line test tool to log transit stop info.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("o_lat", "Origin latitude", cxxopts::value<float>(o_lat))
      ("o_lng", "Origin longitude", cxxopts::value<float>(o_lng))
      ("d_lat", "Destination latitude", cxxopts::value<float>(d_lat))
      ("d_lng", "Destination longitude", cxxopts::value<float>(d_lng))
      ("o,o_onestop_id", "Origin OneStop ID", cxxopts::value<std::string>(o_onestop_id))
      ("d,d_onestop_id", "Destination OneStop ID", cxxopts::value<std::string>(d_onestop_id))
      ("i,tripid", "Trip ID", cxxopts::value<int>(tripid)->default_value("0"))
      ("t,time", "Time", cxxopts::value<std::string>(time))
      ("c,config", "Config path", cxxopts::value<std::string>(config));
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "valhalla_query_transit " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    if (!result.count("config") || !filesystem::is_regular_file(filesystem::path(config))) {
      std::cerr << "Configuration file is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }

    for (const auto& arg : std::vector<std::string>{"o_onestop_id", "o_lat", "o_lng", "conf"}) {
      if (result.count(arg) == 0) {
        std::cerr << "The <" << arg << "> argument was not provided, but is mandatory\n\n";
        std::cerr << options.help() << "\n";
        return EXIT_FAILURE;
      }
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // Read config
  boost::property_tree::ptree pt;
  rapidjson::read_json(config, pt);

  LOG_INFO("Read config");

  // Bail if no transit dir
  auto transit_dir = pt.get_optional<std::string>("mjolnir.transit_dir");
  if (!transit_dir || !filesystem::exists(*transit_dir) || !filesystem::is_directory(*transit_dir)) {
    LOG_INFO("Transit directory not found.");
    return 0;
  }

  // Get the tile
  PointLL stopll(o_lng, o_lat);
  auto local_level = TileHierarchy::levels().back().level;
  auto tiles = TileHierarchy::levels().back().tiles;
  uint32_t tileid = tiles.TileId(stopll);
  LOG_INFO("Origin Tile " + std::to_string(tileid));

  // Read transit tile
  GraphId tile(tileid, local_level, 0);
  std::string file_name;
  Transit transit = read_pbf(tile, *transit_dir, file_name);

  // Get the graph Id of the stop
  GraphId originid = GetGraphId(transit, o_onestop_id);

  if (tripid == 0 || time.empty()) {
    // Log departures from this stop
    LogDepartures(transit, originid, file_name);

    // Log routes in this tile
    LogRoutes(transit);
  } else {

    GraphId destid = GraphId();
    if (!o_onestop_id.empty()) {

      // Get the tile
      PointLL stopll(d_lng, d_lat);
      uint32_t tileid = tiles.TileId(stopll);

      // Read transit tile
      GraphId tile(tileid, local_level, 0);
      std::string file_name;
      Transit transit = read_pbf(tile, *transit_dir, file_name);

      LOG_INFO("Dest Tile " + std::to_string(tileid));
      // Get the graph Id of the stop
      destid = GetGraphId(transit, d_onestop_id);
    }

    LOG_INFO("Schedule:");
    LogSchedule(*transit_dir, originid, destid, tripid, time, transit, file_name, local_level);
  }

  return EXIT_SUCCESS;
}
