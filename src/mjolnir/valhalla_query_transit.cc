//#include "mjolnir/transitbuilder.h"
//#include "mjolnir/graphtilebuilder.h"
#include "proto/transit.pb.h"

#include <unordered_map>
#include <map>
#include <fstream>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <boost/format.hpp>

#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/io/coded_stream.h>

#include "baldr/datetime.h"
#include "baldr/graphfsreader.h"
#include "baldr/tilefshierarchy.h"
#include "midgard/util.h"
#include "midgard/logging.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::baldr::DateTime;
using namespace valhalla::mjolnir;

namespace bpo = boost::program_options;

Transit read_pbf(const std::string& file_name) {
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if(!file) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  auto limit = std::max(static_cast<size_t>(1), buffer.size() * 2);
  cs.SetTotalBytesLimit(limit, limit);
  Transit transit;
  if(!transit.ParseFromCodedStream(&cs))
    throw std::runtime_error("Couldn't load " + file_name);
  return transit;
}

// Get PBF transit data given a GraphId / tile
Transit read_pbf(const GraphId& id, const TileHierarchy& hierarchy,
                 const std::string& transit_dir,
                 std::string& file_name) {
  std::string fname = GraphTile::FileSuffix(id, hierarchy);
  fname = fname.substr(0, fname.size() - 3) + "pbf";
  file_name = transit_dir + '/' + fname;
  Transit transit;
  transit = read_pbf(file_name);
  return transit;
}

// Log all scheduled departures from a stop
void LogDepartures(const Transit& transit, const GraphId& stopid, std::string& file) {

  std::size_t slash_found = file.find_last_of("/\\");
  std::string directory = file.substr(0,slash_found);

  boost::filesystem::recursive_directory_iterator transit_file_itr(directory);
  boost::filesystem::recursive_directory_iterator end_file_itr;

  LOG_INFO("Departures:");

  //for each tile.
  for(; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if(boost::filesystem::is_regular(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      std::string ext = transit_file_itr->path().extension().string();
      std::string file_name = fname.substr(0, fname.size() - ext.size());

      // make sure we are looking at a pbf file
      if ((ext == ".pbf" && fname == file) ||
          (file_name.substr(file_name.size()-4) == ".pbf" && file_name == file)) {

        Transit spp; {
          // already loaded
          if (ext == ".pbf")
            spp = transit;
          else
            spp = read_pbf(fname);
        }

        if (spp.stop_pairs_size() == 0) {
          if (transit.stops_size() > 0) {
            LOG_ERROR("Tile " + fname +
                      " has 0 schedule stop pairs but has " +
                      std::to_string(transit.stops_size()) + " stops");
          }
        }

        // Iterate through the stop pairs in this tile and form Valhalla departure
        // records
        uint32_t tileid;
        for (uint32_t i = 0; i < spp.stop_pairs_size(); i++) {
          const Transit_StopPair& sp = spp.stop_pairs(i);

          // Skip stop pair if either stop graph Id is invalid
          GraphId orig_graphid = GraphId(sp.origin_graphid());
          if (!orig_graphid.Is_Valid() ||
              !GraphId(sp.destination_graphid()).Is_Valid()) {
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

            LOG_INFO("LineID: " + std::to_string(sp.line_id()) +
                     " Route: " + std::to_string(sp.route_index()) +
                     " Trip: " + std::to_string(sp.trip_id()) +
                     " Dep Time: " + ss.str());
          }
        }
      }
    }
  }
}


void GetNextDeparture(const TileHierarchy hierarchy,
                      const std::string transit_dir,
                      GraphId& orig_graphid, const GraphId& destid,
                      const uint32_t tripid,std::string& origin_time,
                      const Transit transit) {

  if (orig_graphid.Is_Valid()) {

    for (uint32_t i = 0; i < transit.stop_pairs_size(); i++) {

      const Transit_StopPair& sp = transit.stop_pairs(i);
      if (sp.trip_id() == tripid && orig_graphid == GraphId(sp.origin_graphid())) {

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

        if (origin_time == ss.str()) {

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

          LOG_INFO("Trip:\t" + sp.trip_headsign() +
                   "\tDep Time:\t" + origin_time +
                   "\tArr Time:\t" + ss.str() +
                   "\tOrigin ----> Dest\t" + sp.origin_onestop_id() +
                   " ----> " + sp.destination_onestop_id());

          origin_time = ss.str();
          orig_graphid = GraphId(sp.destination_graphid());

          if (destid == orig_graphid) {//we are done.
            orig_graphid = GraphId();
            origin_time = "";
          }

          if (orig_graphid.Is_Valid())
            return;
        }
      }
    }
  }
  orig_graphid = GraphId();
  origin_time = "";
}


void LogSchedule(const TileHierarchy hierarchy, const std::string transit_dir,
                 const GraphId& originid, const GraphId& destid,
                 const uint32_t tripid, const std::string time, Transit transit,
                 std::string& file) {

  LOG_INFO("Schedule:");
  std::size_t slash_found = file.find_last_of("/\\");
  std::string directory = file.substr(0,slash_found);

  boost::filesystem::recursive_directory_iterator transit_file_itr(directory);
  boost::filesystem::recursive_directory_iterator end_file_itr;

  //for each tile.
  for(; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if(boost::filesystem::is_regular(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      std::string ext = transit_file_itr->path().extension().string();
      std::string file_name = fname.substr(0, fname.size() - ext.size());

      // make sure we are looking at a pbf file
      if ((ext == ".pbf" && fname == file) ||
          (file_name.substr(file_name.size()-4) == ".pbf" && file_name == file)) {

        Transit spp; {
          // already loaded
          if (ext == ".pbf")
            spp = transit;
          else
            spp = read_pbf(fname);
        }

        if (spp.stop_pairs_size() == 0) {
          if (transit.stops_size() > 0) {
            LOG_ERROR("Tile " + fname +
                      " has 0 schedule stop pairs but has " +
                      std::to_string(transit.stops_size()) + " stops");
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
          if (!orig_graphid.Is_Valid() ||
              !GraphId(sp.destination_graphid()).Is_Valid()) {
            continue;
          }

          //do we have the correct stop?
          if (orig_graphid == originid) {

            //do we have the correct trip?
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

              //do we have the correct time?
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
                orig_graphid = GraphId(sp.destination_graphid());

                LOG_INFO("Trip:\t" + sp.trip_headsign() +
                         "\tDep Time:\t" + time +
                         "\tArr Time:\t" + origin_time +
                         "\tOrigin ----> Dest\t" + sp.origin_onestop_id() +
                         " ----> " + sp.destination_onestop_id());

                break;
              }
            }
          }
        }

        while (orig_graphid.Is_Valid() && !origin_time.empty())
          GetNextDeparture(hierarchy, transit_dir, orig_graphid, destid, tripid, origin_time, spp);

      }
    }
  }
}

// Log the list of routes within the tile
void LogRoutes(const Transit& transit) {
  LOG_INFO("Routes:");
  for (uint32_t i = 0; i < transit.routes_size(); i++) {
    const Transit_Route& r = transit.routes(i);
    LOG_INFO("Route idx = " + std::to_string(i) + ": " + r.name() + ","
                   + r.route_long_name());
  }
}

GraphId GetGraphId(Transit& transit, const std::string& onestop_id) {
  for (uint32_t i = 0; i < transit.stops_size(); i++) {
    const Transit_Stop& stop = transit.stops(i);
    if (stop.onestop_id() == onestop_id) {
      LOG_INFO("Stop: " + stop.name());
      return GraphId(stop.graphid());
    }
  }
  return GraphId();
}

// Main method for testing a single path
int main(int argc, char *argv[]) {
  bpo::options_description options("transit_stop_query\n"
  "\nUsage: transit_stop_query [options]\n"
  "transit_stop_query is a simple command line test tool to log transit stop info."
  "\n");

  std::string config, origin, dest, time;
  float lat,lng;
  int tripid = 0;
  options.add_options()
      ("help,h", "Print this help message.")
      ("version,v", "Print the version of this software.")
      ("lat,y", boost::program_options::value<float>(&lat))
      ("lng,x", boost::program_options::value<float>(&lng))
      ("origin,o", boost::program_options::value<std::string>(&origin))
      ("dest,d", boost::program_options::value<std::string>(&dest))
      ("tripid,i", boost::program_options::value<int>(&tripid))
      ("time,t", boost::program_options::value<std::string>(&time))
      ("conf,c", bpo::value<std::string>(&config), "Valhalla configuration file");

  bpo::variables_map vm;
  try {
    bpo::store(bpo::parse_command_line(argc, argv, options), vm);
    bpo::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what();
    return false;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
  }

  for (auto arg : std::vector<std::string> { "origin", "lat", "lng", "conf" }) {
    if (vm.count(arg) == 0) {
      std::cerr << "The <" << arg
          << "> argument was not provided, but is mandatory\n\n";
      std::cerr << options << "\n";
      return EXIT_FAILURE;
    }
  }

  // Read config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  LOG_INFO("Read config");

  // Bail if no transit dir
  auto transit_dir = pt.get_optional<std::string>("mjolnir.transit_dir");
  if (!transit_dir || !boost::filesystem::exists(*transit_dir) ||
      !boost::filesystem::is_directory(*transit_dir)) {
    LOG_INFO("Transit directory not found.");
    return 0;
  }

  // Get the tile
  PointLL stopll(lng, lat);
  TileFsHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));
  auto local_level = hierarchy.levels().rbegin()->second.level;
  auto tiles = hierarchy.levels().rbegin()->second.tiles;
  uint32_t tileid = tiles.TileId(stopll);

  // Read transit tile
  GraphId tile(tileid, local_level, 0);
  std::string file_name;
  Transit transit = read_pbf(tile, hierarchy, *transit_dir, file_name);

  // Get the graph Id of the stop
  GraphId originid = GetGraphId(transit, origin);



  if (tripid == 0 || time.empty()) {
    // Log departures from this stop
    LogDepartures(transit, originid, file_name);

    // Log routes in this tile
    LogRoutes(transit);
  }
  else {
    GraphId destid = GraphId();
    if (!dest.empty()) {
      // Get the graph Id of the stop
      destid = GetGraphId(transit, dest);
    }
    LogSchedule(hierarchy, *transit_dir, originid, destid, tripid, time, transit, file_name);
  }

}
