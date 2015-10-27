#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <unordered_set>
#include <thread>
#include <future>
#include <random>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/format.hpp>
#include <boost/filesystem/operations.hpp>
#include <curl/curl.h>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphtile.h>

#include "proto/transit.pb.h"

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

struct logged_error_t: public std::runtime_error {
  logged_error_t(const std::string& msg):std::runtime_error(msg) {
    LOG_ERROR(msg);
  }
};

struct curler_t {
  curler_t():connection(curl_easy_init(), [](CURL* c){curl_easy_cleanup(c);}) {
    if(connection.get() == nullptr)
      throw logged_error_t("Failed to created CURL connection");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ERRORBUFFER, error), "Failed to set error buffer");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_FOLLOWLOCATION, 1L), "Failed to set redirect option ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEDATA, &result), "Failed to set write data ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEFUNCTION, write_callback), "Failed to set writer ");
  }
  //for now we only need to handle json
  //with templates we could return a string or whatever
  ptree operator()(const std::string& url) {
    result.str("");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_URL, url.c_str()), "Failed to set URL ");
    assert_curl(curl_easy_perform(connection.get()), "Failed to fetch url");
    ptree pt;
    try { read_json(result, pt); } catch(...) { throw logged_error_t(result.str()); }
    return pt;
  }
  std::string last() const {
    return result.str();
  }
protected:
  void assert_curl(CURLcode code, const std::string& msg){
    if(code != CURLE_OK)
      throw logged_error_t(msg + error);
  };
  static size_t write_callback(char *in, size_t block_size, size_t blocks, std::stringstream *out) {
    if(!out) return static_cast<size_t>(0);
    out->write(in, block_size * blocks);
    return block_size * blocks;
  }
  std::shared_ptr<CURL> connection;
  char error[CURL_ERROR_SIZE];
  std::stringstream result;
};

//TODO: update this call to get only the tiles that have changed since last time
std::unordered_set<GraphId> which_tiles(const ptree& pt) {
  //now real need to catch exceptions since we can't really proceed without this stuff
  LOG_INFO("Fetching transit feeds");
  TileHierarchy hierarchy(pt.get_child("mjolnir.hierarchy"));
  std::unordered_set<GraphId> tiles;
  const auto& tile_level = hierarchy.levels().rbegin()->second;
  curler_t curler;
  auto feeds = curler(pt.get<std::string>("base_url") + "/api/v1/feeds.geojson");
  for(const auto& feature : feeds.get_child("features")) {
    //should be a polygon
    auto type = feature.second.get_optional<std::string>("geometry.type");
    if(!type || *type != "Polygon") {
      LOG_WARN("Skipping non-polygonal feature: " + feature.second.get_value<std::string>());
      continue;
    }
    //grab the tile row and column ranges for the max box around the polygon
    int32_t min_c = tile_level.tiles.ncolumns(), max_c = 0, min_r = tile_level.tiles.nrows(), max_r = 0;
    for(const auto& coord :feature.second.get_child("geometry.coordinates").front().second) {
      auto c = tile_level.tiles.Col(coord.second.front().second.get_value<float>());
      auto r = tile_level.tiles.Row(coord.second.back().second.get_value<float>());
      if(c < min_c) min_c = c;
      if(c > max_c) max_c = c;
      if(r < min_r) min_r = r;
      if(r > max_r) max_r = r;
    }
    for(auto i = min_c; i <= max_c; ++i)
      for(auto j = min_r; j <= min_r; ++j)
        tiles.emplace(GraphId(tile_level.tiles.TileId(i,j), tile_level.level, 0));
  }
  LOG_INFO("Finished with " + std::to_string(tiles.size()) + " expected transit tiles in " +
           std::to_string(feeds.get_child("features").size()) + " feeds");
  return tiles;
}

using fetch_itr_t = std::unordered_set<GraphId>::const_iterator;
void fetch_tiles(const ptree& pt, fetch_itr_t start, fetch_itr_t end, std::promise<std::list<GraphId> >& promise) {
  TileHierarchy hierarchy(pt.get_child("mjolnir.hierarchy"));
  const auto& tiles = hierarchy.levels().rbegin()->second.tiles;
  std::list<GraphId> dangling;
  curler_t curler;
  auto now = time(nullptr);
  auto* utc = gmtime(&now); utc->tm_year += 1900; ++utc->tm_mon; //TODO: use timezone code?
  std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_int_distribution<size_t> distribution(static_cast<size_t>(50), static_cast<size_t>(250));

  //for each tile
  for(; start != end; ++start) {
    auto bbox = tiles.TileBounds(start->fields.tileid);
    ptree response;
    Transit tile;

    //pull out all the STOPS
    std::unordered_map<std::string, uint64_t> stops;
    auto extra_params = (boost::format("&service_from_date=%1%-%2%-%3%&api_key=%4%")
      % utc->tm_year % utc->tm_mon % utc->tm_mday % pt.get<std::string>("api_key")).str();
    boost::optional<std::string> request = (boost::format(pt.get<std::string>("base_url") +
      "/api/v1/stops?per_page=5000&bbox=%1%,%2%,%3%,%4%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str();
    while(request) {
      //grab some stuff
      try {
        LOG_INFO(*request + extra_params);
        response = curler(*request + extra_params);
      }//if it doesnt come back, take a rest and try again
      catch(const std::exception& e) {
        LOG_WARN(e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)));
        continue;
      }

      //copy stops in, keeping map of stopid to graphid
      try {
        for(const auto& stop_pt : response.get_child("stops")) {
          const auto& ll_pt = stop_pt.second.get_child("geometry.coordinates");
          auto lon = ll_pt.front().second.get_value<float>();
          auto lat = ll_pt.back().second.get_value<float>();
          if(!bbox.Contains({lon, lat}))
            continue;
          auto* stop = tile.add_stops();
          stop->set_lon(lon);
          stop->set_lat(lat);
          stop->set_onestop_id(stop_pt.second.get<std::string>("onestop_id"));
          stop->set_graphid(*start + stops.size());
          stops.emplace(stop->onestop_id(), stop->graphid());
          //TODO: copy rest of attributes
        }
        //please sir may i have some more?
        request = response.get_optional<std::string>("meta.next");
      }//if it doesnt come back, take a rest and try again
      catch(const std::exception& e) {
        LOG_WARN(curler.last());
        std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)));
        continue;
      }
      //break; //TODO: testing, remove
    }

    //um yeah.. we need these
    if(stops.size() == 0)
      continue;

    //pull out all ROUTES
    request = (boost::format(pt.get<std::string>("base_url") +
      "/api/v1/routes?per_page=5000&bbox=%1%,%2%,%3%,%4%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str();
    std::unordered_map<std::string, size_t> routes;
    while(request) {
      //grab some stuff
      try {
        LOG_INFO(*request + extra_params);
        response = curler(*request + extra_params);
      }//if it doesnt come back, take a rest and try again
      catch(const std::exception& e) {
        LOG_WARN(e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)));
        continue;
      }

      //copy routes in, keeping track of routeid to route index
      try {
        for(const auto& route_pt : response.get_child("routes")) {
          auto* route = tile.add_routes();
          route->set_onestop_id(route_pt.second.get<std::string>("onestop_id"));
          routes.emplace(route->onestop_id(), routes.size());
          //TODO: copy rest of attributes
        }

        //please sir may i have some more?
        request = response.get_optional<std::string>("meta.next");
      }//if it doesnt come back, take a rest and try again
      catch(const std::exception& e) {
        LOG_WARN(curler.last());
        std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)));
        continue;
      }
      //break; //TODO: testing, remove
    }

    //pull out all SCHEDULE_STOP_PAIRS
    request = (boost::format(pt.get<std::string>("base_url") +
      "/api/v1/schedule_stop_pairs?per_page=5000&bbox=%1%,%2%,%3%,%4%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str();
    while(request) {
      //grab some stuff
      try {
        LOG_INFO(*request + extra_params);
        response = curler(*request + extra_params);
      }//if it doesnt come back, take a rest and try again
      catch(const std::exception& e) {
        LOG_WARN(e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)));
        continue;
      }

      //copy pairs in, noting if any dont have stops
      try {
        for(const auto& pair_pt : response.get_child("schedule_stop_pairs")) {
          auto* pair = tile.add_stop_pairs();
          //origin
          pair->set_origin_onestop_id(pair_pt.second.get<std::string>("origin_onestop_id"));
          auto origin = stops.find(pair->origin_onestop_id());
          if(origin != stops.cend())
            pair->set_origin_graphid(origin->second);
          else if(dangling.size() && dangling.back() != *start)
            dangling.push_back(*start);
          //destination
          pair->set_destination_onestop_id(pair_pt.second.get<std::string>("destination_onestop_id"));
          auto destination = stops.find(pair->destination_onestop_id());
          if(destination != stops.cend())
            pair->set_destination_graphid(destination->second);
          else if(dangling.size() && dangling.back() != *start)
            dangling.push_back(*start);
          //route
          auto route = routes.find(pair_pt.second.get<std::string>("route_onestop_id"));
          if(route == routes.cend()) {
            LOG_ERROR("No route for pair: " + pair->origin_onestop_id() + " --> " + pair->destination_onestop_id());
            tile.mutable_stop_pairs()->RemoveLast();
            continue;
          }
          pair->set_route_index(route->second);
          //TODO: copy rest of attributes
        }

        //please sir may i have some more?
        request = response.get_optional<std::string>("meta.next");
      }//if it doesnt come back, take a rest and try again
      catch(const std::exception& e) {
        LOG_WARN(curler.last());
        std::this_thread::sleep_for(std::chrono::milliseconds(distribution(generator)));
        continue;
      }
      //break; //TODO: testing, remove
    }

    //write pbf to file
    auto file_name = GraphTile::FileSuffix(*start, hierarchy);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    boost::filesystem::path transit_tile = pt.get<std::string>("mjolnir.transit_dir") + '/' + file_name;
    if (!boost::filesystem::exists(transit_tile.parent_path()))
      boost::filesystem::create_directories(transit_tile.parent_path());
    std::fstream stream(transit_tile.string(), std::ios::out | std::ios::trunc | std::ios::binary);
    tile.SerializeToOstream(&stream);
  }

  //give back the work for later
  promise.set_value(dangling);
}

std::list<GraphId> fetch(const ptree& pt, const std::unordered_set<GraphId>& tiles) {
  unsigned int thread_count = std::max(static_cast<unsigned int>(1),
                                  pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency()));
  LOG_INFO("Fetching " + std::to_string(tiles.size()) + " transit tiles with " + std::to_string(thread_count) + " threads...");

  //figure out where the work should go
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);
  std::vector<std::promise<std::list<GraphId> > > promises(threads.size());
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  fetch_itr_t tile_start, tile_end = tiles.begin();

  //make let them rip
  for (size_t i = 0; i < threads.size(); ++i) {
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    tile_start = tile_end;
    std::advance(tile_end, tile_count);
    threads[i].reset(
      new std::thread(fetch_tiles, std::cref(pt), tile_start, tile_end, std::ref(promises[i]))
    );
  }

  //let the threads finish and get the dangling list
  for (auto& thread : threads)
    thread->join();
  std::list<GraphId> dangling;
  for (auto& promise : promises) {
    try {
      dangling.splice(dangling.end(), promise.get_future().get());
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }

  LOG_INFO("Finished");
  return dangling;
}

using stitch_itr_t = std::list<GraphId>::const_iterator;
void stitch_tiles(const ptree& pt, stitch_itr_t start, stitch_itr_t end, std::mutex& lock) {
  TileHierarchy hierarchy(pt.get_child("mjolnir.hierarchy"));
  std::list<GraphId> dangling;

  //for each tile
  for(; start != end; ++start) {
    //TODO: open tile make a hash of missing stop to invalid graphid

    //TODO: add this tile to checked set
    //TODO: do while we have more to find or arent sick of searching
    //        for each tile in checked set add all neighbors not in checked set to temp set
    //        for each tile in temp set
    //          open tile, loop over stops, if stop in hash, update hash value

    //TODO: write pbf to file
  }
}

void stitch(const ptree& pt, const std::list<GraphId>& tiles) {
  unsigned int thread_count = std::max(static_cast<unsigned int>(1),
                                  pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency()));
  LOG_INFO("Stitching " + std::to_string(tiles.size()) + " transit tiles with " + std::to_string(thread_count) + " threads...");

  //figure out where the work should go
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  stitch_itr_t tile_start, tile_end = tiles.begin();
  std::mutex lock;

  //make let them rip
  for (size_t i = 0; i < threads.size(); ++i) {
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    tile_start = tile_end;
    std::advance(tile_end, tile_count);
    threads[i].reset(
      new std::thread(stitch_tiles, std::cref(pt), tile_start, tile_end, std::ref(lock))
    );
  }

  //wait for them to finish
  for (auto& thread : threads)
    thread->join();

  LOG_INFO("Finished");
}

int main(int argc, char** argv) {
  if(argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0]) << " valhalla_config transit_land_url transit_land_api_key " << std::endl;
    std::cerr << "Sample: " << std::string(argv[0]) << " conf/valhalla.json http://transit.land/ transitland-YOUR_KEY_SUFFIX" << std::endl;
    return 1;
  }

  //args and config file loading
  ptree pt;
  boost::property_tree::read_json(std::string(argv[1]), pt);
  pt.add("base_url", std::string(argv[2]));
  pt.add("api_key", std::string(argv[3]));

  //yes we want to curl
  curl_global_init(CURL_GLOBAL_DEFAULT);

  //go get information about what transit tiles we should be fetching
  auto transit_tiles = which_tiles(pt);

  //spawn threads to download all the tiles returning a list of
  //tiles that ended up having dangling stop pairs
  auto dangling_tiles = fetch(pt, transit_tiles);
  curl_global_cleanup();

  //TODO: spawn threads to connect dangling stop pairs to adjacent tiles' stops
  stitch(pt, dangling_tiles);

  //TODO: show some summary informant?
  return 0;
}
