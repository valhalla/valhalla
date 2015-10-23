#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <unordered_set>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <curl/curl.h>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/tilehierarchy.h>

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

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
    result.clear();
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_URL, url.c_str()), "Failed to set URL ");
    assert_curl(curl_easy_perform(connection.get()), "Failed to fetch url");
    ptree pt;
    read_json(result, pt);
    return pt;
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
std::unordered_set<GraphId> which_tiles(const TileHierarchy& hierarchy, const std::string& base_url) {
  //now real need to catch exceptions since we can't really proceed without this stuff
  LOG_INFO("Fetching transit feeds");
  std::unordered_set<GraphId> tiles;
  const auto& tile_level = hierarchy.levels().rbegin()->second;
  curler_t curler;
  auto feeds = curler(base_url + "/api/v1/feeds.geojson");
  for(const auto& feature : feeds.get_child("features")) {
    //should be a polygon
    auto type = feature.second.get_optional<std::string>("geometry.type");
    if(!type || *type != "Polygon") {
      LOG_WARN("Skipping non-polygonal feature: " + feature.second.get_value<std::string>());
      continue;
    }
    //TODO: don't assume its a square, check it
    const auto& coords = feature.second.get_child("geometry.coordinates").front().second;
    auto min_c = tile_level.tiles.Col(coords.begin()->second.front().second.get_value<float>());
    auto min_r = tile_level.tiles.Row(coords.begin()->second.back().second.get_value<float>());
    auto max_c = tile_level.tiles.Col(std::next(std::next(coords.begin()))->second.front().second.get_value<float>());
    auto max_r = tile_level.tiles.Row(std::next(std::next(coords.begin()))->second.back().second.get_value<float>());
    if(min_c > max_c) std::swap(min_c, max_c);
    if(min_r > max_r) std::swap(min_r, max_r);
    for(auto i = min_c; i <= max_c; ++i)
      for(auto j = min_r; j <= min_r; ++j)
        tiles.emplace(GraphId(tile_level.tiles.TileId(i,j), tile_level.level, 0));
  }
  LOG_INFO("Finished with " + std::to_string(tiles.size()) + " expected transit tiles in " +
           std::to_string(feeds.get_child("features").size()) + " feeds");
  return tiles;
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
  TileHierarchy hierarchy(pt.get_child("mjolnir.hierarchy"));
  std::string base_url(argv[2]);
  std::string key(argv[3]);

  //yes we want to curl
  curl_global_init(CURL_GLOBAL_DEFAULT);

  //go get information about what transit tiles we should be fetching
  auto transit_tiles = which_tiles(hierarchy, base_url);

  //TODO: spawn a bunch of threads to download all the tiles
  //storing the daingling_pair information of each
  curl_global_cleanup();

  //TODO: make a pass of all dangling_pairs to add back the information they
  //are missing

  //TODO: show some summary informant?
  return 0;
}
