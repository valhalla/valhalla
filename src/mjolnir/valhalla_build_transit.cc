#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <unordered_set>
#include <thread>
#include <future>
#include <random>
#include <queue>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/format.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <curl/curl.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/io/coded_stream.h>

#include "midgard/logging.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "baldr/graphtile.h"
#include "baldr/datetime.h"
#include "baldr/graphreader.h"
#include "midgard/encoded.h"

#include "mjolnir/admin.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/validatetransit.h"

#include "proto/transit.pb.h"

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

// Shape
struct Shape {
  uint32_t begins;
  uint32_t ends;
  std::vector<PointLL> shape;
};

struct Departure {
  GraphId  orig_pbf_graphid;   // GraphId in pbf tiles
  GraphId  dest_pbf_graphid;   // GraphId in pbf tiles
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t shapeid;
  uint32_t headsign_offset;
  uint32_t dep_time;
  uint32_t schedule_index;
  uint32_t frequency_end_time;
  uint16_t elapsed_time;
  uint16_t frequency;
  float    orig_dist_traveled;
  float    dest_dist_traveled;
  bool     wheelchair_accessible;
  bool     bicycle_accessible;
};

// Unique route and stop
struct TransitLine {
  uint32_t lineid;
  uint32_t routeid;
  GraphId  dest_pbf_graphid;  // GraphId (from pbf) of the destination stop
  uint32_t shapeid;
  float    orig_dist_traveled;
  float    dest_dist_traveled;
};

struct StopEdges {
  GraphId origin_pbf_graphid;        // GraphId (from pbf) of the origin stop
  std::vector<GraphId> intrastation; // List of intra-station connections
  std::vector<TransitLine> lines;    // Set of unique route/stop pairs
};

// Struct to hold stats information during each threads work
struct builder_stats {
  uint32_t no_dir_edge_count;
  uint32_t dep_count;
  uint32_t midnight_dep_count;
  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    no_dir_edge_count += other.no_dir_edge_count;
    dep_count += other.dep_count;
    midnight_dep_count += other.midnight_dep_count;
  }
};

std::string url_encode(const std::string& unencoded) {
  char* encoded = curl_escape(unencoded.c_str(), static_cast<int>(unencoded.size()));
  if(encoded == nullptr)
    throw std::runtime_error("url encoding failed");
  std::string encoded_str(encoded);
  curl_free(encoded);
  return encoded_str;
}

struct logged_error_t: public std::runtime_error {
  logged_error_t(const std::string& msg):std::runtime_error(msg) {
    LOG_ERROR(msg);
  }
};

struct curler_t {
  curler_t():connection(curl_easy_init(), [](CURL* c){curl_easy_cleanup(c);}),
    generator(std::chrono::system_clock::now().time_since_epoch().count()),
    distribution(static_cast<size_t>(300), static_cast<size_t>(700)) {
    if(connection.get() == nullptr)
      throw logged_error_t("Failed to created CURL connection");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ERRORBUFFER, error), "Failed to set error buffer");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_FOLLOWLOCATION, 1L), "Failed to set redirect option ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEDATA, &result), "Failed to set write data ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEFUNCTION, write_callback), "Failed to set writer ");
  }
  //for now we only need to handle json
  //with templates we could return a string or whatever
  ptree operator()(const std::string& url, const std::string& retry_if_no = "", bool gzip = true, boost::optional<size_t> timeout = boost::none) {
    //content encoding header
    if(gzip) {
      char encoding[] = "gzip"; //TODO: allow "identity" and "deflate"
      assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ACCEPT_ENCODING, encoding), "Failed to set gzip content header ");
    }
    //set the url
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_URL, url.c_str()), "Failed to set URL ");
    //dont stop until we have something useful!
    ptree pt;
    size_t tries = 0;
    while(++tries) {
      result.str("");
      long http_code = 0;
      std::string log_extra = "Couldn't fetch url ";
      //can we fetch this url
      LOG_DEBUG(url);
      if(curl_easy_perform(connection.get()) == CURLE_OK) {
        curl_easy_getinfo(connection.get(), CURLINFO_RESPONSE_CODE, &http_code);
        log_extra = std::to_string(http_code) + "'d ";
        //it should be 200 OK
        if(http_code == 200) {
          bool threw = false;
          try { read_json(result, pt); } catch (...) { threw = true; }
          //has to parse and have required info
          if(!threw && (retry_if_no.empty() || pt.get_child_optional(retry_if_no)))
            break;
          log_extra = "Unusable response ";
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(timeout ? *timeout : distribution(generator)));
      //dont log rate limit stuff its too frequent
      if(http_code != 429 || (tries % 10) == 0)
        LOG_WARN(log_extra + "retrying " + url);
    };
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
  std::default_random_engine generator;
  std::uniform_int_distribution<size_t> distribution;
};

std::string url(const std::string& path, const ptree& pt) {
  auto url = pt.get<std::string>("base_url") + path;
  auto key = pt.get_optional<std::string>("api_key");
  if(key)
    url += "&api_key=" + *key;
  return url;
}

//TODO: update this call to get only the tiles that have changed since last time
struct weighted_tile_t { GraphId t; size_t w; bool operator<(const weighted_tile_t& o) const { return w == o.w ? t < o.t : w < o.w; } };
std::priority_queue<weighted_tile_t> which_tiles(const ptree& pt, const std::string& feed) {
  //now real need to catch exceptions since we can't really proceed without this stuff
  LOG_INFO("Fetching transit feeds");

  auto import_level = pt.get_optional<std::string>("import_level") ? "&import_level=" +
      pt.get<std::string>("import_level") : "";

  TileHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));
  std::set<GraphId> tiles;
  const auto& tile_level = hierarchy.levels().rbegin()->second;
  curler_t curler;
  auto feeds = curler(url("/api/v1/feeds.geojson?per_page=false", pt), "features");
  for(const auto& feature : feeds.get_child("features")) {

    auto onestop_feed = feature.second.get_optional<std::string>("properties.onestop_id");
    if (feed.empty() || (onestop_feed && *onestop_feed == feed)) {

      //should be a polygon
      auto type = feature.second.get_optional<std::string>("geometry.type");
      if(!type || *type != "Polygon") {
        LOG_WARN("Skipping non-polygonal feature: " + feature.second.get_value<std::string>());
        continue;
      }
      //grab the tile row and column ranges for the max box around the polygon
      float min_x = 180, max_x = -180, min_y = 90, max_y = -90;
      for(const auto& coord :feature.second.get_child("geometry.coordinates").front().second) {
        auto x = coord.second.front().second.get_value<float>();
        auto y = coord.second.back().second.get_value<float>();
        if(x < min_x) min_x = x;
        if(x > max_x) max_x = x;
        if(y < min_y) min_y = y;
        if(y > max_y) max_y = y;
      }

      //expand the top and bottom edges of the box to account for geodesics
      min_y -= std::abs(min_y - PointLL(min_x, min_y).MidPoint({max_x, min_y}).second);
      max_y += std::abs(max_y - PointLL(min_x, max_y).MidPoint({max_x, max_y}).second);
      auto min_c = tile_level.tiles.Col(min_x), min_r = tile_level.tiles.Row(min_y);
      auto max_c = tile_level.tiles.Col(max_x), max_r = tile_level.tiles.Row(max_y);
      if(min_c > max_c) std::swap(min_c, max_c);
      if(min_r > max_r) std::swap(min_r, max_r);
      //for each tile in the polygon figure out how heavy it is and keep track of it
      for(auto i = min_c; i <= max_c; ++i)
        for(auto j = min_r; j <= max_r; ++j)
          tiles.emplace(GraphId(tile_level.tiles.TileId(i,j), tile_level.level, 0));
    }
  }
  //we want slowest to build tiles first, routes query is slowest so we weight by that
  //stop pairs is most numerous so that might want to be factored in as well
  std::priority_queue<weighted_tile_t> prioritized;
  auto now = time(nullptr);
  auto* utc = gmtime(&now); utc->tm_year += 1900; ++utc->tm_mon;
  for(const auto& tile : tiles) {
    auto bbox = tile_level.tiles.TileBounds(tile.tileid());
    auto min_y = std::max(bbox.miny(), bbox.minpt().MidPoint({bbox.maxx(), bbox.miny()}).second);
    auto max_y = std::min(bbox.maxy(), PointLL(bbox.minx(), bbox.maxy()).MidPoint(bbox.maxpt()).second);
    bbox = AABB2<PointLL>(bbox.minx(), min_y, bbox.maxx(), max_y);
    //stop count
    auto request = url((boost::format("/api/v1/stops?total=true&per_page=0&bbox=%1%,%2%,%3%,%4%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str(), pt);
    request += import_level;

    auto stops_total = curler(request, "meta.total").get<size_t>("meta.total");
    /*
    //route count
    request = url((boost::format("/api/v1/routes?total=true&per_page=0&bbox=%1%,%2%,%3%,%4%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str(), pt);
    auto routes_total = curler(request, "meta.total").get<size_t>("meta.total");
    //pair count
    request = url((boost::format("/api/v1/schedule_stop_pairs?total=true&per_page=0&bbox=%1%,%2%,%3%,%4%&service_from_date=%5%-%6%-%7%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy() % utc->tm_year % utc->tm_mon % utc->tm_mday).str(), pt);
    auto pairs_total = curler(request, "meta.total").get<size_t>("meta.total");
    */
    //we have anything we want it
    if(stops_total > 0/* || routes_total > 0|| pairs_total > 0*/) {
      prioritized.push(weighted_tile_t{tile, stops_total + 10/* + routes_total * 1000 + pairs_total*/}); //TODO: factor in stop pairs as well
      LOG_INFO(GraphTile::FileSuffix(tile, hierarchy) + " should have " + std::to_string(stops_total) +  " stops "/* +
          std::to_string(routes_total) +  " routes and " + std::to_string(pairs_total) +  " stop_pairs"*/);
    }
  }
  LOG_INFO("Finished with " + std::to_string(prioritized.size()) + " transit tiles in " +
           std::to_string(feeds.get_child("features").size()) + " feeds");
  return prioritized;
}

#define set_no_null(T, pt, path, null_value, set) {\
  auto value = pt.get<T>(path, null_value); \
  if(value != null_value) \
    set(value); \
}

void get_stops(Transit& tile, std::unordered_map<std::string, uint64_t>& stops,
    const GraphId& tile_id, const ptree& response, const AABB2<PointLL>& filter,
    bool tile_within_one_tz, const std::unordered_map<uint32_t, multi_polygon_type>& tz_polys) {
  for(const auto& stop_pt : response.get_child("stops")) {
    const auto& ll_pt = stop_pt.second.get_child("geometry.coordinates");
    auto lon = ll_pt.front().second.get_value<float>();
    auto lat = ll_pt.back().second.get_value<float>();
    if(!filter.Contains({lon, lat}))
      continue;
    auto* stop = tile.add_stops();
    stop->set_lon(lon);
    stop->set_lat(lat);
    set_no_null(std::string, stop_pt.second, "onestop_id", "null", stop->set_onestop_id);
    set_no_null(std::string, stop_pt.second, "name", "null", stop->set_name);
    stop->set_wheelchair_boarding(stop_pt.second.get<bool>("wheelchair_boarding", true));
    set_no_null(uint64_t, stop_pt.second, "tags.osm_way_id", 0, stop->set_osm_way_id);
    GraphId stop_id = tile_id;
    stop_id.fields.id = stops.size();
    stop->set_graphid(stop_id);
    stop->set_timezone(0);

    auto tz = stop_pt.second.get<std::string>("timezone", "");
    uint32_t timezone = DateTime::get_tz_db().to_index(tz);
    if (timezone == 0) {

      //fallback to tz database.
      timezone = (tile_within_one_tz) ?
                  tz_polys.begin()->first :
                  GetMultiPolyId(tz_polys, PointLL(lon, lat));
      if (timezone == 0)
        LOG_WARN("Timezone not found for stop " + stop->name());
    }

    stop->set_timezone(timezone);
    stops.emplace(stop->onestop_id(), stop_id);
    if(stops.size() == kMaxGraphId) {
      LOG_ERROR("Hit the maximum number of stops allowed and skipping the rest")
      break;
    }
  }
}

void get_routes(Transit& tile, std::unordered_map<std::string, size_t>& routes,
    const std::unordered_map<std::string, std::string>& websites,
    const std::unordered_map<std::string, std::string>& short_names, const ptree& response) {
  for(const auto& route_pt : response.get_child("routes")) {
    auto* route = tile.add_routes();
    set_no_null(std::string, route_pt.second, "onestop_id", "null", route->set_onestop_id);
    std::string vehicle_type = route_pt.second.get<std::string>("vehicle_type", "null");
    Transit_VehicleType type = Transit_VehicleType::Transit_VehicleType_kRail;
    if (vehicle_type == "tram" || vehicle_type == "tram_service")
      type = Transit_VehicleType::Transit_VehicleType_kTram;
    else if (vehicle_type == "metro")
      type = Transit_VehicleType::Transit_VehicleType_kMetro;
    else if (vehicle_type == "rail" || vehicle_type == "suburban_railway")
      type = Transit_VehicleType::Transit_VehicleType_kRail;
    else if (vehicle_type == "bus" || vehicle_type == "trolleybus_service" ||
             vehicle_type == "express_bus_service" || vehicle_type == "local_bus_service" ||
             vehicle_type == "bus_service" || vehicle_type == "shuttle_bus" ||
             vehicle_type == "demand_and_response_bus_service")
      type = Transit_VehicleType::Transit_VehicleType_kBus;
    else if (vehicle_type == "ferry")
      type = Transit_VehicleType::Transit_VehicleType_kFerry;
    else if (vehicle_type == "cablecar")
      type = Transit_VehicleType::Transit_VehicleType_kCableCar;
    else if (vehicle_type == "gondola")
      type = Transit_VehicleType::Transit_VehicleType_kGondola;
    else if (vehicle_type == "funicular")
      type = Transit_VehicleType::Transit_VehicleType_kFunicular;
    else {
      LOG_ERROR("Skipping unsupported vehicle_type: " + vehicle_type + " for route " + route->onestop_id());
      tile.mutable_routes()->RemoveLast();
      continue;
    }
    route->set_vehicle_type(type);
    set_no_null(std::string, route_pt.second, "operated_by_onestop_id", "null", route->set_operated_by_onestop_id);
    set_no_null(std::string, route_pt.second, "name", "null", route->set_name);
    set_no_null(std::string, route_pt.second, "tags.route_long_name", "null", route->set_route_long_name);
    set_no_null(std::string, route_pt.second, "tags.route_desc", "null", route->set_route_desc);
    std::string route_color = route_pt.second.get<std::string>("tags.route_color", "FFFFFF");
    std::string route_text_color = route_pt.second.get<std::string>("tags.route_text_color", "000000");
    boost::algorithm::trim(route_color);
    boost::algorithm::trim(route_text_color);
    route_color = (route_color == "null" ? "FFFFFF" : route_color);
    route_text_color = (route_text_color == "null" ? "000000" : route_text_color);

    auto website = websites.find(route->operated_by_onestop_id());
    if(website != websites.cend())
      route->set_operated_by_website(website->second);

    //use short name (e.g., BART) over long name (e.g., Bay Area Rapid Transit)
    auto short_name = short_names.find(route->operated_by_onestop_id());
    if(short_name != short_names.cend())
      route->set_operated_by_name(short_name->second);
    else set_no_null(std::string, route_pt.second, "operated_by_name", "null", route->set_operated_by_name);

    route->set_route_color(strtol(route_color.c_str(), nullptr, 16));
    route->set_route_text_color(strtol(route_text_color.c_str(), nullptr, 16));
    routes.emplace(route->onestop_id(), routes.size());
  }
}

void get_stop_patterns(Transit& tile, std::unordered_map<std::string, size_t>& shapes, const ptree& response) {
  for(const auto& shape_pt : response.get_child("route_stop_patterns")) {
    auto* shape = tile.add_shapes();
    auto shape_id = shape_pt.second.get<std::string>("onestop_id");

    std::vector<PointLL> trip_shape;
    for(const auto& geom : shape_pt.second.get_child("geometry.coordinates")) {
      auto lon = geom.second.front().second.get_value<float>();
      auto lat = geom.second.back().second.get_value<float>();
      trip_shape.emplace_back(PointLL(lon,lat));
    }
    // encode the points to reduce size
    shape->set_encoded_shape(encode7(trip_shape));

    // shapes.size()+1 because we can't have a shape id of 0.
    // 0 means shape id is not set in the transit builder.
    shape->set_shape_id(shapes.size()+1);
    shapes.emplace(shape_id, shape->shape_id());
  }
}

struct unique_transit_t {
  std::mutex lock;
  std::unordered_map<std::string, size_t> trips;
  std::unordered_map<std::string, size_t> block_ids;
  std::unordered_set<std::string> missing_routes;
  std::unordered_map<std::string, size_t> lines;
};

bool get_stop_pairs(Transit& tile, unique_transit_t& uniques, const std::unordered_map<std::string, size_t>& shapes,
                    const ptree& response, const std::unordered_map<std::string, uint64_t>& stops,
                    const std::unordered_map<std::string, size_t>& routes) {
  bool dangles = false;
  for(const auto& pair_pt : response.get_child("schedule_stop_pairs")) {
    auto* pair = tile.add_stop_pairs();

    //origin
    pair->set_origin_onestop_id(pair_pt.second.get<std::string>("origin_onestop_id"));
    auto origin = stops.find(pair->origin_onestop_id());
    if(origin != stops.cend())
      pair->set_origin_graphid(origin->second);
    else
      dangles = true;

    //destination
    pair->set_destination_onestop_id(pair_pt.second.get<std::string>("destination_onestop_id"));
    auto destination = stops.find(pair->destination_onestop_id());
    if(destination != stops.cend())
      pair->set_destination_graphid(destination->second);
    else
      dangles = true;

    //um yeah this goes nowhere
    if(pair->origin_onestop_id() == pair->destination_onestop_id()){
      tile.mutable_stop_pairs()->RemoveLast();
      continue;
    }

    //route
    auto route_id = pair_pt.second.get<std::string>("route_onestop_id");
    auto route = routes.find(route_id);
    if(route == routes.cend()) {
      uniques.lock.lock();
      if(uniques.missing_routes.find(route_id) == uniques.missing_routes.cend()) {
        LOG_ERROR("No route " + route_id);
        uniques.missing_routes.emplace(route_id);
      }
      uniques.lock.unlock();
      tile.mutable_stop_pairs()->RemoveLast();
      continue;
    }
    pair->set_route_index(route->second);

    auto frequency_start_time = pair_pt.second.get<std::string>("frequency_start_time", "null");
    auto frequency_end_time = pair_pt.second.get<std::string>("frequency_end_time", "null");
    auto frequency_headway_seconds = pair_pt.second.get<std::string>("frequency_headway_seconds", "null");
    auto origin_time = pair_pt.second.get<std::string>("origin_departure_time", "null");

    // this will be empty for non frequency trips.
    std::string frequency_time;

    if (frequency_start_time != "null" && frequency_end_time != "null" && frequency_headway_seconds != "null") {
      if (origin_time < frequency_start_time)
        LOG_WARN("Frequency frequency_start_time after origin_time: " + pair->origin_onestop_id() + " --> " + pair->destination_onestop_id());
      pair->set_frequency_end_time(DateTime::seconds_from_midnight(frequency_end_time));
      pair->set_frequency_headway_seconds(std::stoi(frequency_headway_seconds));
      frequency_time = frequency_start_time + frequency_end_time;
    }

    //uniq line id
    auto line_id = pair->origin_onestop_id() < pair->destination_onestop_id() ?
                    pair->origin_onestop_id() + pair->destination_onestop_id() + route_id + frequency_time:
                    pair->destination_onestop_id() + pair->origin_onestop_id() + route_id + frequency_time;
    uniques.lock.lock();
    auto inserted = uniques.lines.insert({line_id, uniques.lines.size()});
    pair->set_line_id(inserted.first->second);
    uniques.lock.unlock();

    //timing information
    auto dest_time = pair_pt.second.get<std::string>("destination_arrival_time", "null");
    auto start_date = pair_pt.second.get<std::string>("service_start_date", "null");
    auto end_date = pair_pt.second.get<std::string>("service_end_date", "null");
    if (origin_time == "null" || dest_time == "null" || start_date == "null" || end_date == "null") {
      LOG_ERROR("Missing timing information: " + pair->origin_onestop_id() + " --> " + pair->destination_onestop_id());
      tile.mutable_stop_pairs()->RemoveLast();
      continue;
    }

    pair->set_origin_departure_time(DateTime::seconds_from_midnight(origin_time));
    pair->set_destination_arrival_time(DateTime::seconds_from_midnight(dest_time));
    pair->set_service_start_date(DateTime::get_formatted_date(start_date).julian_day());
    pair->set_service_end_date(DateTime::get_formatted_date(end_date).julian_day());
    for(const auto& service_days : pair_pt.second.get_child("service_days_of_week")) {
      pair->add_service_days_of_week(service_days.second.get_value<bool>());
    }

    //trip
    std::string trip = pair_pt.second.get<std::string>("trip", "null");
    if (trip == "null") {
      LOG_ERROR("No trip for pair: " + pair->origin_onestop_id() + " --> " + pair->destination_onestop_id());
      tile.mutable_stop_pairs()->RemoveLast();
      continue;
    }

    uniques.lock.lock();
    inserted = uniques.trips.insert({trip, uniques.trips.size()});
    pair->set_trip_id(inserted.first->second);
    uniques.lock.unlock();

    //block id
    std::string block_id = pair_pt.second.get<std::string>("block_id", "null");
    if(block_id != "null") {
      uniques.lock.lock();
      // uniques.block_ids.size()+1 because we can't have a block id of 0.
      // 0 means block id is not set in the transit builder.
      inserted = uniques.block_ids.insert({block_id, uniques.block_ids.size()+1});
      pair->set_block_id(inserted.first->second);
      uniques.lock.unlock();
    }

    pair->set_wheelchair_accessible(pair_pt.second.get<bool>("wheelchair_accessible", true));

    set_no_null(std::string, pair_pt.second, "trip_headsign", "null", pair->set_trip_headsign);
    pair->set_bikes_allowed(pair_pt.second.get<bool>("bikes_allowed", false));

    const auto& except_dates = pair_pt.second.get_child_optional("service_except_dates");
    if (except_dates && !except_dates->empty()) {
      for(const auto& service_except_dates : pair_pt.second.get_child("service_except_dates")) {
        auto d = DateTime::get_formatted_date(service_except_dates.second.get_value<std::string>());
        pair->add_service_except_dates(d.julian_day());
      }
    }

    const auto& added_dates = pair_pt.second.get_child_optional("service_added_dates");
    if (added_dates && !added_dates->empty()) {
      for(const auto& service_added_dates : pair_pt.second.get_child("service_added_dates")) {
        auto d = DateTime::get_formatted_date(service_added_dates.second.get_value<std::string>());
        pair->add_service_added_dates(d.julian_day());
      }
    }

    // shape data
    std::string shape_id = pair_pt.second.get<std::string>("route_stop_pattern_onestop_id", "null");
    if(shape_id != "null") {

      auto shape = shapes.find(shape_id);
      if(shape != shapes.cend()) {
        pair->set_shape_id(shape->second);
        std::string origin_dist_traveled = pair_pt.second.get<std::string>("origin_dist_traveled", "null");
        if(origin_dist_traveled != "null") {
          pair->set_origin_dist_traveled(std::stof(origin_dist_traveled));
        }

        std::string destination_dist_traveled = pair_pt.second.get<std::string>("destination_dist_traveled", "null");
        if(destination_dist_traveled != "null") {
          pair->set_destination_dist_traveled(std::stof(destination_dist_traveled));
        }
      } else {
        LOG_WARN("Shape not found for " + shape_id);
      }
    }
  }

  // no stop pairs due to new route type.
  if (tile.mutable_stop_pairs()->size() == 0)
    return false;

  return dangles;
}

void write_pbf(const Transit& tile, const boost::filesystem::path& transit_tile) {
  //check for empty stop pairs and routes.
  if(tile.stop_pairs_size() == 0 && tile.routes_size() == 0 && tile.shapes_size() == 0) {
    LOG_WARN(transit_tile.string() + " had no data and will not be stored");
    return;
  }

  //write pbf to file
  if (!boost::filesystem::exists(transit_tile.parent_path()))
    boost::filesystem::create_directories(transit_tile.parent_path());
  std::fstream stream(transit_tile.string(), std::ios::out | std::ios::trunc | std::ios::binary);
  if(!tile.SerializeToOstream(&stream))
    LOG_ERROR("Couldn't write: " + transit_tile.string() + " it would have been " + std::to_string(tile.ByteSize()));

  if (tile.routes_size() && tile.stops_size() && tile.stop_pairs_size() && tile.shapes_size()) {
    LOG_INFO(transit_tile.string() + " had " + std::to_string(tile.stops_size()) + " stops " +
             std::to_string(tile.routes_size()) + " routes " + std::to_string(tile.shapes_size()) + " shapes " +
             std::to_string(tile.stop_pairs_size()) + " stop pairs");
  } else {
    LOG_INFO(transit_tile.string() + " had " + std::to_string(tile.stop_pairs_size()) + " stop pairs");
  }
}

void fetch_tiles(const ptree& pt, std::priority_queue<weighted_tile_t>& queue, unique_transit_t& uniques,
                 std::promise<std::list<GraphId> >& promise) {
  TileHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));
  const auto& tiles = hierarchy.levels().rbegin()->second.tiles;
  std::list<GraphId> dangling;
  curler_t curler;
  auto now = time(nullptr);
  auto* utc = gmtime(&now); utc->tm_year += 1900; ++utc->tm_mon; //TODO: use timezone code?

  auto database = pt.get_optional<std::string>("mjolnir.timezone");
  // Initialize the tz DB (if it exists)
  sqlite3 *tz_db_handle = GetDBHandle(*database);
  if (!tz_db_handle)
    LOG_WARN("Time zone db " + *database + " not found.  Not saving time zone information from db.");

  //for each tile
  while(true) {
    GraphId current;
    uniques.lock.lock();
    if(queue.empty()) {
      uniques.lock.unlock();
      break;
    }
    current = queue.top().t;
    queue.pop();
    uniques.lock.unlock();
    auto filter = tiles.TileBounds(current.tileid());
    //expanding both top and bottom by distance to geodesic running through the coords
    auto min_y = filter.miny() - std::abs(filter.miny() - filter.minpt().MidPoint({filter.maxx(), filter.miny()}).second);
    auto max_y = filter.maxy() + std::abs(filter.maxy() - filter.maxpt().MidPoint({filter.minx(), filter.maxy()}).second);
    AABB2<PointLL> bbox(filter.minx(), min_y, filter.maxx(), max_y);
    ptree response;
    auto api_key = pt.get_optional<std::string>("api_key") ? "&api_key=" + pt.get<std::string>("api_key") : "";
    auto import_level = pt.get_optional<std::string>("import_level") ? "&import_level=" +
        pt.get<std::string>("import_level") : "";

    Transit tile;
    auto file_name = GraphTile::FileSuffix(current, hierarchy);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    boost::filesystem::path transit_tile = pt.get<std::string>("mjolnir.transit_dir") + '/' + file_name;

    //tiles are wrote out with .pbf or .pbf.n ext
    uint32_t ext = 0;
    std::string prefix = transit_tile.string();
    LOG_INFO("Fetching " + transit_tile.string());

    bool tile_within_one_tz = false;
    std::unordered_map<uint32_t,multi_polygon_type> tz_polys;
    if (tz_db_handle) {
      tz_polys = GetTimeZones(tz_db_handle, filter);
      if (tz_polys.size() == 1) {
        tile_within_one_tz = true;
      }
    }

    //pull out all the STOPS (you see what we did there?)
    std::unordered_map<std::string, uint64_t> stops;
    boost::optional<std::string> request = url((boost::format("/api/v1/stops?total=false&per_page=%1%&bbox=%2%,%3%,%4%,%5%")
      % pt.get<std::string>("per_page") % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str(), pt);
    request = *request + import_level;

    do {
      //grab some stuff
      response = curler(*request, "stops");
      //copy stops in, keeping map of stopid to graphid
      get_stops(tile, stops, current, response, filter, tile_within_one_tz, tz_polys);
      //please sir may i have some more?
      request = response.get_optional<std::string>("meta.next");

    } while(request && (request = *request + api_key));
    //um yeah.. we need these
    if(stops.size() == 0) {
      LOG_WARN(transit_tile.string() + " had no stops and will not be stored");
      continue;
    }

    //pull out all operator WEBSITES
    request = url((boost::format("/api/v1/operators?total=false&per_page=%1%&bbox=%2%,%3%,%4%,%5%")
      % pt.get<std::string>("per_page") % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str(), pt);
    request = *request + import_level;

    std::unordered_map<std::string, std::string> websites;
    std::unordered_map<std::string, std::string> short_names;
    do {
      //grab some stuff
      response = curler(*request, "operators");
      //save the websites to a map
      for(const auto& operators_pt : response.get_child("operators")) {
        std::string onestop_id = operators_pt.second.get<std::string>("onestop_id", "");
        std::string website = operators_pt.second.get<std::string>("website", "");
        if(!onestop_id.empty() && onestop_id != "null" && !website.empty() && website != "null")
          websites.emplace(onestop_id, website);

        std::string short_name = operators_pt.second.get<std::string>("short_name", "");
        if(!onestop_id.empty() && onestop_id != "null" && !short_name.empty() && short_name != "null")
          short_names.emplace(onestop_id, short_name);
      }
      //please sir may i have some more?
      request = response.get_optional<std::string>("meta.next");
    } while(request && (request = *request + api_key));

    //pull out all ROUTES
    request = url((boost::format("/api/v1/routes?total=false&include_geometry=false&per_page=%1%&bbox=%2%,%3%,%4%,%5%")
      % pt.get<std::string>("per_page") % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str(), pt);
    std::unordered_map<std::string, size_t> routes;
    request = *request + import_level;

    do {
      //grab some stuff
      uniques.lock.lock();
      response = curler(*request, "routes");
      uniques.lock.unlock();
      //copy routes in, keeping track of routeid to route index
      get_routes(tile, routes, websites, short_names, response);
      //please sir may i have some more?
      request = response.get_optional<std::string>("meta.next");
    } while(request && (request = *request + api_key));

    //pull out all the route_stop_patterns or shapes
    std::unordered_map<std::string, size_t> shapes;
    for(const auto& route : routes) {
      request = url((boost::format("/api/v1/route_stop_patterns?total=false&per_page=%1%&traversed_by=%2%")
              % pt.get<std::string>("per_page") % url_encode(route.first)).str(), pt);
      do {
        //grab some stuff
        response = curler(*request, "route_stop_patterns");
        //copy shapes in.
        get_stop_patterns(tile, shapes, response);
        //please sir may i have some more?
        request = response.get_optional<std::string>("meta.next");
      } while(request && (request = *request + api_key));
    }

    //pull out all SCHEDULE_STOP_PAIRS
    bool dangles = false;
    for(const auto& stop : stops) {
      request = url((boost::format("/api/v1/schedule_stop_pairs?active=true&total=false&per_page=%1%&origin_onestop_id=%2%&service_from_date=%3%-%4%-%5%")
        % pt.get<std::string>("per_page") % url_encode(stop.first) % utc->tm_year % utc->tm_mon % utc->tm_mday).str(), pt);
      request = *request + import_level;
      do {
        //grab some stuff
        response = curler(*request, "schedule_stop_pairs");
        //copy pairs in, noting if any dont have stops
        dangles = get_stop_pairs(tile, uniques, shapes, response, stops, routes) || dangles;
        //if stop pairs is large save to a path with an incremented extension
        if (tile.stop_pairs_size() >= 500000) {
          LOG_INFO("Writing " + transit_tile.string());
          write_pbf(tile, transit_tile.string());
          //reset everything
          tile.Clear();
          transit_tile = prefix + '.' + std::to_string(ext++);
        }
        //please sir may i have some more?
        request = response.get_optional<std::string>("meta.next");
      } while(request && (request = *request + api_key));
    }

    //remember who dangles
    if(dangles)
      dangling.emplace_back(current);

    //save the last tile
    if (tile.stop_pairs_size())
      write_pbf(tile, transit_tile.string());
  }

  //give back the work for later
  promise.set_value(dangling);
}

std::list<GraphId> fetch(const ptree& pt, std::priority_queue<weighted_tile_t>& tiles,
    unsigned int thread_count = std::max(static_cast<unsigned int>(1), std::thread::hardware_concurrency())) {
  LOG_INFO("Fetching " + std::to_string(tiles.size()) + " transit tiles with " + std::to_string(thread_count) + " threads...");

  //schedule some work
  unique_transit_t uniques;
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);
  std::vector<std::promise<std::list<GraphId> > > promises(threads.size());
  for (size_t i = 0; i < threads.size(); ++i)
    threads[i].reset(new std::thread(fetch_tiles, std::cref(pt), std::ref(tiles), std::ref(uniques), std::ref(promises[i])));

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

Transit read_pbf(const std::string& file_name, std::mutex& lock) {
  lock.lock();
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if(!file) {
    throw std::runtime_error("Couldn't load " + file_name);
    lock.unlock();
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  lock.unlock();
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  auto limit = std::max(static_cast<size_t>(1), buffer.size() * 2);
  cs.SetTotalBytesLimit(limit, limit);
  Transit transit;
  if(!transit.ParseFromCodedStream(&cs))
    throw std::runtime_error("Couldn't load " + file_name);
  return transit;
}

struct dist_sort_t {
  PointLL center;
  Tiles<PointLL> grid;
  dist_sort_t(const GraphId& center, const Tiles<PointLL>& grid):grid(grid) {
    this->center = grid.TileBounds(center.tileid()).Center();
  }
  bool operator()(const GraphId& a, const GraphId& b) const {
    auto a_dist = center.Distance(grid.TileBounds(a.tileid()).Center());
    auto b_dist = center.Distance(grid.TileBounds(b.tileid()).Center());
    if(a_dist == b_dist)
      return a.tileid() < b.tileid();
    return a_dist < b_dist;
  }
};

void stitch_tiles(const ptree& pt, const std::unordered_set<GraphId>& all_tiles, std::list<GraphId>& tiles, std::mutex& lock) {
  TileHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));
  auto grid = hierarchy.levels().rbegin()->second.tiles;
  auto tile_name = [&hierarchy, &pt](const GraphId& id){
    auto file_name = GraphTile::FileSuffix(id, hierarchy);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    return pt.get<std::string>("mjolnir.transit_dir") + '/' + file_name;
  };

  //for each tile
  while(true) {
    GraphId current;
    lock.lock();
    if(tiles.empty()) {
      lock.unlock();
      break;
    }
    current = tiles.front();
    tiles.pop_front();
    lock.unlock();

    auto prefix = tile_name(current);
    auto file_name = prefix;
    int ext = 0;

    do {

      //open tile make a hash of missing stop to invalid graphid
      auto tile = read_pbf(file_name, lock);
      std::unordered_map<std::string, GraphId> needed;
      for(const auto& stop_pair : tile.stop_pairs()) {
        if(!stop_pair.has_origin_graphid())
          needed.emplace(stop_pair.origin_onestop_id(), GraphId{});
        if(!stop_pair.has_destination_graphid())
          needed.emplace(stop_pair.destination_onestop_id(), GraphId{});
      }

      //do while we have more to find and arent sick of searching
      std::set<GraphId, dist_sort_t> unchecked(all_tiles.cbegin(), all_tiles.cend(), dist_sort_t(current, grid));
      size_t found = 0;
      while(found < needed.size() && unchecked.size()) {
        //crack it open to see if it has what we want
        auto neighbor_id = *unchecked.cbegin();
        unchecked.erase(unchecked.begin());
        if(neighbor_id != current) {
          auto neighbor_file_name = tile_name(neighbor_id);
          auto neighbor = read_pbf(neighbor_file_name, lock);
          for(const auto& stop : neighbor.stops()) {
            auto stop_itr = needed.find(stop.onestop_id());
            if(stop_itr != needed.cend()) {
              stop_itr->second.value = stop.graphid();
              ++found;
            }
          }
        }
      }

      //get the ids fixed up and write pbf to file
      std::unordered_set<std::string> not_found;
      for(auto& stop_pair : *tile.mutable_stop_pairs()) {
        if(!stop_pair.has_origin_graphid()) {
          auto found = needed.find(stop_pair.origin_onestop_id())->second;
          if(found.Is_Valid())
            stop_pair.set_origin_graphid(found);
          else if(not_found.find(stop_pair.origin_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.origin_onestop_id());
            not_found.emplace(stop_pair.origin_onestop_id());
          }
          //else{ TODO: we could delete this stop pair }
        }
        if(!stop_pair.has_destination_graphid()) {
          auto found = needed.find(stop_pair.destination_onestop_id())->second;
          if(found.Is_Valid())
            stop_pair.set_destination_graphid(found);
          else if(not_found.find(stop_pair.destination_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.destination_onestop_id());
            not_found.emplace(stop_pair.destination_onestop_id());
          }
          //else{ TODO: we could delete this stop pair }
        }
      }
      lock.lock();
      std::fstream stream(file_name, std::ios::out | std::ios::trunc | std::ios::binary);
      tile.SerializeToOstream(&stream);
      lock.unlock();
      LOG_INFO(file_name + " stitched " + std::to_string(found) + " of " + std::to_string(needed.size()) + " stops");

      file_name = prefix + "." + std::to_string(ext++);
    }while(boost::filesystem::exists(file_name));
  }
}

void stitch(const ptree& pt, const std::unordered_set<GraphId>& all_tiles, std::list<GraphId>& dangling_tiles,
    unsigned int thread_count = std::max(static_cast<unsigned int>(1), std::thread::hardware_concurrency())) {
  LOG_INFO("Stitching " + std::to_string(dangling_tiles.size()) + " transit tiles with " + std::to_string(thread_count) + " threads...");

  //figure out where the work should go
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);
  std::mutex lock;

  //make let them rip
  for (size_t i = 0; i < threads.size(); ++i)
    threads[i].reset(new std::thread(stitch_tiles, std::cref(pt), std::cref(all_tiles), std::ref(dangling_tiles), std::ref(lock)));

  //wait for them to finish
  for (auto& thread : threads)
    thread->join();

  LOG_INFO("Finished");
}

// Get scheduled departures for a stop
std::unordered_multimap<GraphId, Departure> ProcessStopPairs(
    GraphTileBuilder& transit_tilebuilder,
    const uint32_t tile_date,
    const Transit& transit,
    std::unordered_map<GraphId, uint16_t>& stop_access,
    const std::string& file,
    const GraphId& tile_id, std::mutex& lock,
    builder_stats& stats) {
  // Check if there are no schedule stop pairs in this tile
  std::unordered_multimap<GraphId, Departure> departures;

  // Map of unique schedules (validity) in this tile
  uint32_t schedule_index = 0;
  std::map<TransitSchedule, uint32_t> schedules;

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
            spp = read_pbf(fname, lock);
        }

        if (spp.stop_pairs_size() == 0) {
          if (transit.stops_size() > 0) {
            LOG_ERROR("Tile " + fname +
                      " has 0 schedule stop pairs but has " +
                      std::to_string(transit.stops_size()) + " stops");
          }
          departures.clear();
          return departures;
        }

        // Iterate through the stop pairs in this tile and form Valhalla departure
        // records
        for (const auto& sp : spp.stop_pairs()) {
          // We do not know in this step if the end node is in a valid (non-empty)
          // Valhalla tile. So just add the stop pair and we will address this later

          // Use transit PBF graph Ids internally until adding to the graph tiles
          // TODO - wheelchair accessible, shape information
          Departure dep;
          dep.orig_pbf_graphid = GraphId(sp.origin_graphid());
          dep.dest_pbf_graphid = GraphId(sp.destination_graphid());
          dep.route = sp.route_index();
          dep.trip = sp.trip_id();

          // if we have shape data then set everything else shapeid = 0;
          if (sp.has_shape_id() && sp.has_destination_dist_traveled() && sp.has_origin_dist_traveled()) {
            dep.shapeid = sp.shape_id();
            dep.orig_dist_traveled = sp.origin_dist_traveled();
            dep.dest_dist_traveled = sp.destination_dist_traveled();
          } else dep.shapeid = 0;

          dep.blockid = sp.has_block_id() ? sp.block_id() : 0;
          dep.dep_time = sp.origin_departure_time();
          dep.elapsed_time = sp.destination_arrival_time() - dep.dep_time;

          dep.frequency_end_time = sp.has_frequency_end_time() ? sp.frequency_end_time() : 0;
          dep.frequency = sp.has_frequency_headway_seconds() ? sp.frequency_headway_seconds() : 0;

          if (!sp.bikes_allowed()) {
            stop_access[dep.orig_pbf_graphid] |= kBicycleAccess;
            stop_access[dep.dest_pbf_graphid] |= kBicycleAccess;
          }

          if (!sp.wheelchair_accessible()) {
            stop_access[dep.orig_pbf_graphid] |= kWheelchairAccess;
            stop_access[dep.dest_pbf_graphid] |= kWheelchairAccess;
          }

          dep.bicycle_accessible = sp.bikes_allowed();
          dep.wheelchair_accessible = sp.wheelchair_accessible();

          // Compute days of week mask
          uint32_t dow_mask = kDOWNone;
          for (uint32_t x = 0; x < sp.service_days_of_week_size(); x++) {
            bool dow = sp.service_days_of_week(x);
            if (dow) {
              switch (x) {
                case 0:
                  dow_mask |= kMonday;
                  break;
                case 1:
                  dow_mask |= kTuesday;
                  break;
                case 2:
                  dow_mask |= kWednesday;
                  break;
                case 3:
                  dow_mask |= kThursday;
                  break;
                case 4:
                  dow_mask |= kFriday;
                  break;
                case 5:
                  dow_mask |= kSaturday;
                  break;
                case 6:
                  dow_mask |= kSunday;
                  break;
              }
            }
          }

          // Compute the valid days
          // set the bits based on the dow.
          boost::gregorian::date start_date(boost::gregorian::gregorian_calendar::from_julian_day_number(sp.service_start_date()));
          boost::gregorian::date end_date(boost::gregorian::gregorian_calendar::from_julian_day_number(sp.service_end_date()));
          uint64_t days = DateTime::get_service_days(start_date, end_date, tile_date, dow_mask);

          // if this is a service addition for one day, delete the dow_mask.
          if (sp.service_start_date() == sp.service_end_date())
            dow_mask = kDOWNone;

          // if dep.days == 0 then feed either starts after the end_date or tile_header_date > end_date
          if (days == 0 && !sp.service_added_dates_size()) {
            LOG_DEBUG("Feed rejected!  Start date: " + to_iso_extended_string(start_date) + " End date: " + to_iso_extended_string(end_date));
            continue;
          }

          dep.headsign_offset = transit_tilebuilder.AddName(sp.trip_headsign());
          uint32_t end_day = (DateTime::days_from_pivot_date(end_date) - tile_date);

          if (end_day > kScheduleEndDay)
            end_day = kScheduleEndDay;

          //if subtractions are between start and end date then turn off bit.
          for (const auto& x : sp.service_except_dates()) {
            boost::gregorian::date d(boost::gregorian::gregorian_calendar::from_julian_day_number(x));
            days = DateTime::remove_service_day(days, end_date, tile_date, d);
          }

          //if additions are between start and end date then turn on bit.
          for (const auto& x : sp.service_added_dates()) {
            boost::gregorian::date d(boost::gregorian::gregorian_calendar::from_julian_day_number(x));
            days = DateTime::add_service_day(days, end_date, tile_date, d);
          }

          TransitSchedule sched(days, dow_mask, end_day);
          auto sched_itr = schedules.find(sched);
          if (sched_itr == schedules.end()) {
            // Not in the map - add a new transit schedule to the tile
            transit_tilebuilder.AddTransitSchedule(sched);

            // Add to the map and increment the index
            schedules[sched] = schedule_index;
            dep.schedule_index = schedule_index;
            schedule_index++;
          } else {
            dep.schedule_index = sched_itr->second;
          }

          //is this passed midnight?
          //adjust the time if it is after midnight.
          //create a departure for before midnight and one after
          uint32_t origin_seconds = sp.origin_departure_time();
          if (origin_seconds >= kSecondsPerDay) {

            // Add the current dep to the departures list
            // and then update it with new dep time.  This
            // dep will be used when the start time is after
            // midnight.
            stats.midnight_dep_count++;
            departures.emplace(dep.orig_pbf_graphid, dep);
            while (origin_seconds >= kSecondsPerDay)
              origin_seconds -= kSecondsPerDay;

            dep.dep_time = origin_seconds;
            dep.frequency_end_time = 0;
            dep.frequency = 0;
            if (sp.has_frequency_end_time() && sp.has_frequency_headway_seconds()) {
              uint32_t frequency_end_time = sp.frequency_end_time();
              //adjust the end time if it is after midnight.
              while (frequency_end_time >= kSecondsPerDay)
                frequency_end_time -= kSecondsPerDay;

              dep.frequency_end_time = frequency_end_time;
              dep.frequency = sp.frequency_headway_seconds();
            }
          }
          // Add to the departures list
          departures.emplace(dep.orig_pbf_graphid, std::move(dep));
          stats.dep_count++;
        }
      }
    }
  }
  return departures;
}

// Add routes to the tile. Return a vector of route types.
std::vector<uint32_t> AddRoutes(const Transit& transit,
                   GraphTileBuilder& tilebuilder) {
  // Route types vs. index
  std::vector<uint32_t> route_types;

  for (uint32_t i = 0; i < transit.routes_size(); i++) {
    const Transit_Route& r = transit.routes(i);

      // These should all be correctly set in the fetcher as it tosses types that we
      // don't support.  However, let's report an error if we encounter one.
      TransitType route_type = static_cast<TransitType>(r.vehicle_type());
      switch (route_type) {
        case TransitType::kTram:        // Tram, streetcar, lightrail
        case TransitType::kMetro:      // Subway, metro
        case TransitType::kRail:        // Rail
        case TransitType::kBus:         // Bus
        case TransitType::kFerry:       // Ferry
        case TransitType::kCableCar:    // Cable car
        case TransitType::kGondola:     // Gondola (suspended cable car)
        case TransitType::kFunicular:   // Funicular (steep incline)
          break;
        default:
          // Log an unsupported vehicle type, set to bus for now
          LOG_ERROR("Unsupported vehicle type!");
          route_type = TransitType::kBus;
          break;
      }

      TransitRoute route(route_type,
                         tilebuilder.AddName(r.onestop_id()),
                         tilebuilder.AddName(r.operated_by_onestop_id()),
                         tilebuilder.AddName(r.operated_by_name()),
                         tilebuilder.AddName(r.operated_by_website()),
                         r.route_color(),
                         r.route_text_color(),
                         tilebuilder.AddName(r.name()),
                         tilebuilder.AddName(r.route_long_name()),
                         tilebuilder.AddName(r.route_desc()));
      LOG_DEBUG("Route idx = " + std::to_string(i) + ": " + r.name() + ","
                     + r.route_long_name());
      tilebuilder.AddTransitRoute(route);

      // Route type - need this to store in edge.
      route_types.push_back(r.vehicle_type());
  }
  return route_types;
}

// Get Use given the transit route type
// TODO - add separate Use for different types - when we do this change
// the directed edge IsTransit method
Use GetTransitUse(const uint32_t rt) {
  switch (static_cast<TransitType>(rt)) {
    default:
  case TransitType::kTram:        // Tram, streetcar, lightrail
  case TransitType::kMetro:      // Subway, metro
  case TransitType::kRail:        // Rail
  case TransitType::kCableCar:    // Cable car
  case TransitType::kGondola:     // Gondola (suspended cable car)
  case TransitType::kFunicular:   // Funicular (steep incline)
    return Use::kRail;
  case TransitType::kBus:         // Bus
    return Use::kBus;
  case TransitType::kFerry:       // Ferry (boat)
    return Use::kRail;            // TODO - add ferry use
  }
}

std::list<PointLL> GetShape(const PointLL& stop_ll, const PointLL& endstop_ll, uint32_t shapeid,
                            const float orig_dist_traveled, const float dest_dist_traveled,
                            const std::vector<PointLL>& trip_shape, const std::vector<float>& distances,
                            const std::string origin_id, const std::string dest_id) {

  std::list<PointLL> shape;
  if (shapeid != 0 && trip_shape.size() && stop_ll != endstop_ll &&
      orig_dist_traveled < dest_dist_traveled) {

    float distance = 0.0f, d_from_p0_to_x = 0.0f;

    // point x - we are trying to find it on the line segment between p0 and p1
    PointLL x;
    // find out where orig_dist_traveled should be in the list.
    auto lower_bound = std::lower_bound(distances.cbegin(), distances.cend(), orig_dist_traveled);
    // find out where dest_dist_traveled should be in the list.
    auto upper_bound = std::upper_bound(distances.cbegin(), distances.cend(), dest_dist_traveled);
    float prev_distance = *(lower_bound);

    // distance calculations can be off just a bit (i.e., 9372.224609 < 9372.500000) so set it to
    // the last element.
    if (distances.back() < dest_dist_traveled)
      upper_bound = distances.cend()-1;

    // lower_bound returns an iterator pointing to the first element which does not compare less than the dist_traveled;
    // therefore, we need to back up one if it does not equal the lower_bound value.  For example, we could be starting
    // at the beginning of the points list
    if (orig_dist_traveled != (*lower_bound))
      prev_distance = *(--lower_bound);

    // loop through the points.
    for (auto itr = lower_bound; itr != upper_bound; ++itr) {

      /*    |
       *    |
       *    p0
       *    | }--d_from_p0_to_x (distance from p0 to x)
       *    x -- point we are trying to find on the segment (orig_dist_traveled or dest_dist_traveled on this segment)
       *    |
       *    |
       *    |
       *    |
       *    p1
       *    |
       *    |
       */

      // index into our vector of points
      uint32_t index = (itr - distances.cbegin());
      PointLL p0 = trip_shape[index];
      PointLL p1 = trip_shape[index + 1];

      // this is our distance that is beyond x.
      distance = *(itr+1);

      // find point x using the orig_dist_traveled - this is our first point added to shape
      if (itr == lower_bound) {
        if (orig_dist_traveled == *itr) // just add p0
          shape.push_back(p0);
        else {
          // distance from p0 to x using the orig_dist_traveled
          d_from_p0_to_x = (orig_dist_traveled - prev_distance) / (distance - prev_distance);
          x = p0 + (p1 - p0) * d_from_p0_to_x;
          shape.push_back(x);
        }
      }

      // find point x using the dest_dist_traveled - this is our last point added to the shape
      if ((itr+1) == upper_bound) {
        if (dest_dist_traveled == *itr) { // just add p0
          if (shape.back() != p0) //avoid dups
            shape.push_back(p0);
        } else {
          // distance from p0 to x using the dest_dist_traveled
          d_from_p0_to_x = (dest_dist_traveled - prev_distance) / (distance - prev_distance);
          x = p0 + (p1 - p0) * d_from_p0_to_x;

          if (shape.back() != x) //avoid dups
            shape.push_back(x);
          // we are done p1 is too far away
        }
        break;
      }
      // add all the midpoints.
      shape.push_back(p1);

      prev_distance = distance;
    }
  // else no shape exists.
  } else {
    shape.push_back(stop_ll);
    shape.push_back(endstop_ll);
  }

  if (shape.size() == 0) {
    LOG_ERROR("Invalid shape from " + origin_id + " to " + dest_id);
    shape.push_back(stop_ll);
    shape.push_back(endstop_ll);
  }

  return shape;
}

// Converts a stop's pbf graph Id to a Valhalla graph Id by adding the
// tile's node count. Returns an Invalid GraphId if the tile is not found
// in the list of Valhalla tiles
GraphId GetGraphId(const GraphId& nodeid,
                   const std::unordered_set<GraphId>& all_tiles) {
  auto t = all_tiles.find(nodeid.Tile_Base());
  if (t == all_tiles.end()) {
    return GraphId();  // Invalid graph Id
  } else {
    return { nodeid.tileid(), nodeid.level()+1, nodeid.id()};
  }
}

void AddToGraph(GraphTileBuilder& tilebuilder_transit,
                const TileHierarchy& hierarchy_transit,
                const GraphId& tileid,
                const std::string& tile,
                const std::string& transit_dir,
                std::mutex& lock,
                const std::unordered_set<GraphId>& all_tiles,
                const std::map<GraphId, StopEdges>& stop_edge_map,
                const std::unordered_map<GraphId, uint16_t>& stop_access,
                const std::unordered_map<uint32_t, Shape> shape_data,
                const std::vector<float> distances,
                const std::vector<uint32_t>& route_types,
                std::vector<OneStopTest>& onestoptests,
                uint32_t& no_dir_edge_count) {
  auto t1 = std::chrono::high_resolution_clock::now();

  // Get Transit PBF data for this tile
  Transit transit = read_pbf(tile, lock);

  tilebuilder_transit.AddAdmin("None","None","","");

  // Iterate through the stops and their edges
  uint32_t nadded = 0;
  uint32_t transitedges = 0;
  for (const auto& stop_edges : stop_edge_map) {
    // Get the stop information
    GraphId stopid = stop_edges.second.origin_pbf_graphid;
    uint32_t stop_index = stopid.id();
    const Transit_Stop& stop = transit.stops(stop_index);
    std::string origin_id = stop.onestop_id();
    if (GraphId(stop.graphid()) != stopid) {
      LOG_ERROR("Stop key not equal!");
    }

    LOG_DEBUG("Transit Stop: " + stop.name() + " stop index= " +
              std::to_string(stop_index));

    // Get the Valhalla graphId of the origin node (transit stop)
    GraphId origin_node = GetGraphId(stopid, all_tiles);

    // Build the node info. Use generic transit stop type
    uint32_t n_access = (kPedestrianAccess | kWheelchairAccess | kBicycleAccess);

    auto s_access = stop_access.find(stopid);
    if (s_access != stop_access.end()) {
      n_access &= ~s_access->second;
    }
    // TODO - parent/child stop types
    PointLL stopll = { stop.lon(), stop.lat() };
    NodeInfo node(stopll, RoadClass::kServiceOther, n_access,
                        NodeType::kMultiUseTransitStop, false);
    node.set_mode_change(true);
    node.set_stop_index(stop_index);
    node.set_edge_index(tilebuilder_transit.directededges().size());
    node.set_timezone(stop.timezone());
    node.set_connecting_wayid(stop.osm_way_id());

 /** TODO - future when we get egress, station, platform hierarchy
    // Add any intra-station connections - these are always in the same tile?
    for (const auto& endstopid : stop_edges.second.intrastation) {

      const Stop& endstop = stops[endstopid.id()];
      if (endstopid != endstop.pbf_graphid) {
        LOG_ERROR("End stop key not equal");
      }

      GraphId endnode = GetGraphId(endstop.pbf_graphid);
      if (!endnode.Is_Valid()) {
        continue;
      }

      DirectedEdge directededge;
      directededge.set_endnode(endstop.pbf_graphid);

      // Make sure length is non-zero
      float length = std::max(1.0f, stop.ll().Distance(endstop.ll()));
      directededge.set_length(length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder.directededges().size() - node.edge_index());
      directededge.set_forwardaccess(kPedestrianAccess);  // TODO - bikes?
      directededge.set_reverseaccess(kPedestrianAccess);  // TODO - bikes?

      LOG_DEBUG("Add parent/child directededge - endnode stop id = " +
               std::to_string(endstop.key) + " GraphId: " +
               std::to_string(endstop.graphid.tileid()) + "," +
               std::to_string(endstop.graphid.id()));

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      std::vector<std::string> names;
      std::list<PointLL> shape = { stop.ll(), endstop.ll() };

      // TODO - these need to be valhalla graph Ids
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, stop.pbf_graphid,
                     endstop.pbf_graphid, 0, shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      tilebuilder.directededges().emplace_back(std::move(directededge));
    }
**/

    // Add transit lines
    // level 3
    for (const auto& transitedge : stop_edges.second.lines) {
      // Get the end node. Skip this directed edge if the Valhalla tile is
      // not valid (or empty)
      GraphId endnode = GetGraphId(transitedge.dest_pbf_graphid, all_tiles);
      if (!endnode.Is_Valid()) {
        continue;
      }

      // Find the lat,lng of the end stop
      PointLL endll;
      std::string endstopname;
      GraphId end_stop_graphid = transitedge.dest_pbf_graphid;
      std::string dest_id;

      if (end_stop_graphid.Tile_Base() == tileid) {
        // End stop is in the same pbf transit tile
        const Transit_Stop& endstop = transit.stops(end_stop_graphid.id());
        endstopname = endstop.name();
        endll = {endstop.lon(), endstop.lat()};
        dest_id = endstop.onestop_id();

      } else {
        // Get Transit PBF data for this tile
        // Get transit pbf tile
        std::string file_name = GraphTile::FileSuffix(GraphId(end_stop_graphid.tileid(), end_stop_graphid.level(),0), hierarchy_transit);
        boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
        file_name += ".pbf";
        const std::string file = transit_dir + '/' + file_name;
        Transit endtransit = read_pbf(file, lock);
        const Transit_Stop& endstop = endtransit.stops(end_stop_graphid.id());
        endstopname = endstop.name();
        endll = {endstop.lon(), endstop.lat()};
        dest_id = endstop.onestop_id();
      }

      // Add the directed edge
      DirectedEdge directededge;
      directededge.set_endnode(endnode);
      directededge.set_length(stopll.Distance(endll));
      Use use = GetTransitUse(route_types[transitedge.routeid]);
      directededge.set_use(use);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder_transit.directededges().size() - node.edge_index());
      directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
      directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
      directededge.set_lineid(transitedge.lineid);

      LOG_DEBUG("Add transit directededge - lineId = " + std::to_string(transitedge.lineid) +
         " Route Key = " + std::to_string(transitedge.routeid) +
         " EndStop " + endstopname);

      // Add edge info to the tile and set the offset in the directed edge
      // Leave the name empty. Use the trip Id to look up the route Id and
      // route within TripPathBuilder.
      bool added = false;
      std::vector<std::string> names;

      std::vector<PointLL> points;
      std::vector<float> distance;
      // get the indexes and vector of points for this shape id
      const auto& found = shape_data.find(transitedge.shapeid);
      if (transitedge.shapeid != 0 && found != shape_data.cend()) {
        const auto& shape_d = found->second;
        points = shape_d.shape;
        // copy only the distances that we care about.
        std::copy((distances.cbegin()+shape_d.begins),
                  (distances.cbegin()+shape_d.ends),back_inserter(distance));
      }
      else if (transitedge.shapeid != 0)
        LOG_WARN("Shape Id not found: " + std::to_string(transitedge.shapeid));

      // TODO - if we separate transit edges based on more than just routeid
      // we will need to do something to differentiate edges (maybe use
      // lineid) so the shape doesn't get messed up.
      auto shape = GetShape(stopll, endll, transitedge.shapeid, transitedge.orig_dist_traveled,
                            transitedge.dest_dist_traveled, points, distance, origin_id, dest_id);

      uint32_t edge_info_offset = tilebuilder_transit.AddEdgeInfo(transitedge.routeid,
           origin_node, endnode, 0, shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      tilebuilder_transit.directededges().emplace_back(std::move(directededge));
      transitedges++;
    }

    // Get the directed edge count, log an error if no directed edges are added
    uint32_t edge_count = tilebuilder_transit.directededges().size() - node.edge_index();
    if (edge_count == 0) {
      // Set the edge index to 0
      node.set_edge_index(0);
      no_dir_edge_count++;
    }

    // Add the node
    node.set_edge_count(edge_count);
    tilebuilder_transit.nodes().emplace_back(std::move(node));
  }

  // Log the number of added nodes and edges
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                  t2 - t1).count();
  LOG_INFO("Tile " + std::to_string(tileid.tileid())
          + ": added " + std::to_string(transitedges) + " transit edges, and "
          + std::to_string(tilebuilder_transit.nodes().size()) + " nodes. time = "
          + std::to_string(msecs) + " ms");
}

// We make sure to lock on reading and writing since tiles are now being
// written. Also lock on queue access since shared by different threads.
void build_tiles(const boost::property_tree::ptree& pt, std::mutex& lock,
                 const std::unordered_set<GraphId>& all_tiles,
                 std::unordered_set<GraphId>::const_iterator tile_start,
                 std::unordered_set<GraphId>::const_iterator tile_end,
                 std::vector<OneStopTest>& onestoptests,
                 std::promise<builder_stats>& results) {

  builder_stats stats;
  stats.no_dir_edge_count = 0;
  stats.dep_count = 0;
  stats.midnight_dep_count = 0;

  GraphReader reader_transit_level(pt);
  const TileHierarchy& hierarchy_transit_level = reader_transit_level.GetTileHierarchy();

  // Iterate through the tiles in the queue and find any that include stops
  for(; tile_start != tile_end; ++tile_start) {
    // Get the next tile Id from the queue and get a tile builder
    if(reader_transit_level.OverCommitted())
      reader_transit_level.Clear();
    GraphId tile_id = tile_start->Tile_Base();

    // Get transit pbf tile
    const std::string transit_dir = pt.get<std::string>("transit_dir");
    std::string file_name = GraphTile::FileSuffix(GraphId(tile_id.tileid(), tile_id.level(),0), hierarchy_transit_level);
    boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
    file_name += ".pbf";
    const std::string file = transit_dir + '/' + file_name;

    // Make sure it exists
    if (!boost::filesystem::exists(file)) {
      LOG_ERROR("File not found.  " + file);
      return;
    }

    Transit transit = read_pbf(file, lock);
    // Get Valhalla tile - get a read only instance for reference and
    // a writeable instance (deserialize it so we can add to it)
    lock.lock();

    GraphId transit_tile_id = GraphId(tile_id.tileid(), tile_id.level()+1, tile_id.id());
    const GraphTile* transit_tile = reader_transit_level.GetGraphTile(transit_tile_id);
    GraphTileBuilder tilebuilder_transit(hierarchy_transit_level, transit_tile_id, false);

    auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
    uint32_t tile_creation_date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));
    tilebuilder_transit.AddTileCreationDate(tile_creation_date);

    lock.unlock();

    std::unordered_map<GraphId, uint16_t> stop_access;
    std::unordered_multimap<GraphId, GraphId> children;
    for (uint32_t i = 0; i < transit.stops_size(); i++) {
      const Transit_Stop& stop = transit.stops(i);

      if (!stop.wheelchair_boarding())
        stop_access[GraphId(stop.graphid())] |= kWheelchairAccess;

      // Store stop information in TransitStops
      tilebuilder_transit.AddTransitStop( { tilebuilder_transit.AddName(stop.onestop_id()),
                                            tilebuilder_transit.AddName(stop.name()) } );

      /** TODO - parent/child relationships
      if (stop.type == 0 && stop.parent.Is_Valid()) {
        children.emplace(stop.parent, stop.pbf_graphid);
      }       **/
    }

    //Get all the shapes for this tile and calculate the distances
    std::unordered_map<uint32_t, Shape> shapes;
    std::vector<float> distances;
    for (uint32_t i = 0; i < transit.shapes_size(); i++) {
      const Transit_Shape& shape = transit.shapes(i);
      const std::vector<PointLL> trip_shape = decode7<std::vector<PointLL> >(shape.encoded_shape());

      float distance = 0.0f;
      Shape shape_data;
      //first is always 0.0f.
      distances.push_back(distance);
      shape_data.begins = distances.size()-1;

      // loop through the points getting the distances.
      for (size_t index = 0; index < trip_shape.size() - 1; ++index) {
        PointLL p0 = trip_shape[index];
        PointLL p1 = trip_shape[index + 1];
        distance += p0.Distance(p1);
        distances.push_back(distance);
      }
      //must be distances.size for the end index as we use std::copy later on and want
      //to include the last element in the vector we wish to copy.
      shape_data.ends = distances.size();
      shape_data.shape = trip_shape;
      //shape id --> begin and end indexes in the distance vector and vector of points.
      shapes[shape.shape_id()] = shape_data;
    }

    // Get all scheduled departures from the stops within this tile.
    std::map<GraphId, StopEdges> stop_edge_map;
    uint32_t unique_lineid = 1;
    std::vector<TransitDeparture> transit_departures;

    // Create a map of stop key to index in the stop vector

    // Process schedule stop pairs (departures)
    std::unordered_multimap<GraphId, Departure> departures =
                ProcessStopPairs(tilebuilder_transit,tile_creation_date,
                                 transit, stop_access, file, tile_id, lock,
                                 stats);

    // Form departures and egress/station/platform hierarchy
    for (uint32_t i = 0; i < transit.stops_size(); i++) {
      const Transit_Stop& stop = transit.stops(i);
      GraphId stop_pbf_graphid = GraphId(stop.graphid());
      StopEdges stopedges;
      stopedges.origin_pbf_graphid = stop_pbf_graphid;

      /** TODO - Identify any parent-child edge connections
      if (stop.type == 1) {
        // Station - identify any children.
        auto range = children.equal_range(stop.pbf_graphid);
        for(auto kv = range.first; kv != range.second; ++kv)
          stopedges.intrastation.push_back(kv->second);
      } else if (stop.parent != 0) {
        stopedges.intrastation.push_back(stop.parent);
      } **/

      // TODO - perhaps replace this code with use of headsign below
      // to solve problem of a trip that doesn't go the whole way to
      // the end of the route line
      std::map<std::pair<uint32_t, GraphId>, uint32_t> unique_transit_edges;
      auto range = departures.equal_range(stop_pbf_graphid);
       for(auto key = range.first; key != range.second; ++key) {
         Departure dep = key->second;

         // Identify unique route and arrival stop pairs - associate to a
         // unique line Id stored in the directed edge.
         uint32_t lineid;
         auto m = unique_transit_edges.find({dep.route, dep.dest_pbf_graphid});
         if (m == unique_transit_edges.end()) {
           // Add to the map and update the line id
           lineid = unique_lineid;
           unique_transit_edges[{dep.route, dep.dest_pbf_graphid}] = unique_lineid;
           unique_lineid++;
           stopedges.lines.emplace_back(TransitLine{lineid, dep.route,
                 dep.dest_pbf_graphid, dep.shapeid, dep.orig_dist_traveled,
                 dep.dest_dist_traveled});
         } else {
           lineid = m->second;
         }

         try {
           if (dep.frequency == 0) {
             // Form transit departures -- fixed departure time
             TransitDeparture td(lineid, dep.trip, dep.route,
                                 dep.blockid, dep.headsign_offset, dep.dep_time,
                                 dep.elapsed_time, dep.schedule_index,
                                 dep.wheelchair_accessible, dep.bicycle_accessible);
             tilebuilder_transit.AddTransitDeparture(std::move(td));
           } else {

             // Form transit departures -- frequency departure time
             TransitDeparture td(lineid, dep.trip, dep.route,
                                 dep.blockid, dep.headsign_offset,
                                 dep.dep_time, dep.frequency_end_time,
                                 dep.frequency, dep.elapsed_time, dep.schedule_index,
                                 dep.wheelchair_accessible, dep.bicycle_accessible);
             tilebuilder_transit.AddTransitDeparture(std::move(td));
           }
         } catch(const std::exception& e) {
           LOG_ERROR(e.what());
         }
      }

      // TODO Get any transfers from this stop (no transfers currently
      // available from Transitland)
      // AddTransfers(tilebuilder);

      // Add to stop edge map - track edges that need to be added. This is
      // sorted by graph Id so the stop nodes are added in proper order
      stop_edge_map.insert({stop_pbf_graphid, stopedges});
    }

    // Add routes to the tile. Get vector of route types.
    std::vector<uint32_t> route_types = AddRoutes(transit, tilebuilder_transit);

    // Add nodes, directededges, and edgeinfo
    AddToGraph(tilebuilder_transit, hierarchy_transit_level, tile_id, file, transit_dir,
               lock, all_tiles, stop_edge_map, stop_access, shapes, distances,
               route_types, onestoptests, stats.no_dir_edge_count);

    LOG_INFO("Tile " + std::to_string(tile_id.tileid()) + ": added " +
             std::to_string(transit.stops_size()) + " stops, " +
             std::to_string(transit.shapes_size()) + " shapes, " +
             std::to_string(route_types.size()) + " routes, and " +
             std::to_string(departures.size()) + " departures");

    // Write the new file
    lock.lock();
    tilebuilder_transit.StoreTileData();
    lock.unlock();
  }

  // Send back the statistics
  results.set_value(stats);
}

void build(const ptree& pt, const std::unordered_set<GraphId>& all_tiles,
           std::vector<OneStopTest>& onestoptests,
           unsigned int thread_count = std::max(static_cast<unsigned int>(1), std::thread::hardware_concurrency())) {

  LOG_INFO("Building transit network.");

  auto t1 = std::chrono::high_resolution_clock::now();
  if (!all_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be added.");
    return;
  }

  // TODO - intermediate pass to find any connections that cross into different
  // tile than the stop

  // Second pass - for all tiles with transit stops get all transit information
  // and populate tiles

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats> > results;

  // Start the threads, divvy up the work
  LOG_INFO("Adding " + std::to_string(all_tiles.size()) + " transit tiles to the transit graph...");
  size_t floor = all_tiles.size() / threads.size();
  size_t at_ceiling = all_tiles.size() - (threads.size() * floor);
  std::unordered_set<GraphId>::const_iterator tile_start, tile_end = all_tiles.begin();

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i].reset(
        new std::thread(build_tiles, std::cref(pt.get_child("mjolnir")),
                        std::ref(lock), std::cref(all_tiles), tile_start, tile_end,
                        std::ref(onestoptests), std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }


  // Check all of the outcomes, to see about maximum density (km/km2)
  builder_stats stats{};
  uint32_t total_no_dir_edge_count = 0;
  uint32_t total_dep_count = 0;
  uint32_t total_midnight_dep_count = 0;

  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
      total_no_dir_edge_count += stats.no_dir_edge_count;
      total_dep_count += stats.dep_count;
      total_midnight_dep_count += stats.midnight_dep_count;
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }

  if (total_no_dir_edge_count)
    LOG_ERROR("There were " + std::to_string(total_no_dir_edge_count) + " nodes with no directed edges");

  if (total_dep_count) {
    float percent = static_cast<float>(total_midnight_dep_count) / static_cast<float>(total_dep_count);
    percent *= 100;

    LOG_INFO("There were " + std::to_string(total_dep_count) + " departures and " +
              std::to_string(total_midnight_dep_count) + " midnight departures were added: " +
              std::to_string(percent) + "% increase.");
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished building transit network - took " + std::to_string(secs) + " secs");
}

int main(int argc, char** argv) {
  if(argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0]) << " valhalla_config transit_land_url per_page [target_directory] [transit_land_api_key]" << std::endl;
    std::cerr << "Sample: " << std::string(argv[0]) << " conf/valhalla.json http://transit.land/ 1000 ./transit_tiles transitland-YOUR_KEY_SUFFIX" << std::endl;
    return 1;
  }

  //args and config file loading
  ptree pt;
  boost::property_tree::read_json(std::string(argv[1]), pt);
  pt.erase("base_url"); pt.add("base_url", std::string(argv[2]));
  pt.erase("per_page"); pt.add("per_page", argc > 3 ? std::string(argv[3]) : std::to_string(1000));
  if(argc > 4) { pt.get_child("mjolnir").erase("transit_dir"); pt.add("mjolnir.transit_dir", std::string(argv[4])); }
  if(argc > 5) { pt.erase("api_key"); pt.add("api_key", std::string(argv[5])); }
  if(argc > 6) { pt.erase("import_level"); pt.add("import_level", std::string(argv[6])); }

  //yes we want to curl
  curl_global_init(CURL_GLOBAL_DEFAULT);

  std::string feed;
  if(argc > 7) { feed = std::string(argv[7]); }

  std::string testfile;
  std::vector<OneStopTest> onestoptests;
  if(argc > 8) {
    testfile = std::string(argv[8]);
    onestoptests = ParseTestFile(testfile);
    std::sort(onestoptests.begin(), onestoptests.end());
  }

  //go get information about what transit tiles we should be fetching
  auto transit_tiles = which_tiles(pt, feed);
  //spawn threads to download all the tiles returning a list of
  //tiles that ended up having dangling stop pairs
  auto dangling_tiles = fetch(pt, transit_tiles);
  curl_global_cleanup();

  //figure out which transit tiles even exist
  TileHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));
  boost::filesystem::recursive_directory_iterator transit_file_itr(pt.get<std::string>("mjolnir.transit_dir") + '/' +
                                                                   std::to_string(hierarchy.levels().rbegin()->first));
  boost::filesystem::recursive_directory_iterator end_file_itr;
  std::unordered_set<GraphId> all_tiles;
  for(; transit_file_itr != end_file_itr; ++transit_file_itr)
    if(boost::filesystem::is_regular(transit_file_itr->path()) && transit_file_itr->path().extension() == ".pbf")
      all_tiles.emplace(GraphTile::GetTileId(transit_file_itr->path().string()));

  //spawn threads to connect dangling stop pairs to adjacent tiles' stops
  stitch(pt, all_tiles, dangling_tiles);

  // update tile dir loc.  Don't want to overwrite the real transit tiles
  if(argc > 4) { pt.get_child("mjolnir").erase("tile_dir"); pt.add("mjolnir.tile_dir", std::string(argv[4])); }

  build(pt, all_tiles, onestoptests);
  ValidateTransit::Validate(pt, all_tiles, onestoptests);

  return 0;
}
