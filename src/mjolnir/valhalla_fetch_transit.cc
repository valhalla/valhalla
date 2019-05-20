#include <cmath>
#include <cstdint>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>
#include <curl/curl.h>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"

#include "mjolnir/admin.h"
#include "mjolnir/servicedays.h"
#include "mjolnir/transitpbf.h"

#include <valhalla/proto/transit.pb.h>

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

std::string url_encode(const std::string& unencoded) {
  char* encoded = curl_escape(unencoded.c_str(), static_cast<int>(unencoded.size()));
  if (encoded == nullptr) {
    throw std::runtime_error("url encoding failed");
  }
  std::string encoded_str(encoded);
  curl_free(encoded);
  return encoded_str;
}

struct logged_error_t : public std::runtime_error {
  logged_error_t(const std::string& msg) : std::runtime_error(msg) {
    LOG_ERROR(msg);
  }
};

// TODO: use curler_t and expand its interface to be more flexible
struct pt_curler_t {
  pt_curler_t() : connection(curl_easy_init(), [](CURL* c) { curl_easy_cleanup(c); }) {
    if (connection.get() == nullptr) {
      throw logged_error_t("Failed to created CURL connection");
    }
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ERRORBUFFER, error),
                "Failed to set error buffer");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_FOLLOWLOCATION, 1L),
                "Failed to set redirect option ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEFUNCTION, write_callback),
                "Failed to set writer ");
  }
  // for now we only need to handle json
  // with templates we could return a string or whatever
  ptree operator()(const std::string& url, const std::string& retry_if_no = "", bool gzip = true) {
    // content encoding header
    if (gzip) {
      char encoding[] = "gzip"; // TODO: allow "identity" and "deflate"
      assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ACCEPT_ENCODING, encoding),
                  "Failed to set gzip content header ");
    }
    // set the url
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_URL, url.c_str()), "Failed to set URL ");
    // dont stop until we have something useful!
    ptree pt;
    size_t tries = 0;
    std::stringstream result;
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEDATA, &result),
                "Failed to set write data ");
    while (++tries) {
      result.str("");
      long http_code = 0;
      std::string log_extra = "Couldn't fetch url ";
      // can we fetch this url
      LOG_DEBUG(url);
      if (curl_easy_perform(connection.get()) == CURLE_OK) {
        curl_easy_getinfo(connection.get(), CURLINFO_RESPONSE_CODE, &http_code);
        log_extra = std::to_string(http_code) + "'d ";
        // it should be 200 OK
        if (http_code == 200) {
          bool threw = false;
          try {
            rapidjson::read_json(result, pt);
          } catch (...) { threw = true; }
          // has to parse and have required info
          if (!threw && (retry_if_no.empty() || pt.get_child_optional(retry_if_no))) {
            break;
          }
          log_extra = "Unusable response ";
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      // dont log rate limit stuff its too frequent
      if (http_code != 429 || (tries % 10) == 0) {
        LOG_WARN(log_extra + "retrying " + url);
      }
    };
    return pt;
  }

protected:
  void assert_curl(CURLcode code, const std::string& msg) {
    if (code != CURLE_OK) {
      throw logged_error_t(msg + error);
    }
  };
  static size_t write_callback(char* in, size_t block_size, size_t blocks, std::stringstream* out) {
    if (!out) {
      return static_cast<size_t>(0);
    }
    out->write(in, block_size * blocks);
    return block_size * blocks;
  }
  std::shared_ptr<CURL> connection;
  char error[CURL_ERROR_SIZE];
};

std::string url(const std::string& path, const ptree& pt) {
  auto url = pt.get<std::string>("base_url") + path;
  auto key = pt.get_optional<std::string>("api_key");
  if (key) {
    url += "&api_key=" + *key;
  }
  return url;
}

// TODO: update this call to get only the tiles that have changed since last time
struct weighted_tile_t {
  GraphId t;
  size_t w;
  bool operator<(const weighted_tile_t& o) const {
    return w == o.w ? t < o.t : w < o.w;
  }
};
std::priority_queue<weighted_tile_t> which_tiles(const ptree& pt, const std::string& feed) {
  // now real need to catch exceptions since we can't really proceed without this stuff
  LOG_INFO("Fetching transit feeds");

  auto transit_bounding_box = pt.get_optional<std::string>("mjolnir.transit_bounding_box")
                                  ? "&bbox=" + pt.get<std::string>("mjolnir.transit_bounding_box")
                                  : "";

  auto import_level = pt.get_optional<std::string>("import_level")
                          ? "&import_level=" + pt.get<std::string>("import_level")
                          : "";

  auto active_feed_version_import_level =
      pt.get_optional<std::string>("import_level")
          ? "&active_feed_version_import_level=" + pt.get<std::string>("import_level")
          : "";

  std::set<GraphId> tiles;
  const auto& tile_level = TileHierarchy::levels().rbegin()->second;
  pt_curler_t curler;
  auto request = url("/api/v1/feeds.geojson?per_page=false", pt);
  request += transit_bounding_box;
  request += active_feed_version_import_level;
  auto feeds = curler(request, "features");
  for (const auto& feature : feeds.get_child("features")) {

    auto onestop_feed = feature.second.get_optional<std::string>("properties.onestop_id");
    if (feed.empty() || (onestop_feed && *onestop_feed == feed)) {

      // should be a polygon
      auto type = feature.second.get_optional<std::string>("geometry.type");
      if (!type || *type != "Polygon") {
        LOG_WARN("Skipping non-polygonal feature: " + feature.second.get_value<std::string>());
        continue;
      }
      // grab the tile row and column ranges for the max box around the polygon
      float min_x = 180, max_x = -180, min_y = 90, max_y = -90;
      for (const auto& coord : feature.second.get_child("geometry.coordinates").front().second) {
        auto x = coord.second.front().second.get_value<float>();
        auto y = coord.second.back().second.get_value<float>();
        if (x < min_x) {
          min_x = x;
        }
        if (x > max_x) {
          max_x = x;
        }
        if (y < min_y) {
          min_y = y;
        }
        if (y > max_y) {
          max_y = y;
        }
      }

      // expand the top and bottom edges of the box to account for geodesics
      min_y -= std::abs(min_y - PointLL(min_x, min_y).MidPoint({max_x, min_y}).second);
      max_y += std::abs(max_y - PointLL(min_x, max_y).MidPoint({max_x, max_y}).second);
      auto min_c = tile_level.tiles.Col(min_x), min_r = tile_level.tiles.Row(min_y);
      auto max_c = tile_level.tiles.Col(max_x), max_r = tile_level.tiles.Row(max_y);
      if (min_c > max_c) {
        std::swap(min_c, max_c);
      }
      if (min_r > max_r) {
        std::swap(min_r, max_r);
      }
      // for each tile in the polygon figure out how heavy it is and keep track of it
      for (auto i = min_c; i <= max_c; ++i) {
        for (auto j = min_r; j <= max_r; ++j) {
          tiles.emplace(GraphId(tile_level.tiles.TileId(i, j), tile_level.level, 0));
        }
      }
    }
  }
  // we want slowest to build tiles first, routes query is slowest so we weight by that
  // stop pairs is most numerous so that might want to be factored in as well
  std::priority_queue<weighted_tile_t> prioritized;
  auto now = time(nullptr);
  auto* utc = gmtime(&now);
  utc->tm_year += 1900;
  ++utc->tm_mon;
  for (const auto& tile : tiles) {
    auto bbox = tile_level.tiles.TileBounds(tile.tileid());
    auto min_y = std::max(bbox.miny(), bbox.minpt().MidPoint({bbox.maxx(), bbox.miny()}).second);
    auto max_y =
        std::min(bbox.maxy(), PointLL(bbox.minx(), bbox.maxy()).MidPoint(bbox.maxpt()).second);
    bbox = AABB2<PointLL>(bbox.minx(), min_y, bbox.maxx(), max_y);
    // stop count
    auto request =
        url((boost::format("/api/v1/stop_stations?total=true&per_page=0&bbox=%1%,%2%,%3%,%4%") %
             bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy())
                .str(),
            pt);
    request += import_level;

    auto stations_total = curler(request, "meta.total").get<size_t>("meta.total");
    /*
    //route count
    request = url((boost::format("/api/v1/routes?total=true&per_page=0&bbox=%1%,%2%,%3%,%4%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy()).str(), pt);
    auto routes_total = curler(request, "meta.total").get<size_t>("meta.total");
    //pair count
    request =
    url((boost::format("/api/v1/schedule_stop_pairs?total=true&per_page=0&bbox=%1%,%2%,%3%,%4%&service_from_date=%5%-%6%-%7%")
      % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy() % utc->tm_year % utc->tm_mon %
    utc->tm_mday).str(), pt); auto pairs_total = curler(request,
    "meta.total").get<size_t>("meta.total");
    */
    // we have anything we want it
    if (stations_total > 0 /* || routes_total > 0|| pairs_total > 0*/) {
      prioritized.push(
          weighted_tile_t{tile,
                          stations_total +
                              10 /* + routes_total * 1000 + pairs_total*/}); // TODO: factor in stop
                                                                             // pairs as well
      LOG_INFO(GraphTile::FileSuffix(tile) + " should have " + std::to_string(stations_total) +
               " stations " /* +
          std::to_string(routes_total) +  " routes and " + std::to_string(pairs_total) +  " stop_pairs"*/);
    }
  }
  LOG_INFO("Finished with " + std::to_string(prioritized.size()) + " transit tiles in " +
           std::to_string(feeds.get_child("features").size()) + " feeds");
  return prioritized;
}

#define set_no_null(T, pt, path, null_value, set)                                                    \
  {                                                                                                  \
    auto value = pt.get<T>(path, null_value);                                                        \
    if (value != null_value)                                                                         \
      set(value);                                                                                    \
  }

void get_stop_stations(Transit& tile,
                       std::unordered_map<std::string, uint64_t>& nodes,
                       std::unordered_map<std::string, uint64_t>& platforms,
                       const GraphId& tile_id,
                       const ptree& response,
                       const AABB2<PointLL>& filter,
                       bool tile_within_one_tz,
                       const std::unordered_multimap<uint32_t, multi_polygon_type>& tz_polys) {

  for (const auto& station_pt : response.get_child("stop_stations")) {

    const auto& ll_pt = station_pt.second.get_child("geometry_centroid.coordinates");
    auto lon = ll_pt.front().second.get_value<float>();
    auto lat = ll_pt.back().second.get_value<float>();
    std::string onestop_id = station_pt.second.get<std::string>("onestop_id");

    if (!filter.Contains({lon, lat}) || station_pt.second.get_child("stop_egresses").size() == 0 ||
        station_pt.second.get_child("stop_platforms").size() == 0 ||
        onestop_id.find('>') != std::string::npos || onestop_id.find('<') != std::string::npos) {
      continue;
    }

    // add egresses first.
    GraphId prev_type_graphid;
    for (const auto& egress_pt : station_pt.second.get_child("stop_egresses")) {
      const auto& ll_pt = egress_pt.second.get_child("geometry_centroid.coordinates");
      auto lon = ll_pt.front().second.get_value<float>();
      auto lat = ll_pt.back().second.get_value<float>();
      auto* node = tile.add_nodes();

      node->set_lon(lon);
      node->set_lat(lat);
      set_no_null(std::string, egress_pt.second, "onestop_id", "null", node->set_onestop_id);
      node->set_type(static_cast<uint32_t>(NodeType::kTransitEgress));
      set_no_null(std::string, egress_pt.second, "name", "null", node->set_name);
      node->set_wheelchair_boarding(egress_pt.second.get<bool>("wheelchair_boarding", true));
      set_no_null(uint64_t, egress_pt.second, "osm_way_id", 0, node->set_osm_way_id);
      node->set_generated(egress_pt.second.get<bool>("generated", true));

      auto traversability = egress_pt.second.get<std::string>("directionality", "null");
      node->set_traversability(static_cast<uint32_t>(Traversability::kBoth));

      // null (both; can enter and exit the egress)
      // enter (can enter the egress from street)
      // exit (can exit egress from street)
      if (traversability == "enter") {
        node->set_traversability(static_cast<uint32_t>(Traversability::kForward));
      } else if (traversability == "exit") {
        node->set_traversability(static_cast<uint32_t>(Traversability::kBackward));
      }

      GraphId egress_id(tile_id.tileid(), tile_id.level(), nodes.size());
      node->set_graphid(egress_id);

      // we want to set the previous id to the first egress in the
      // list so that when we write to the valhalla tile we know
      // where to start.
      if (!prev_type_graphid.Is_Valid()) {
        prev_type_graphid = egress_id;
      }

      auto tz = egress_pt.second.get<std::string>("timezone", "null");
      if (tz != "null") {
        node->set_timezone(tz);
      }

      nodes.emplace(node->onestop_id(), egress_id);
      if (nodes.size() == kMaxGraphId) {
        LOG_ERROR("Hit the maximum number of nodes allowed and skipping the rest");
        return;
      }
    }

    auto* node = tile.add_nodes();
    // stations next.
    node->set_lon(lon);
    node->set_lat(lat);
    set_no_null(std::string, station_pt.second, "onestop_id", "null", node->set_onestop_id);
    node->set_type(static_cast<uint32_t>(NodeType::kTransitStation));
    set_no_null(std::string, station_pt.second, "name", "null", node->set_name);
    node->set_wheelchair_boarding(station_pt.second.get<bool>("wheelchair_boarding", true));
    GraphId station_id(tile_id.tileid(), tile_id.level(), nodes.size());
    node->set_graphid(station_id);

    auto tz = station_pt.second.get<std::string>("timezone", "null");
    if (tz != "null") {
      node->set_timezone(tz);
    }

    node->set_prev_type_graphid(prev_type_graphid);
    nodes.emplace(node->onestop_id(), station_id);
    if (nodes.size() == kMaxGraphId) {
      LOG_ERROR("Hit the maximum number of nodes allowed and skipping the rest");
      return;
    }

    // finally add the platforms
    for (const auto& platforms_pt : station_pt.second.get_child("stop_platforms")) {
      const auto& ll_pt = platforms_pt.second.get_child("geometry_centroid.coordinates");
      auto lon = ll_pt.front().second.get_value<float>();
      auto lat = ll_pt.back().second.get_value<float>();
      auto* node = tile.add_nodes();

      node->set_lon(lon);
      node->set_lat(lat);
      set_no_null(std::string, platforms_pt.second, "onestop_id", "null", node->set_onestop_id);
      node->set_type(static_cast<uint32_t>(NodeType::kMultiUseTransitPlatform));
      set_no_null(std::string, platforms_pt.second, "name", "null", node->set_name);
      node->set_wheelchair_boarding(platforms_pt.second.get<bool>("wheelchair_boarding", true));
      GraphId platform_id(tile_id.tileid(), tile_id.level(), nodes.size());
      node->set_graphid(platform_id);

      auto tz = platforms_pt.second.get<std::string>("timezone", "null");
      if (tz != "null") {
        node->set_timezone(tz);
      }

      node->set_prev_type_graphid(station_id);
      std::string onestop = node->onestop_id();
      if (onestop.back() == '<') {
        onestop.pop_back();
      }

      nodes.emplace(node->onestop_id(), platform_id);
      platforms.emplace(onestop, platform_id);
      if (nodes.size() == kMaxGraphId) {
        LOG_ERROR("Hit the maximum number of nodes allowed and skipping the rest");
        return;
      }
    }
  }
}

void get_routes(Transit& tile,
                std::unordered_map<std::string, size_t>& routes,
                const std::unordered_map<std::string, std::string>& websites,
                const std::unordered_map<std::string, std::string>& short_names,
                const ptree& response) {
  for (const auto& route_pt : response.get_child("routes")) {
    auto* route = tile.add_routes();
    set_no_null(std::string, route_pt.second, "onestop_id", "null", route->set_onestop_id);
    std::string vehicle_type = route_pt.second.get<std::string>("vehicle_type", "null");
    Transit_VehicleType type = Transit_VehicleType::Transit_VehicleType_kRail;
    if (vehicle_type == "tram" || vehicle_type == "tram_service") {
      type = Transit_VehicleType::Transit_VehicleType_kTram;
    } else if (vehicle_type == "metro") {
      type = Transit_VehicleType::Transit_VehicleType_kMetro;
    } else if (vehicle_type == "rail" || vehicle_type == "suburban_railway" ||
               vehicle_type == "railway_service") {
      type = Transit_VehicleType::Transit_VehicleType_kRail;
    } else if (vehicle_type == "bus" || vehicle_type == "trolleybus_service" ||
               vehicle_type == "express_bus_service" || vehicle_type == "local_bus_service" ||
               vehicle_type == "bus_service" || vehicle_type == "shuttle_bus" ||
               vehicle_type == "demand_and_response_bus_service" ||
               vehicle_type == "regional_bus_service" || vehicle_type == "coach_service") {
      type = Transit_VehicleType::Transit_VehicleType_kBus;
    } else if (vehicle_type == "ferry") {
      type = Transit_VehicleType::Transit_VehicleType_kFerry;
    } else if (vehicle_type == "cablecar") {
      type = Transit_VehicleType::Transit_VehicleType_kCableCar;
    } else if (vehicle_type == "gondola") {
      type = Transit_VehicleType::Transit_VehicleType_kGondola;
    } else if (vehicle_type == "funicular") {
      type = Transit_VehicleType::Transit_VehicleType_kFunicular;
    } else {
      LOG_ERROR("Skipping unsupported vehicle_type: " + vehicle_type + " for route " +
                route->onestop_id());
      tile.mutable_routes()->RemoveLast();
      continue;
    }
    route->set_vehicle_type(type);
    set_no_null(std::string, route_pt.second, "operated_by_onestop_id", "null",
                route->set_operated_by_onestop_id);
    set_no_null(std::string, route_pt.second, "name", "null", route->set_name);
    set_no_null(std::string, route_pt.second, "tags.route_long_name", "null",
                route->set_route_long_name);
    set_no_null(std::string, route_pt.second, "tags.route_desc", "null", route->set_route_desc);
    std::string route_color = route_pt.second.get<std::string>("tags.route_color", "FFFFFF");
    std::string route_text_color =
        route_pt.second.get<std::string>("tags.route_text_color", "000000");
    boost::algorithm::trim(route_color);
    boost::algorithm::trim(route_text_color);
    route_color = (route_color == "null" ? "FFFFFF" : route_color);
    route_text_color = (route_text_color == "null" ? "000000" : route_text_color);

    auto website = websites.find(route->operated_by_onestop_id());
    if (website != websites.cend()) {
      route->set_operated_by_website(website->second);
    }

    // use short name (e.g., BART) over long name (e.g., Bay Area Rapid Transit)
    auto short_name = short_names.find(route->operated_by_onestop_id());
    if (short_name != short_names.cend()) {
      route->set_operated_by_name(short_name->second);
    } else
      set_no_null(std::string, route_pt.second, "operated_by_name", "null",
                  route->set_operated_by_name);

    route->set_route_color(strtol(route_color.c_str(), nullptr, 16));
    route->set_route_text_color(strtol(route_text_color.c_str(), nullptr, 16));
    routes.emplace(route->onestop_id(), routes.size());
  }
}

void get_stop_patterns(Transit& tile,
                       std::unordered_map<std::string, size_t>& shapes,
                       const ptree& response) {
  for (const auto& shape_pt : response.get_child("route_stop_patterns")) {
    auto* shape = tile.add_shapes();
    auto shape_id = shape_pt.second.get<std::string>("onestop_id");

    std::vector<PointLL> trip_shape;
    for (const auto& geom : shape_pt.second.get_child("geometry.coordinates")) {
      auto lon = geom.second.front().second.get_value<float>();
      auto lat = geom.second.back().second.get_value<float>();
      trip_shape.emplace_back(PointLL(lon, lat));
    }
    if (trip_shape.size() > 1) {
      // encode the points to reduce size
      shape->set_encoded_shape(encode7(trip_shape));

      // shapes.size()+1 because we can't have a shape id of 0.
      // 0 means shape id is not set in the transit builder.
      shape->set_shape_id(shapes.size() + 1);
      shapes.emplace(shape_id, shape->shape_id());
    }
  }
}

struct unique_transit_t {
  std::mutex lock;
  std::unordered_map<std::string, size_t> trips;
  std::unordered_map<std::string, size_t> block_ids;
  std::unordered_set<std::string> missing_routes;
  std::unordered_map<std::string, size_t> lines;
};

bool get_stop_pairs(Transit& tile,
                    unique_transit_t& uniques,
                    const std::unordered_map<std::string, size_t>& shapes,
                    const ptree& response,
                    const std::unordered_map<std::string, uint64_t>& stops,
                    const std::unordered_map<std::string, size_t>& routes) {
  bool dangles = false;
  for (const auto& pair_pt : response.get_child("schedule_stop_pairs")) {
    auto* pair = tile.add_stop_pairs();

    // origin
    auto origin_id = pair_pt.second.get<std::string>("origin_onestop_id");

    if (origin_id.find('<') == std::string::npos) {
      pair->set_origin_onestop_id(origin_id + "<");
    } else {
      pair->set_origin_onestop_id(origin_id);
    }

    auto origin = stops.find(origin_id);
    if (origin != stops.cend()) {
      pair->set_origin_graphid(origin->second);
    } else {
      dangles = true;
    }

    // destination
    auto destination_id = pair_pt.second.get<std::string>("destination_onestop_id");

    if (destination_id.find('<') == std::string::npos) {
      pair->set_destination_onestop_id(destination_id + "<");
    } else {
      pair->set_destination_onestop_id(destination_id);
    }

    auto destination = stops.find(destination_id);
    if (destination != stops.cend()) {
      pair->set_destination_graphid(destination->second);
    } else {
      dangles = true;
    }

    // um yeah this goes nowhere
    if (pair->origin_onestop_id() == pair->destination_onestop_id()) {
      tile.mutable_stop_pairs()->RemoveLast();
      continue;
    }

    // route
    auto route_id = pair_pt.second.get<std::string>("route_onestop_id");
    auto route = routes.find(route_id);
    if (route == routes.cend()) {
      uniques.lock.lock();
      if (uniques.missing_routes.find(route_id) == uniques.missing_routes.cend()) {
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
    auto frequency_headway_seconds =
        pair_pt.second.get<std::string>("frequency_headway_seconds", "null");
    auto origin_time = pair_pt.second.get<std::string>("origin_departure_time", "null");

    // this will be empty for non frequency trips.
    std::string frequency_time;

    if (frequency_start_time != "null" && frequency_end_time != "null" &&
        frequency_headway_seconds != "null") {

      // this should never happen and if it does then it is a bad frequency.  just continue
      if (origin_time < frequency_start_time) {
        tile.mutable_stop_pairs()->RemoveLast();
        continue;
        // LOG_WARN("Frequency frequency_start_time after origin_time: " +
        // pair->origin_onestop_id()
        // + " --> " + pair->destination_onestop_id());
      }
      pair->set_frequency_end_time(DateTime::seconds_from_midnight(frequency_end_time));
      pair->set_frequency_headway_seconds(std::stoi(frequency_headway_seconds));
      frequency_time = frequency_start_time + frequency_end_time;
    }

    // uniq line id
    auto line_id =
        pair->origin_onestop_id() < pair->destination_onestop_id()
            ? pair->origin_onestop_id() + pair->destination_onestop_id() + route_id + frequency_time
            : pair->destination_onestop_id() + pair->origin_onestop_id() + route_id + frequency_time;
    uniques.lock.lock();
    auto inserted = uniques.lines.insert({line_id, uniques.lines.size()});
    uniques.lock.unlock();

    // timing information
    auto dest_time = pair_pt.second.get<std::string>("destination_arrival_time", "null");
    auto start_date = pair_pt.second.get<std::string>("service_start_date", "null");
    auto end_date = pair_pt.second.get<std::string>("service_end_date", "null");
    if (origin_time == "null" || dest_time == "null" || start_date == "null" || end_date == "null") {
      LOG_ERROR("Missing timing information: " + pair->origin_onestop_id() + " --> " +
                pair->destination_onestop_id());
      tile.mutable_stop_pairs()->RemoveLast();
      continue;
    }

    pair->set_origin_departure_time(DateTime::seconds_from_midnight(origin_time));
    pair->set_destination_arrival_time(DateTime::seconds_from_midnight(dest_time));

    pair->set_service_start_date(
        DateTime::days_from_pivot_date(DateTime::get_formatted_date(start_date)));
    pair->set_service_end_date(
        DateTime::days_from_pivot_date(DateTime::get_formatted_date(end_date)));
    for (const auto& service_days : pair_pt.second.get_child("service_days_of_week")) {
      pair->add_service_days_of_week(service_days.second.get_value<bool>());
    }

    // trip
    std::string trip = pair_pt.second.get<std::string>("trip", "null");
    if (trip == "null") {
      LOG_ERROR("No trip for pair: " + pair->origin_onestop_id() + " --> " +
                pair->destination_onestop_id());
      tile.mutable_stop_pairs()->RemoveLast();
      continue;
    }

    uniques.lock.lock();
    inserted = uniques.trips.insert({trip, uniques.trips.size()});
    pair->set_trip_id(inserted.first->second);
    uniques.lock.unlock();

    // block id
    std::string block_id = pair_pt.second.get<std::string>("block_id", "null");
    if (block_id != "null") {
      uniques.lock.lock();
      // uniques.block_ids.size()+1 because we can't have a block id of 0.
      // 0 means block id is not set in the transit builder.
      inserted = uniques.block_ids.insert({block_id, uniques.block_ids.size() + 1});
      pair->set_block_id(inserted.first->second);
      uniques.lock.unlock();
    }

    pair->set_wheelchair_accessible(pair_pt.second.get<bool>("wheelchair_accessible", true));

    set_no_null(std::string, pair_pt.second, "trip_headsign", "null", pair->set_trip_headsign);
    pair->set_bikes_allowed(pair_pt.second.get<bool>("bikes_allowed", false));

    const auto& except_dates = pair_pt.second.get_child_optional("service_except_dates");
    if (except_dates && !except_dates->empty()) {
      for (const auto& service_except_dates : pair_pt.second.get_child("service_except_dates")) {
        auto d = DateTime::get_formatted_date(service_except_dates.second.get_value<std::string>());
        pair->add_service_except_dates(DateTime::days_from_pivot_date(d));
      }
    }

    const auto& added_dates = pair_pt.second.get_child_optional("service_added_dates");
    if (added_dates && !added_dates->empty()) {
      for (const auto& service_added_dates : pair_pt.second.get_child("service_added_dates")) {
        auto d = DateTime::get_formatted_date(service_added_dates.second.get_value<std::string>());
        pair->add_service_added_dates(DateTime::days_from_pivot_date(d));
      }
    }

    // shape data
    std::string shape_id = pair_pt.second.get<std::string>("route_stop_pattern_onestop_id", "null");
    if (shape_id != "null") {

      auto shape = shapes.find(shape_id);
      if (shape != shapes.cend()) {
        pair->set_shape_id(shape->second);
        std::string origin_dist_traveled =
            pair_pt.second.get<std::string>("origin_dist_traveled", "null");
        if (origin_dist_traveled != "null") {
          pair->set_origin_dist_traveled(std::stof(origin_dist_traveled));
        }

        std::string destination_dist_traveled =
            pair_pt.second.get<std::string>("destination_dist_traveled", "null");
        if (destination_dist_traveled != "null") {
          pair->set_destination_dist_traveled(std::stof(destination_dist_traveled));
        }
      } else {
        LOG_WARN("Shape not found for " + shape_id);
      }
    }
  }

  // no stop pairs due to new route type.
  if (tile.mutable_stop_pairs()->size() == 0) {
    return false;
  }

  return dangles;
}

void fetch_tiles(const ptree& pt,
                 std::priority_queue<weighted_tile_t>& queue,
                 unique_transit_t& uniques,
                 std::promise<std::list<GraphId>>& promise) {
  const auto& tiles = TileHierarchy::levels().rbegin()->second.tiles;
  std::list<GraphId> dangling;
  pt_curler_t curler;
  auto now = time(nullptr);
  auto* utc = gmtime(&now);
  utc->tm_year += 1900;
  ++utc->tm_mon; // TODO: use timezone code?

  auto database = pt.get_optional<std::string>("mjolnir.timezone");
  // Initialize the tz DB (if it exists)
  sqlite3* tz_db_handle = database ? GetDBHandle(*database) : nullptr;
  if (!database) {
    LOG_WARN("Time zone db not found.  Not saving time zone information from db.");
  } else if (!tz_db_handle) {
    LOG_WARN("Time zone db " + *database + " not found.  Not saving time zone information from db.");
  }

  // for each tile
  while (true) {
    GraphId current;
    uniques.lock.lock();
    if (queue.empty()) {
      uniques.lock.unlock();
      break;
    }
    current = queue.top().t;
    queue.pop();
    uniques.lock.unlock();
    auto filter = tiles.TileBounds(current.tileid());
    // expanding both top and bottom by distance to geodesic running through the coords
    auto min_y =
        filter.miny() -
        std::abs(filter.miny() - filter.minpt().MidPoint({filter.maxx(), filter.miny()}).second);
    auto max_y =
        filter.maxy() +
        std::abs(filter.maxy() - filter.maxpt().MidPoint({filter.minx(), filter.maxy()}).second);
    AABB2<PointLL> bbox(filter.minx(), min_y, filter.maxx(), max_y);
    ptree response;
    auto api_key =
        pt.get_optional<std::string>("api_key") ? "&api_key=" + pt.get<std::string>("api_key") : "";
    auto import_level = pt.get_optional<std::string>("import_level")
                            ? "&import_level=" + pt.get<std::string>("import_level")
                            : "";

    Transit tile;
    auto file_name = GraphTile::FileSuffix(current);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    boost::filesystem::path transit_tile = pt.get<std::string>("mjolnir.transit_dir") +
                                           filesystem::path::preferred_separator + file_name;

    // tiles are wrote out with .pbf or .pbf.n ext
    uint32_t ext = 0;
    std::string prefix = transit_tile.string();
    LOG_INFO("Fetching " + transit_tile.string());

    bool tile_within_one_tz = false;
    std::unordered_multimap<uint32_t, multi_polygon_type> tz_polys;
    if (tz_db_handle) {
      tz_polys = GetTimeZones(tz_db_handle, filter);
      if (tz_polys.size() == 1) {
        tile_within_one_tz = true;
      }
    }

    // all the nodes...stations, platforms, and egresses
    std::unordered_map<std::string, uint64_t> nodes;
    // just the platforms
    std::unordered_map<std::string, uint64_t> platforms;
    boost::optional<std::string> request =
        url((boost::format("/api/v1/stop_stations?total=false&per_page=1000&bbox=%1%,%2%,%3%,%4%") %
             bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy())
                .str(),
            pt);
    request = *request + import_level;

    do {
      // grab some stuff
      response = curler(*request, "stop_stations");
      // copy stops in, keeping map of stopid to graphid
      get_stop_stations(tile, nodes, platforms, current, response, filter, tile_within_one_tz,
                        tz_polys);
      // please sir may i have some more?
      request = response.get_optional<std::string>("meta.next");

    } while (request && (request = *request + api_key));
    // um yeah.. we need these
    if (nodes.size() == 0) {
      LOG_WARN(transit_tile.string() + " had no stops and will not be stored");
      continue;
    }

    // pull out all operator WEBSITES
    request =
        url((boost::format("/api/v1/operators?total=false&per_page=%1%&bbox=%2%,%3%,%4%,%5%") %
             pt.get<std::string>("per_page") % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy())
                .str(),
            pt);
    request = *request + import_level;

    std::unordered_map<std::string, std::string> websites;
    std::unordered_map<std::string, std::string> short_names;
    do {
      // grab some stuff
      response = curler(*request, "operators");
      // save the websites to a map
      for (const auto& operators_pt : response.get_child("operators")) {
        std::string onestop_id = operators_pt.second.get<std::string>("onestop_id", "");
        std::string website = operators_pt.second.get<std::string>("website", "");
        if (!onestop_id.empty() && onestop_id != "null" && !website.empty() && website != "null") {
          websites.emplace(onestop_id, website);
        }

        std::string short_name = operators_pt.second.get<std::string>("short_name", "");
        if (!onestop_id.empty() && onestop_id != "null" && !short_name.empty() &&
            short_name != "null") {
          short_names.emplace(onestop_id, short_name);
        }
      }
      // please sir may i have some more?
      request = response.get_optional<std::string>("meta.next");
    } while (request && (request = *request + api_key));

    // pull out all ROUTES
    request =
        url((boost::format(
                 "/api/v1/"
                 "routes?total=false&include_geometry=false&per_page=%1%&bbox=%2%,%3%,%4%,%5%") %
             pt.get<std::string>("per_page") % bbox.minx() % bbox.miny() % bbox.maxx() % bbox.maxy())
                .str(),
            pt);
    std::unordered_map<std::string, size_t> routes;
    request = *request + import_level;

    do {
      // grab some stuff
      uniques.lock.lock();
      response = curler(*request, "routes");
      uniques.lock.unlock();
      // copy routes in, keeping track of routeid to route index
      get_routes(tile, routes, websites, short_names, response);
      // please sir may i have some more?
      request = response.get_optional<std::string>("meta.next");
    } while (request && (request = *request + api_key));

    // pull out all the route_stop_patterns or shapes
    std::unordered_map<std::string, size_t> shapes;
    for (const auto& route : routes) {
      request = url((boost::format(
                         "/api/v1/route_stop_patterns?total=false&per_page=100&traversed_by=%2%") %
                     pt.get<std::string>("per_page") % url_encode(route.first))
                        .str(),
                    pt);
      do {
        // grab some stuff
        response = curler(*request, "route_stop_patterns");
        // copy shapes in.
        get_stop_patterns(tile, shapes, response);
        // please sir may i have some more?
        request = response.get_optional<std::string>("meta.next");
      } while (request && (request = *request + api_key));
    }

    // pull out all SCHEDULE_STOP_PAIRS
    bool dangles = false;
    for (const auto& platform : platforms) {

      request = url((boost::format("/api/v1/"
                                   "schedule_stop_pairs?active=true&total=false&per_page=%1%&"
                                   "origin_onestop_id=%2%&service_from_date=%3%-%4%-%5%") %
                     pt.get<std::string>("per_page") % url_encode(platform.first) % utc->tm_year %
                     utc->tm_mon % utc->tm_mday)
                        .str(),
                    pt);
      request = *request + import_level;
      do {
        // grab some stuff
        response = curler(*request, "schedule_stop_pairs");
        // copy pairs in, noting if any dont have stops
        dangles = get_stop_pairs(tile, uniques, shapes, response, platforms, routes) || dangles;
        // if stop pairs is large save to a path with an incremented extension
        if (tile.stop_pairs_size() >= 500000) {
          LOG_INFO("Writing " + transit_tile.string());
          write_pbf(tile, transit_tile.string());
          // reset everything
          tile.Clear();
          transit_tile = prefix + '.' + std::to_string(ext++);
        }
        // please sir may i have some more?
        request = response.get_optional<std::string>("meta.next");
      } while (request && (request = *request + api_key));
    }

    // remember who dangles
    if (dangles) {
      dangling.emplace_back(current);
    }

    // save the last tile
    if (tile.stop_pairs_size()) {
      write_pbf(tile, transit_tile.string());
    }
  }

  // give back the work for later
  promise.set_value(dangling);
}

std::list<GraphId> fetch(const ptree& pt,
                         std::priority_queue<weighted_tile_t>& tiles,
                         unsigned int thread_count = std::max(static_cast<unsigned int>(1),
                                                              std::thread::hardware_concurrency())) {
  LOG_INFO("Fetching " + std::to_string(tiles.size()) + " transit tiles with " +
           std::to_string(thread_count) + " threads...");

  // schedule some work
  unique_transit_t uniques;
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);
  std::vector<std::promise<std::list<GraphId>>> promises(threads.size());
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].reset(new std::thread(fetch_tiles, std::cref(pt), std::ref(tiles), std::ref(uniques),
                                     std::ref(promises[i])));
  }

  // let the threads finish and get the dangling list
  for (auto& thread : threads) {
    thread->join();
  }
  std::list<GraphId> dangling;
  for (auto& promise : promises) {
    try {
      dangling.splice(dangling.end(), promise.get_future().get());
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Finished");
  return dangling;
}

struct dist_sort_t {
  PointLL center;
  Tiles<PointLL> grid;
  dist_sort_t(const GraphId& center, const Tiles<PointLL>& grid) : grid(grid) {
    this->center = grid.TileBounds(center.tileid()).Center();
  }
  bool operator()(const GraphId& a, const GraphId& b) const {
    auto a_dist = center.Distance(grid.TileBounds(a.tileid()).Center());
    auto b_dist = center.Distance(grid.TileBounds(b.tileid()).Center());
    if (a_dist == b_dist) {
      return a.tileid() < b.tileid();
    }
    return a_dist < b_dist;
  }
};

void stitch_tiles(const ptree& pt,
                  const std::unordered_set<GraphId>& all_tiles,
                  std::list<GraphId>& tiles,
                  std::mutex& lock) {
  auto grid = TileHierarchy::levels().rbegin()->second.tiles;
  auto tile_name = [&pt](const GraphId& id) {
    auto file_name = GraphTile::FileSuffix(id);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    return pt.get<std::string>("mjolnir.transit_dir") + filesystem::path::preferred_separator +
           file_name;
  };

  // for each tile
  while (true) {
    GraphId current;
    lock.lock();
    if (tiles.empty()) {
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

      // open tile make a hash of missing stop to invalid graphid
      auto tile = read_pbf(file_name, lock);
      std::unordered_map<std::string, GraphId> needed;
      for (const auto& stop_pair : tile.stop_pairs()) {
        if (!stop_pair.has_origin_graphid()) {
          needed.emplace(stop_pair.origin_onestop_id(), GraphId{});
        }
        if (!stop_pair.has_destination_graphid()) {
          needed.emplace(stop_pair.destination_onestop_id(), GraphId{});
        }
      }

      // do while we have more to find and arent sick of searching
      std::set<GraphId, dist_sort_t> unchecked(all_tiles.cbegin(), all_tiles.cend(),
                                               dist_sort_t(current, grid));
      size_t found = 0;
      while (found < needed.size() && unchecked.size()) {
        // crack it open to see if it has what we want
        auto neighbor_id = *unchecked.cbegin();
        unchecked.erase(unchecked.begin());
        if (neighbor_id != current) {
          auto neighbor_file_name = tile_name(neighbor_id);
          auto neighbor = read_pbf(neighbor_file_name, lock);
          for (const auto& node : neighbor.nodes()) {
            auto platform_itr = needed.find(node.onestop_id());
            if (platform_itr != needed.cend()) {
              platform_itr->second.value = node.graphid();
              ++found;
            }
          }
        }
      }

      // get the ids fixed up and write pbf to file
      std::unordered_set<std::string> not_found;
      for (auto& stop_pair : *tile.mutable_stop_pairs()) {
        if (!stop_pair.has_origin_graphid()) {
          auto found = needed.find(stop_pair.origin_onestop_id())->second;
          if (found.Is_Valid()) {
            stop_pair.set_origin_graphid(found);
          } else if (not_found.find(stop_pair.origin_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.origin_onestop_id());
            not_found.emplace(stop_pair.origin_onestop_id());
          }
          // else{ TODO: we could delete this stop pair }
        }
        if (!stop_pair.has_destination_graphid()) {
          auto found = needed.find(stop_pair.destination_onestop_id())->second;
          if (found.Is_Valid()) {
            stop_pair.set_destination_graphid(found);
          } else if (not_found.find(stop_pair.destination_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.destination_onestop_id());
            not_found.emplace(stop_pair.destination_onestop_id());
          }
          // else{ TODO: we could delete this stop pair }
        }
      }
      lock.lock();
      auto size = tile.ByteSize();
      valhalla::midgard::mem_map<char> buffer;
      buffer.create(file_name, size);
      tile.SerializeToArray(buffer.get(), size);
      lock.unlock();
      LOG_INFO(file_name + " stitched " + std::to_string(found) + " of " +
               std::to_string(needed.size()) + " stops");

      file_name = prefix + "." + std::to_string(ext++);
    } while (boost::filesystem::exists(file_name));
  }
}

void stitch(const ptree& pt,
            const std::unordered_set<GraphId>& all_tiles,
            std::list<GraphId>& dangling_tiles,
            unsigned int thread_count = std::max(static_cast<unsigned int>(1),
                                                 std::thread::hardware_concurrency())) {
  LOG_INFO("Stitching " + std::to_string(dangling_tiles.size()) + " transit tiles with " +
           std::to_string(thread_count) + " threads...");

  // figure out where the work should go
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);
  std::mutex lock;

  // make let them rip
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].reset(new std::thread(stitch_tiles, std::cref(pt), std::cref(all_tiles),
                                     std::ref(dangling_tiles), std::ref(lock)));
  }

  // wait for them to finish
  for (auto& thread : threads) {
    thread->join();
  }

  LOG_INFO("Finished");
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "Usage: " << std::string(argv[0])
        << " valhalla_config transit_land_url per_page [target_directory] [bounding_box]"
           "[transit_land_api_key] [import_level] [feed_onestop_id] [import_level] [feed_onestop_id]"
        << std::endl;
    std::cerr << "Sample: " << std::string(argv[0])
              << " conf/valhalla.json http://transit.land/ 1000 ./transit_tiles "
                 "-122.469,37.502,-121.78,38.018 transitland-YOUR_KEY_SUFFIX 4 f-9q9-bart"
              << std::endl;
    return 1;
  }

  // args and config file loading
  ptree pt;
  rapidjson::read_json(std::string(argv[1]), pt);
  pt.erase("base_url");
  pt.add("base_url", std::string(argv[2]));
  pt.erase("per_page");
  pt.add("per_page", argc > 3 ? std::string(argv[3]) : std::to_string(1000));
  if (argc > 4) {
    pt.get_child("mjolnir").erase("transit_dir");
    pt.add("mjolnir.transit_dir", std::string(argv[4]));
  }
  if (argc > 5) {
    pt.get_child("mjolnir").erase("transit_bounding_box");
    pt.add("mjolnir.transit_bounding_box", std::string(argv[5]));
  }
  if (argc > 6) {
    pt.erase("api_key");
    pt.add("api_key", std::string(argv[6]));
  }
  if (argc > 7) {
    pt.erase("import_level");
    pt.add("import_level", std::string(argv[7]));
  }

  // yes we want to curl
  curl_global_init(CURL_GLOBAL_DEFAULT);

  std::string feed;
  if (argc > 8) {
    feed = std::string(argv[8]);
  }

  // go get information about what transit tiles we should be fetching
  auto transit_tiles = which_tiles(pt, feed);
  // spawn threads to download all the tiles returning a list of
  // tiles that ended up having dangling stop pairs
  auto dangling_tiles = fetch(pt, transit_tiles);
  curl_global_cleanup();

  // figure out which transit tiles even exist
  boost::filesystem::recursive_directory_iterator transit_file_itr(
      pt.get<std::string>("mjolnir.transit_dir") + filesystem::path::preferred_separator +
      std::to_string(TileHierarchy::levels().rbegin()->first));
  boost::filesystem::recursive_directory_iterator end_file_itr;
  std::unordered_set<GraphId> all_tiles;
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (boost::filesystem::is_regular(transit_file_itr->path()) &&
        transit_file_itr->path().extension() == ".pbf") {
      all_tiles.emplace(GraphTile::GetTileId(transit_file_itr->path().string()));
    }
  }

  // spawn threads to connect dangling stop pairs to adjacent tiles' stops
  stitch(pt, all_tiles, dangling_tiles);

  return 0;
}
