#include <iostream>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"
#include "baldr/graphtile.h"
#include "baldr/gtfs.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"

#include "mjolnir/transitpbf.h"

#include "valhalla/proto/transit.pb.h"

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

// TODO: for debugging delete this
void printQueue(std::queue<GraphId> queue) {
  while (!queue.empty()) {
    std::cout << queue.front() << std::endl;
    queue.pop();
  }
}

namespace one_stop {
enum class OneStopIdType : char { Feed = 'f', Operator = 'o', Stop = 's', Route = 'r' };

std::vector<bool> float_to_bits(float value,
                                float lower = -90.0,
                                float middle = 0.0,
                                float upper = 90.0,
                                int length = 15) {
  // Convert a float to a list of GeoHash bits.
  std::vector<bool> ret;
  for (int i = 0; i < length; i++) {
    if (value >= middle) {
      lower = middle;
      ret.push_back(1);
    } else {
      upper = middle;
      ret.push_back(0);
    }
    middle = (upper + lower) / 2;
  }
  return ret;
}

const std::string base64mapping = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
std::string bits_to_geohash(std::vector<bool>& in) {
  std::string out;
  while (in.size() % 6 != 0) {
    in.push_back(0);
  }
  for (int i = 0; i < in.size(); i += 6) {
    int val = 0;
    for (int j = 5; j >= 0; j--) {
      val += static_cast<int>(in[i + j]) << j;
    }
    out.push_back(base64mapping[val]);
  }
  return out;
}

std::string encode(PointLL pt, int length = 12) {
  length /= 2;
  std::vector<bool> lon = float_to_bits(pt.lng(), -180.0, 0, 180.0, length * 5);
  std::vector<bool> lat = float_to_bits(pt.lat(), -90.0, 0, 90.0, length * 5);

  // appending both
  lon.insert(lon.end(), lat.begin(), lat.end());
  return bits_to_geohash(lon);
}

// one stop id specification https://v1.transit.land/documentation/onestop-id-scheme/
static const PointLL invalidPoint = PointLL();
std::string generate_one_stop_id(OneStopIdType type, std::string name, PointLL pt = invalidPoint) {
  std::string id;
  id.push_back(static_cast<char>(type));
  boost::trim(name);
  std::replace(name.begin(), name.end(), ' ', '~');
  if (pt.IsValid()) {
    id.push_back('-');
    id.append(encode(pt));
  }
  id.push_back('-');
  id.append(name);
  return id.substr(0, 64); // Accroding to transitland one-stopId cannot be greater than 64 char
}
} // namespace one_stop

/*
  For figuring out which tiles to genrate, by looping thorugh all stops
*/
std::queue<GraphId> which_tiles(const ptree& pt, const gtfs::Feed& feed) {
  std::queue<GraphId> tiles;
  auto& tile_level = TileHierarchy::levels().back();

  auto stops = feed.get_stops();
  double min_x = 180, max_x = -180, min_y = 90, max_y = -90;
  for (const auto& stop : stops) {
    auto x = stop.stop_lon;
    auto y = stop.stop_lat;
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
  min_y -= std::abs(min_y - PointLL(min_x, min_y).PointAlongSegment({max_x, min_y}).second);
  max_y += std::abs(max_y - PointLL(min_x, max_y).PointAlongSegment({max_x, max_y}).second);
  auto min_c = tile_level.tiles.Col(min_x), min_r = tile_level.tiles.Row(min_y);
  auto max_c = tile_level.tiles.Col(max_x), max_r = tile_level.tiles.Row(max_y);
  if (min_c > max_c) {
    std::swap(min_c, max_c);
  }
  if (min_r > max_r) {
    std::swap(min_r, max_r);
  }

  for (auto i = min_c; i <= max_c; ++i) {
    for (auto j = min_r; j <= max_r; ++j) {
      tiles.push(GraphId(tile_level.tiles.TileId(i, j), tile_level.level, 0));
    }
  }

  return tiles;
}

struct Station {
  gtfs::Stop station;
  gtfs::Stops platform;
  gtfs::Stops egress;
  bool genrated = false;

  Station(gtfs::Stop station) : station(station) {
  }
};

void save_nodes(Transit& tile,
                const gtfs::Feed& feed,
                const AABB2<PointLL>& bbox,
                const GraphId& tile_id,
                std::unordered_map<gtfs::Id, std::pair<std::string, GraphId>>& platformMapping) {
  gtfs::Stops stops;
  std::copy_if(feed.get_stops().begin(), feed.get_stops().end(), std::back_inserter(stops),
               [bbox](gtfs::Stop stop) {
                 return bbox.Contains({stop.stop_lon, stop.stop_lat});
               });

  /* Converting the stops into hierarchical form, based on location type
    0 - stop/platform
    1 - Station
    2 - Entrance/Exit / Egress
    3 - Generic Node
    4 - Boarding Area.

    Station ----> Stop/platform --->Boarding Area
            |
            ----> Entrance/exit
  */
  std::unordered_map<gtfs::Id, Station*> stations;
  // finding all the stations
  for (const auto& stop : stops) {
    if (stop.location_type == gtfs::StopLocationType::Station) {
      stations[stop.stop_id] = new Station(stop);
    }
  }

  for (const auto& stop : stops) {
    if (stop.location_type == gtfs::StopLocationType::Station) {
      continue;
    }
    if (stop.location_type == gtfs::StopLocationType::StopOrPlatform) {
      if (stations.find(stop.parent_station) == stations.end()) {
        // If the station corresponding to the stop node don't exists make that node as station
        stations[stop.stop_id] = new Station(stop);
        stations[stop.stop_id]->platform.push_back(stop);
        stations[stop.stop_id]->egress.push_back(stop);
        stations[stop.stop_id]->genrated = true;
      } else {
        stations[stop.parent_station]->platform.push_back(stop);
      }
    }
    if (stop.location_type == gtfs::StopLocationType::EntranceExit) { // Egress
      if (stations.find(stop.parent_station) == stations.end()) {
        // this condition should never happen according to GTFS
      } else {
        stations[stop.parent_station]->egress.push_back(stop);
      }
    }
    // We are not considering other types as we don't need them in building tiles
  }

  GraphId prev_type_graphid;
  std::unordered_map<std::string, uint64_t> nodes;

  for (const auto& station : stations) {

    GraphId prev_type_graphid;

    // If for a station egress and stop platform does not exist generate them
    if (station.second->egress.size() == 0) {
      station.second->egress.push_back(station.second->station);
      station.second->genrated = true;
    }

    for (const auto& egress_pt : station.second->egress) {
      auto* node = tile.add_nodes();
      node->set_lon(egress_pt.stop_lon);
      node->set_lat(egress_pt.stop_lat);

      node->set_onestop_id(one_stop::generate_one_stop_id(one_stop::OneStopIdType::Stop,
                                                          egress_pt.stop_name,
                                                          {egress_pt.stop_lon, egress_pt.stop_lat}));
      node->set_type(static_cast<uint32_t>(NodeType::kTransitEgress));

      if (egress_pt.stop_name.size() > 0) {
        node->set_name(egress_pt.stop_name);
      }

      if (egress_pt.wheelchair_boarding.size() == 0 || egress_pt.wheelchair_boarding == "0") {
        if (station.second->station.wheelchair_boarding.size() == 0 ||
            station.second->station.wheelchair_boarding == "0") {
          node->set_wheelchair_boarding(false);
        } else if (station.second->station.wheelchair_boarding == "1") {
          node->set_wheelchair_boarding(true);
        } else {
          node->set_wheelchair_boarding(false);
        }
      } else if (egress_pt.wheelchair_boarding == "1") {
        node->set_wheelchair_boarding(true);
      } else {
        node->set_wheelchair_boarding(false);
      }
      // we will be setting OSM wayid in post processing
      node->set_osm_way_id(0);
      node->set_generated(station.second->genrated);
      node->set_traversability(static_cast<uint32_t>(Traversability::kBoth));

      GraphId egress_id(tile_id.tileid(), tile_id.level(), nodes.size());
      node->set_graphid(egress_id);

      if (egress_pt.stop_timezone.size()) {
        node->set_timezone(egress_pt.stop_timezone);
      }

      if (!prev_type_graphid.Is_Valid()) {
        prev_type_graphid = egress_id;
      }

      nodes.emplace(node->onestop_id(), egress_id);
      if (nodes.size() == kMaxGraphId) {
        LOG_ERROR("Hit the maximum number of nodes allowed and skipping the rest");
        return;
      }
    }

    auto* node = tile.add_nodes();
    const auto& stationNode = station.second->station;
    node->set_lon(stationNode.stop_lon);
    node->set_lat(stationNode.stop_lat);

    node->set_onestop_id(
        one_stop::generate_one_stop_id(one_stop::OneStopIdType::Stop, stationNode.stop_name,
                                       {stationNode.stop_lon, stationNode.stop_lat}));
    node->set_type(static_cast<uint32_t>(NodeType::kTransitStation));
    if (stationNode.stop_name.size() > 0) {
      node->set_name(stationNode.stop_name);
    }

    if (stationNode.wheelchair_boarding.size() == 0 || stationNode.wheelchair_boarding == "0") {
      node->set_wheelchair_boarding(false);
    } else if (stationNode.wheelchair_boarding == "1") {
      node->set_wheelchair_boarding(true);
    } else {
      node->set_wheelchair_boarding(false);
    }
    GraphId station_id(tile_id.tileid(), tile_id.level(), nodes.size());
    node->set_graphid(station_id);

    if (stationNode.stop_timezone.size()) {
      node->set_timezone(stationNode.stop_timezone);
    }
    node->set_prev_type_graphid(prev_type_graphid);

    nodes.emplace(node->onestop_id(), station_id);
    if (nodes.size() == kMaxGraphId) {
      LOG_ERROR("Hit the maximum number of nodes allowed and skipping the rest");
      return;
    }

    if (station.second->platform.size() == 0) {
      station.second->platform.push_back(station.second->station);
      station.second->genrated = true;
    }
    // finally add the platforms
    for (const auto& platforms_pt : station.second->platform) {
      auto* node = tile.add_nodes();
      node->set_lon(platforms_pt.stop_lon);
      node->set_lat(platforms_pt.stop_lat);

      node->set_onestop_id(
          one_stop::generate_one_stop_id(one_stop::OneStopIdType::Stop, platforms_pt.stop_name,
                                         {platforms_pt.stop_lon, platforms_pt.stop_lat}));

      node->set_type(static_cast<uint32_t>(NodeType::kMultiUseTransitPlatform));

      if (platforms_pt.stop_name.size() > 0) {
        node->set_name(platforms_pt.stop_name);
      }

      if (platforms_pt.wheelchair_boarding.size() == 0 || platforms_pt.wheelchair_boarding == "0") {
        if (station.second->station.wheelchair_boarding.size() == 0 ||
            station.second->station.wheelchair_boarding == "0") {
          node->set_wheelchair_boarding(false);
        } else if (station.second->station.wheelchair_boarding == "1") {
          node->set_wheelchair_boarding(true);
        } else {
          node->set_wheelchair_boarding(false);
        }
      } else if (platforms_pt.wheelchair_boarding == "1") {
        node->set_wheelchair_boarding(true);
      } else {
        node->set_wheelchair_boarding(false);
      }
      // we will be setting OSM wayid in post processing
      node->set_osm_way_id(0);
      node->set_generated(station.second->genrated);
      node->set_traversability(static_cast<uint32_t>(Traversability::kBoth));

      GraphId platform_id(tile_id.tileid(), tile_id.level(), nodes.size());
      node->set_graphid(platform_id);

      if (platforms_pt.stop_timezone.size()) {
        node->set_timezone(platforms_pt.stop_timezone);
      }
      node->set_prev_type_graphid(station_id);

      nodes.emplace(node->onestop_id(), platform_id);
      platformMapping[platforms_pt.stop_id] = {node->onestop_id(), platform_id};
      if (nodes.size() == kMaxGraphId) {
        LOG_ERROR("Hit the maximum number of nodes allowed and skipping the rest");
        return;
      }
    }
  }
  // clean up
  for (auto& station : stations) {
    delete station.second;
  }
}

void save_route(Transit& tile,
                const gtfs::Feed& feed,
                const AABB2<PointLL>& bbox,
                const GraphId& tile_id,
                std::unordered_map<gtfs::Id, std::pair<std::string, int>>& routeMapping,
                std::unordered_map<gtfs::Id, int>& shapeMapping) {
  // each tile will contain only that route which begin in that tile
  auto& trips = feed.get_trips();
  std::set<gtfs::Id> filtred_routes_id;
  std::set<gtfs::Id> filtred_shape_id;
  for (auto& trip : trips) {
    const auto& stoptimes = feed.get_stop_times_for_trip(trip.trip_id);
    if (stoptimes.size() == 0) {
      continue;
    }
    boost::optional<gtfs::Stop> firstStop = feed.get_stop(stoptimes[0].stop_id);
    if (firstStop) {
      if (bbox.Contains({firstStop->stop_lon, firstStop->stop_lat})) {
        filtred_routes_id.insert(trip.route_id);
        if (trip.shape_id.size() != 0) {
          filtred_shape_id.insert(trip.shape_id);
        }
      }
    }
  }

  // saving the shapes
  for (auto& shape_id : filtred_shape_id) {
    const auto& shape = feed.get_shape(shape_id);
    auto* shapeNode = tile.add_shapes();
    std::vector<PointLL> trip_shape;
    for (const auto& shape_point : shape) {
      trip_shape.emplace_back(PointLL(shape_point.shape_pt_lon, shape_point.shape_pt_lat));
    }
    if (trip_shape.size() > 1) {
      // encode the points to reduce size
      shapeNode->set_encoded_shape(encode7(trip_shape));

      // shapes.size()+1 because we can't have a shape id of 0.
      // 0 means shape id is not set in the transit builder.
      shapeNode->set_shape_id(shapeMapping.size() + 1);
      shapeMapping.emplace(shape_id, shapeNode->shape_id());
    }
  }

  gtfs::Routes routes;
  for (auto& route_id : filtred_routes_id) {
    boost::optional<gtfs::Route> route = feed.get_route(route_id);
    if (route) {
      routes.push_back(*route);
    }
  }

  for (auto& route_pt : routes) {
    auto* route = tile.add_routes();
    route->set_onestop_id(
        one_stop::generate_one_stop_id(one_stop::OneStopIdType::Route, route_pt.route_long_name));
    gtfs::RouteType vehicle_type = route_pt.route_type;
    Transit_VehicleType type = Transit_VehicleType::Transit_VehicleType_kRail;
    if (vehicle_type == gtfs::RouteType::Tram || vehicle_type == gtfs::RouteType::TramService) {
      type = Transit_VehicleType::Transit_VehicleType_kTram;
    } else if (vehicle_type == gtfs::RouteType::MetroService) {
      type = Transit_VehicleType::Transit_VehicleType_kMetro;
    } else if (vehicle_type == gtfs::RouteType::Rail || vehicle_type == gtfs::RouteType::Subway ||
               vehicle_type == gtfs::RouteType::RailwayService) {
      type = Transit_VehicleType::Transit_VehicleType_kRail;
    } else if (vehicle_type == gtfs::RouteType::Bus || vehicle_type == gtfs::RouteType::Trolleybus ||
               vehicle_type == gtfs::RouteType::ExpressBusService ||
               vehicle_type == gtfs::RouteType::LocalBusService ||
               vehicle_type == gtfs::RouteType::BusService ||
               vehicle_type == gtfs::RouteType::ShuttleBus ||
               vehicle_type == gtfs::RouteType::DemandAndResponseBusService ||
               vehicle_type == gtfs::RouteType::RegionalBusService ||
               vehicle_type == gtfs::RouteType::CoachService) {
      type = Transit_VehicleType::Transit_VehicleType_kBus;
    } else if (vehicle_type == gtfs::RouteType::Ferry) {
      type = Transit_VehicleType::Transit_VehicleType_kFerry;
    } else if (vehicle_type == gtfs::RouteType::CableTram) {
      type = Transit_VehicleType::Transit_VehicleType_kCableCar;
    } else if (vehicle_type == gtfs::RouteType::AerialLift) {
      type = Transit_VehicleType::Transit_VehicleType_kGondola;
    } else if (vehicle_type == gtfs::RouteType::Funicular) {
      type = Transit_VehicleType::Transit_VehicleType_kFunicular;
    } else {
      LOG_ERROR("Skipping unsupported vehicle_type: for route " + route_pt.route_id);
      tile.mutable_routes()->RemoveLast();
      continue;
    }
    route->set_vehicle_type(type);
    const boost::optional<gtfs::Agency> agency = feed.get_agency(route_pt.agency_id);
    if (!agency) {
      LOG_ERROR("Skipping cannot find agency for the route with route-id" + route_pt.route_id);
      tile.mutable_routes()->RemoveLast();
      continue;
    }
    route->set_operated_by_name((*agency).agency_name);
    route->set_operated_by_website((*agency).agency_url);
    route->set_operated_by_onestop_id(
        one_stop::generate_one_stop_id(one_stop::OneStopIdType::Operator, (*agency).agency_name));
    if (!route_pt.route_short_name.size()) {
      route->set_name(route_pt.route_short_name);
    }
    if (!route_pt.route_long_name.size()) {
      route->set_route_long_name(route_pt.route_long_name);
    }
    if (!route_pt.route_desc.size()) {
      route->set_route_desc(route_pt.route_desc);
    }

    std::string route_color = (route_pt.route_color == "" ? "FFFFFF" : route_pt.route_color);
    std::string route_text_color =
        (route_pt.route_text_color == "" ? "FFFFFF" : route_pt.route_text_color);
    boost::algorithm::trim(route_color);
    boost::algorithm::trim(route_text_color);
    route->set_route_color(strtol(route_color.c_str(), nullptr, 16));
    route->set_route_text_color(strtol(route_text_color.c_str(), nullptr, 16));
    routeMapping.insert(
        make_pair(route_pt.route_id, std::make_pair(route->onestop_id(), routeMapping.size())));
  }
}

void save_stop_pair(
    Transit& tile,
    const gtfs::Feed& feed,
    const AABB2<PointLL>& bbox,
    const GraphId& tile_id,
    const std::unordered_map<gtfs::Id, std::pair<std::string, int>>& routeMapping,
    const std::unordered_map<gtfs::Id, int>& shapeMapping,
    const std::unordered_map<gtfs::Id, std::pair<std::string, GraphId>>& platformMapping) {
  // getting all the trips that will belong to routes in this tile
  gtfs::Trips trips;
  trips.reserve(routeMapping.size());
  for (auto& trip : feed.get_trips()) {
    if (routeMapping.find(trip.route_id) != routeMapping.end()) {
      trips.push_back(trip);
    }
  }

  for (auto& trip : trips) {
    const auto& stop_times = feed.get_stop_times_for_trip(trip.trip_id);
    const auto& calender_dates = feed.get_calendar_dates(trip.service_id);
    const auto& calender = feed.get_calendar(trip.service_id);
    const auto& frequency = feed.get_frequencies(trip.service_id);

    if (!calender) {
      LOG_ERROR("Please check your GTFS feed, calender corresponding to service ID" +
                trip.service_id + " not found");
      return;
    }

    for (auto i = 0; i < stop_times.size() - 1; i++) {
      const auto& origin = feed.get_stop(stop_times[i].stop_id);
      const auto& destination = feed.get_stop(stop_times[i + 1].stop_id);

      if (!origin) {
        LOG_ERROR("Please check your GTFS feed, stop with id" + origin.get().stop_id + " not found");
        return;
      }

      if (!destination) {
        LOG_ERROR("Please check your GTFS feed, stop with id" + destination.get().stop_id +
                  " not found");
        return;
      }

      if (!bbox.Contains({origin.get().stop_lon, origin.get().stop_lat})) {
        continue;
      }
      auto* pair = tile.add_stop_pairs();

      const auto& origin_stop_mapping = platformMapping.find(origin.get().stop_id);
      if (origin_stop_mapping != platformMapping.cend()) {
        pair->set_origin_onestop_id(origin_stop_mapping->second.first);
        pair->set_origin_graphid(origin_stop_mapping->second.second);
      } else {
        LOG_ERROR(
            "Cannot happen please file an issue If found this log, origin_stop_mapping not found");
        return;
      }

      const auto& destination_stop_mapping = platformMapping.find(destination.get().stop_id);
      if (destination_stop_mapping != platformMapping.cend()) {
        pair->set_destination_onestop_id(destination_stop_mapping->second.first);
        pair->set_destination_graphid(destination_stop_mapping->second.second);
      } else {
        LOG_ERROR(
            "Cannot happen please file an issue If found this log, destination_stop_mapping not found");
        return;
      }

      // um yeah this goes nowhere
      if (pair->origin_onestop_id() == pair->destination_onestop_id()) {
        tile.mutable_stop_pairs()->RemoveLast();
        continue;
      }

      const auto& route_mapping = routeMapping.find(trip.route_id);
      if (route_mapping == routeMapping.cend()) {
        LOG_ERROR("Please check your GTFS feed, route with id" + trip.route_id + " not found");
        return;
      }
      pair->set_route_index(route_mapping->second.second);

      // auto frequency_start_time = frequency.
      // auto frequency_end_time = pair_pt.second.get<std::string>("frequency_end_time", "null");

      auto origin_time = stop_times[i].departure_time;
      auto dest_time = stop_times[i + 1].arrival_time;
      auto start_date = calender->start_date;
      auto end_date = calender->end_date;

      // TODO handle this
      // pair->set_trip_id(trip.trip_id);

      if (trip.trip_headsign.size() > 0) {
        pair->set_trip_headsign(trip.trip_headsign);
      }

      if (static_cast<int>(trip.wheelchair_accessible)) {
        if (trip.wheelchair_accessible == gtfs::TripAccess::Yes) {
          pair->set_wheelchair_accessible(true);
        }
      } else {
        pair->set_wheelchair_accessible(false);
      }

      if (static_cast<int>(trip.bikes_allowed)) {
        if (trip.bikes_allowed == gtfs::TripAccess::Yes) {
          pair->set_bikes_allowed(true);
        }
      } else {
        pair->set_bikes_allowed(false);
      }

      pair->add_service_days_of_week(calender->monday == gtfs::CalendarAvailability::Available);
      pair->add_service_days_of_week(calender->tuesday == gtfs::CalendarAvailability::Available);
      pair->add_service_days_of_week(calender->wednesday == gtfs::CalendarAvailability::Available);
      pair->add_service_days_of_week(calender->thursday == gtfs::CalendarAvailability::Available);
      pair->add_service_days_of_week(calender->friday == gtfs::CalendarAvailability::Available);
      pair->add_service_days_of_week(calender->saturday == gtfs::CalendarAvailability::Available);
      pair->add_service_days_of_week(calender->sunday == gtfs::CalendarAvailability::Available);

      for (const auto& calender_date : calender_dates) {
        if (calender_date.exception_type == gtfs::CalendarDateException::Added) {
          pair->add_service_added_dates(DateTime::days_from_pivot_date(
              DateTime::get_formatted_date(calender_date.date.get_raw_date())));
        } else {
          pair->add_service_except_dates(DateTime::days_from_pivot_date(
              DateTime::get_formatted_date(calender_date.date.get_raw_date())));
        }
      }

      if (stop_times[i].shape_dist_traveled) {
        pair->set_origin_dist_traveled(stop_times[i].shape_dist_traveled);
      }

      if (stop_times[i + 1].shape_dist_traveled) {
        pair->set_destination_dist_traveled(stop_times[i + 1].shape_dist_traveled);
      }

      // if(const auto& a) {
      //  tile.add_stop_pairs();
      // }

      // skipping setting block id
      pair->set_origin_departure_time(DateTime::seconds_from_midnight(origin_time.get_raw_time()));
      pair->set_destination_arrival_time(DateTime::seconds_from_midnight(dest_time.get_raw_time()));

      pair->set_service_start_date(
          DateTime::days_from_pivot_date(DateTime::get_formatted_date(start_date.get_raw_date())));
      pair->set_service_end_date(
          DateTime::days_from_pivot_date(DateTime::get_formatted_date(end_date.get_raw_date())));
    }
  }
}

struct unique_transit_t {
  std::mutex lock;
  int thread_in_zone = 0;
  std::condition_variable cv;
  std::unordered_map<std::string, size_t> trips;
  std::unordered_map<std::string, size_t> block_ids;
  std::unordered_set<std::string> missing_routes;
  std::unordered_map<std::string, size_t> lines;
};

void build_tiles(const ptree& pt, std::queue<GraphId>& queue, const gtfs::Feed& feed) {

  const auto& tiles = TileHierarchy::levels().back().tiles;

  std::unordered_map<GraphId, Transit> tiles_map;
  std::unordered_map<gtfs::Id, std::pair<std::string, GraphId>> platforms;
  std::unordered_map<gtfs::Id, std::pair<std::string, int>> routes;
  std::unordered_map<gtfs::Id, int> shapes;

  while (true) {
    GraphId current;
    if (queue.empty()) {
      break;
    }
    current = queue.front();
    queue.pop();
    auto filter = tiles.TileBounds(current.tileid());
    auto min_y = filter.miny() -
                 std::abs(filter.miny() -
                          filter.minpt().PointAlongSegment({filter.maxx(), filter.miny()}).second);
    auto max_y = filter.maxy() +
                 std::abs(filter.maxy() -
                          filter.maxpt().PointAlongSegment({filter.minx(), filter.maxy()}).second);
    AABB2<PointLL> bbox(filter.minx(), min_y, filter.maxx(), max_y);
    tiles_map[current] = Transit();
    save_nodes(tiles_map[current], feed, bbox, current, platforms);
    save_route(tiles_map[current], feed, bbox, current, routes, shapes);
    // save_stop_pair(tile, feed, bbox, current, routes, shapes, platforms);
  }
  for (auto& tile_map : tiles_map) {
    auto filter = tiles.TileBounds(tile_map.first.tileid());
    auto min_y = filter.miny() -
                 std::abs(filter.miny() -
                          filter.minpt().PointAlongSegment({filter.maxx(), filter.miny()}).second);
    auto max_y = filter.maxy() +
                 std::abs(filter.maxy() -
                          filter.maxpt().PointAlongSegment({filter.minx(), filter.maxy()}).second);
    AABB2<PointLL> bbox(filter.minx(), min_y, filter.maxx(), max_y);
    save_stop_pair(tile_map.second, feed, bbox, tile_map.first, routes, shapes, platforms);

    auto file_name = GraphTile::FileSuffix(tile_map.first, SUFFIX_NON_COMPRESSED);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    filesystem::path transit_tile = pt.get<std::string>("mjolnir.transit_dir") +
                                    filesystem::path::preferred_separator + file_name;
    if (tile_map.second.stop_pairs_size()) {
      write_pbf(tile_map.second, transit_tile.string());
    }
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0])
              << " valhalla_config [gtfs directory] [target_directory]" << std::endl;
    std::cerr << "Sample: " << std::string(argv[0])
              << " conf/valhalla.json ./nyc_subway_gtfs  ./transit_tiles " << std::endl;
    return 1;
  }

  // args and config file loading
  ptree pt;
  rapidjson::read_json(std::string(argv[1]), pt);
  if (argc > 2) {
    pt.get_child("mjolnir").erase("gtfs_dir");
    pt.add("mjolnir.gtfs_dir", std::string(argv[2]));
  }
  if (argc > 3) {
    pt.get_child("mjolnir").erase("transit_dir");
    pt.add("mjolnir.transit_dir", std::string(argv[3]));
  }

  gtfs::Feed feed(pt.get<std::string>("mjolnir.gtfs_dir"));

  auto result = feed.read_feed();
  if (result.code != gtfs::ResultCode::OK) {
    std::cerr << "Error reading gtfs file " << result.message;
    return 1;
  }

  // go get information about what transit tiles we should be fetching
  auto transit_tiles = which_tiles(pt, feed);
  printQueue(transit_tiles); // TODO: for debugging delete this

  // Transit transit;
  build_tiles(pt, transit_tiles, feed);

  return 0;
}