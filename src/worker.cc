#include <iostream>
#include <sstream>
#include <unordered_map>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/location.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "odin/util.h"
#include "proto_conversions.h"
#include "sif/costfactory.h"
#include "worker.h"

using namespace valhalla;
#ifdef HAVE_HTTP
using namespace prime_server;
#endif

namespace {
// Credits: http://werkzeug.pocoo.org/
const std::unordered_map<unsigned, std::string> HTTP_STATUS_CODES{
    // 1xx
    {100, "Continue"},
    {101, "Switching Protocols"},
    {102, "Processing"},

    // 2xx
    {200, "OK"},
    {201, "Created"},
    {202, "Accepted"},
    {203, "Non Authoritative Information"},
    {204, "No Content"},
    {205, "Reset Content"},
    {206, "Partial Content"},
    {207, "Multi Status"},
    {226, "IM Used"}, // see RFC 322

    // 3xx
    {300, "Multiple Choices"},
    {301, "Moved Permanently"},
    {302, "Found"},
    {303, "See Other"},
    {304, "Not Modified"},
    {305, "Use Proxy"},
    {307, "Temporary Redirect"},

    // 4xx
    {400, "Bad Request"},
    {401, "Unauthorized"},
    {402, "Payment Required"}, // unuse
    {403, "Forbidden"},
    {404, "Not Found"},
    {405, "Method Not Allowed"},
    {406, "Not Acceptable"},
    {407, "Proxy Authentication Required"},
    {408, "Request Timeout"},
    {409, "Conflict"},
    {410, "Gone"},
    {411, "Length Required"},
    {412, "Precondition Failed"},
    {413, "Request Entity Too Large"},
    {414, "Request URI Too Long"},
    {415, "Unsupported Media Type"},
    {416, "Requested Range Not Satisfiable"},
    {417, "Expectation Failed"},
    {418, "I\'m a teapot"}, // see RFC 232
    {422, "Unprocessable Entity"},
    {423, "Locked"},
    {424, "Failed Dependency"},
    {426, "Upgrade Required"},
    {428, "Precondition Required"}, // see RFC 658
    {429, "Too Many Requests"},
    {431, "Request Header Fields Too Large"},
    {449, "Retry With"}, // proprietary MS extension

    // 5xx
    {500, "Internal Server Error"},
    {501, "Not Implemented"},
    {502, "Bad Gateway"},
    {503, "Service Unavailable"},
    {504, "Gateway Timeout"},
    {505, "HTTP Version Not Supported"},
    {507, "Insufficient Storage"},
    {510, "Not Extended"},
};

// from valhalla error code to http status code
const std::unordered_map<unsigned, unsigned> ERROR_TO_STATUS{
    {100, 400}, {101, 405}, {102, 503}, {106, 404}, {107, 501},

    {110, 400}, {111, 400}, {112, 400}, {113, 400}, {114, 400},

    {120, 400}, {121, 400}, {122, 400}, {123, 400}, {124, 400}, {125, 400}, {126, 400}, {127, 400},

    {130, 400}, {131, 400}, {132, 400}, {133, 400}, {136, 400}, {137, 400},

    {140, 400}, {141, 501}, {142, 501}, {143, 400},

    {150, 400}, {151, 400}, {152, 400}, {153, 400}, {154, 400}, {155, 400}, {156, 400}, {157, 400},
    {158, 400}, {159, 400},

    {160, 400}, {161, 400}, {162, 400}, {163, 400}, {164, 400}, {165, 400}, {166, 400}, {167, 400},

    {170, 400}, {171, 400}, {172, 400},

    {199, 400},

    {200, 500}, {201, 500}, {202, 500}, {203, 503},

    {210, 400}, {211, 400}, {212, 400}, {213, 400},

    {220, 400},

    {230, 400}, {231, 400}, {232, 400},

    {299, 400},

    {304, 404}, {305, 501},

    {310, 400}, {311, 400}, {312, 400}, {313, 400}, {314, 400},

    {399, 400},

    {400, 400}, {401, 500}, {402, 503},

    {420, 400}, {421, 400}, {422, 400}, {423, 400}, {424, 400},

    {430, 400},

    {440, 400}, {441, 400}, {442, 400}, {443, 400}, {444, 400}, {445, 400},

    {499, 400},

    {500, 500}, {501, 500}, {502, 400},

    {599, 400},
};

const std::unordered_map<unsigned, std::string> OSRM_ERRORS_CODES{
    // loki project 1xx
    {100, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {101, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {102, R"({"code":"ServiceUnavailable","message":"The service is shutting down."})"},
    {106, R"({"code":"InvalidService","message":"Service name is invalid."})"},
    {107, R"({"code":"InvalidService","message":"Service name is invalid."})"},
    {110, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {111, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {112, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {113, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {114, R"({"code":"InvalidOptions","message":"Options are invalid."})"},

    {120, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {121, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {122, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {123, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {124, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {125, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {126, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {127, R"({"code":"InvalidOptions","message":"Options are invalid."})"},

    {130,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {131,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {132,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {133,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {134,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {135,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {136,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {137,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

    {140,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {141,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {142,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {143,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

    {150,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {151,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {152,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {153,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    // OSRM has no equivalent message for this case so we return our own
    {154, R"({"code":"DistanceExceeded","message":"Path distance exceeds the max distance limit."})"},
    {155, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {156, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {157,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {158,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {159,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

    {160, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {161, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {162,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {163,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {164,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {165, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {167,
     R"({"code":"PerimeterExceeded","message":"Perimeter of avoid polygons exceeds the max limit."})"},

    {170, R"({"code":"NoRoute","message":"Impossible route between points"})"},
    {171,
     R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})"},
    {172,
     R"({"code":"BreakageDistanceExceeded","message":"All coordinates are too far away from each other"})"},

    {199, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    // odin project 2xx
    {200, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {201, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {202, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {203, R"({"code":"ServiceUnavailable","message":"The service is shutting down."})"},

    {210, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {211, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {212, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {213, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    {220, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    {230, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {231, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {232, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    {299, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    // skadi project 3xx
    {304, R"({"code":"InvalidService","message":"Service name is invalid."})"},
    {305, R"({"code":"InvalidService","message":"Service name is invalid."})"},

    {310, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {311, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {312, R"({"code":"InvalidOptions","message":"Options are invalid."})"},
    {313, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {314,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

    {399, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    // thor project 4xx
    {400, R"({"code":"InvalidService","message":"Service name is invalid."})"},
    {401, R"({"code":"InvalidUrl","message":"Failed to serialize route."})"},
    {402, R"({"code":"ServiceUnavailable","message":"The service is shutting down."})"},

    {420,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {421,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {422,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {423,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {424,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

    {430, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    {440, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {441, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {442, R"({"code":"NoRoute","message":"Impossible route between points"})"},
    {443,
     R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})"},
    {444,
     R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})"},
    {445, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    {499, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    // tyr project 5xx
    {500, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {501, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {502, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    {599, R"({"code":"InvalidUrl","message":"URL string is invalid."})"}};

rapidjson::Document from_string(const std::string& json, const valhalla_exception_t& e) {
  rapidjson::Document d;
  if (json.empty()) {
    d.SetObject();
    return d;
  }
  d.Parse(json.c_str());
  if (d.HasParseError()) {
    throw e;
  }
  return d;
}

void add_date_to_locations(Options& options,
                           google::protobuf::RepeatedPtrField<valhalla::Location>& locations) {
  // otherwise we do what the person was asking for
  if (options.has_date_time() && !locations.empty()) {
    switch (options.date_time_type()) {
      case Options::current:
        locations.Mutable(0)->set_date_time("current");
        break;
      case Options::depart_at:
        locations.Mutable(0)->set_date_time(options.date_time());
        break;
      case Options::arrive_by:
        locations.Mutable(locations.size() - 1)->set_date_time(options.date_time());
        break;
      case Options::invariant:
        for (auto& loc : locations)
          loc.set_date_time(options.date_time());
      default:
        break;
    }
  }
}

// Parses JSON rings of the form [[lon1, lat1], [lon2, lat2], ...]] and operates on
// PBF objects of the sort "repeated LatLng". Open rings will be closed.
template <typename ring_pbf_t>
void parse_ring(ring_pbf_t& ring, const rapidjson::Value& coord_array) {
  for (const auto& coords : coord_array.GetArray()) {
    if (coords.Size() < 2) {
      throw std::runtime_error("Polygon coordinates must consist of [Lon, Lat] arrays.");
    }

    double lon = coords[0].GetDouble();
    lon = midgard::circular_range_clamp<double>(lon, -180, 180);
    double lat = coords[1].GetDouble();
    if (lat < -90.0 || lat > 90.0) {
      throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
    }

    auto* ll = ring->add_coords();
    ll->set_lng(lon);
    ll->set_lat(lat);
  }

  bool is_open = (ring->coords().begin()->lat() != ring->coords().rbegin()->lat() ||
                  ring->coords().begin()->lng() != ring->coords().rbegin()->lng());

  // close open rings
  if (!ring->coords().empty() && is_open) {
    ring->add_coords()->CopyFrom(*ring->coords().begin());
  }
}

void parse_locations(const rapidjson::Document& doc,
                     Options& options,
                     const std::string& node,
                     unsigned location_parse_error_code,
                     const boost::optional<bool>& ignore_closures) {

  google::protobuf::RepeatedPtrField<valhalla::Location>* locations = nullptr;
  if (node == "locations") {
    locations = options.mutable_locations();
  } else if (node == "shape") {
    locations = options.mutable_shape();
  } else if (node == "trace") {
    locations = options.mutable_trace();
  } else if (node == "sources") {
    locations = options.mutable_sources();
  } else if (node == "targets") {
    locations = options.mutable_targets();
  } else if (node == "exclude_locations" || node == "avoid_locations") {
    locations = options.mutable_exclude_locations();
  } else {
    return;
  }

  bool had_date_time = false;
  bool exclude_closures_disabled = false;
  auto request_locations =
      rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, std::string("/" + node).c_str());
  if (request_locations) {
    for (const auto& r_loc : *request_locations) {
      try {
        auto* location = locations->Add();
        location->set_original_index(locations->size() - 1);

        auto lat = rapidjson::get_optional<double>(r_loc, "/lat");
        if (!lat) {
          throw std::runtime_error{"lat is missing"};
        };

        if (*lat < -90.0 || *lat > 90.0) {
          throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
        }

        auto lon = rapidjson::get_optional<double>(r_loc, "/lon");
        if (!lon) {
          throw std::runtime_error{"lon is missing"};
        };

        lon = midgard::circular_range_clamp<double>(*lon, -180, 180);
        location->mutable_ll()->set_lat(*lat);
        location->mutable_ll()->set_lng(*lon);

        // trace attributes does not support legs or breaks at discontinuities
        auto stop_type_json = rapidjson::get_optional<std::string>(r_loc, "/type");
        if (options.action() == Options::trace_attributes) {
          location->set_type(valhalla::Location::kVia);
        } // other actions let you specify whatever type of stop you want
        else if (stop_type_json) {
          Location::Type type = Location::kBreak;
          Location_Type_Enum_Parse(*stop_type_json, &type);
          location->set_type(type);
        } // and if you didnt set it it defaulted to break which is not the default for trace_route
        else if (options.action() == Options::trace_route) {
          location->set_type(valhalla::Location::kVia);
        }

        auto name = rapidjson::get_optional<std::string>(r_loc, "/name");
        if (name) {
          location->set_name(*name);
        }
        auto street = rapidjson::get_optional<std::string>(r_loc, "/street");
        if (street) {
          location->set_street(*street);
        }
        auto city = rapidjson::get_optional<std::string>(r_loc, "/city");
        if (city) {
          location->set_city(*city);
        }
        auto state = rapidjson::get_optional<std::string>(r_loc, "/state");
        if (state) {
          location->set_state(*state);
        }
        auto zip = rapidjson::get_optional<std::string>(r_loc, "/postal_code");
        if (zip) {
          location->set_postal_code(*zip);
        }
        auto country = rapidjson::get_optional<std::string>(r_loc, "/country");
        if (country) {
          location->set_country(*country);
        }
        auto phone = rapidjson::get_optional<std::string>(r_loc, "/phone");
        if (phone) {
          location->set_phone(*phone);
        }
        auto url = rapidjson::get_optional<std::string>(r_loc, "/url");
        if (url) {
          location->set_url(*url);
        }

        auto date_time = rapidjson::get_optional<std::string>(r_loc, "/date_time");
        if (date_time) {
          location->set_date_time(*date_time);
          had_date_time = true;
        }
        auto heading = rapidjson::get_optional<int>(r_loc, "/heading");
        if (heading) {
          location->set_heading(*heading);
        }
        auto heading_tolerance = rapidjson::get_optional<int>(r_loc, "/heading_tolerance");
        if (heading_tolerance) {
          location->set_heading_tolerance(*heading_tolerance);
        }
        auto node_snap_tolerance = rapidjson::get_optional<float>(r_loc, "/node_snap_tolerance");
        if (node_snap_tolerance) {
          location->set_node_snap_tolerance(*node_snap_tolerance);
        }
        auto way_id = rapidjson::get_optional<uint64_t>(r_loc, "/way_id");
        if (way_id) {
          location->set_way_id(*way_id);
        }
        auto minimum_reachability =
            rapidjson::get_optional<unsigned int>(r_loc, "/minimum_reachability");
        if (minimum_reachability) {
          location->set_minimum_reachability(*minimum_reachability);
        }
        auto radius = rapidjson::get_optional<unsigned int>(r_loc, "/radius");
        if (radius) {
          location->set_radius(*radius);
        }
        auto accuracy = rapidjson::get_optional<unsigned int>(r_loc, "/accuracy");
        if (accuracy) {
          location->set_accuracy(*accuracy);
        }
        auto time = rapidjson::get_optional<unsigned int>(r_loc, "/time");
        if (time) {
          location->set_time(*time);
        }
        auto rank_candidates = rapidjson::get_optional<bool>(r_loc, "/rank_candidates");
        if (rank_candidates) {
          location->set_rank_candidates(*rank_candidates);
        }
        auto preferred_side = rapidjson::get_optional<std::string>(r_loc, "/preferred_side");
        valhalla::Location::PreferredSide side;
        if (preferred_side && PreferredSide_Enum_Parse(*preferred_side, &side)) {
          location->set_preferred_side(side);
        }
        lat = rapidjson::get_optional<double>(r_loc, "/display_lat");
        lon = rapidjson::get_optional<double>(r_loc, "/display_lon");
        if (lat && lon && *lat >= -90.0 && *lat <= 90.0) {
          lon = midgard::circular_range_clamp<double>(*lon, -180, 180);
          location->mutable_display_ll()->set_lat(*lat);
          location->mutable_display_ll()->set_lng(*lon);
        }
        auto search_cutoff = rapidjson::get_optional<unsigned int>(r_loc, "/search_cutoff");
        if (search_cutoff) {
          location->set_search_cutoff(*search_cutoff);
        }
        auto street_side_tolerance =
            rapidjson::get_optional<unsigned int>(r_loc, "/street_side_tolerance");
        if (street_side_tolerance) {
          location->set_street_side_tolerance(*street_side_tolerance);
        }
        auto street_side_max_distance =
            rapidjson::get_optional<unsigned int>(r_loc, "/street_side_max_distance");
        if (street_side_max_distance) {
          location->set_street_side_max_distance(*street_side_max_distance);
        }

        auto search_filter = rapidjson::get_child_optional(r_loc, "/search_filter");
        if (search_filter) {
          // search_filter.min_road_class
          auto min_road_class =
              rapidjson::get_optional<std::string>(*search_filter, "/min_road_class");
          valhalla::RoadClass min_rc;
          if (min_road_class && RoadClass_Enum_Parse(*min_road_class, &min_rc)) {
            location->mutable_search_filter()->set_min_road_class(min_rc);
          }
          // search_filter.max_road_class
          auto max_road_class =
              rapidjson::get_optional<std::string>(*search_filter, "/max_road_class");
          valhalla::RoadClass max_rc;
          if (max_road_class && RoadClass_Enum_Parse(*max_road_class, &max_rc)) {
            location->mutable_search_filter()->set_max_road_class(max_rc);
          }
          // search_filter.exclude_tunnel
          location->mutable_search_filter()->set_exclude_tunnel(
              rapidjson::get_optional<bool>(*search_filter, "/exclude_tunnel").get_value_or(false));
          // search_filter.exclude_bridge
          location->mutable_search_filter()->set_exclude_bridge(
              rapidjson::get_optional<bool>(*search_filter, "/exclude_bridge").get_value_or(false));
          // search_filter.exclude_ramp
          location->mutable_search_filter()->set_exclude_ramp(
              rapidjson::get_optional<bool>(*search_filter, "/exclude_ramp").get_value_or(false));
        }

        // search_filter.exclude_closures must always be set because ignore_closures overrides it
        // so if only ignore_closures is set we still need to set the search filter
        auto exclude_closures =
            search_filter ? rapidjson::get_optional<bool>(*search_filter, "/exclude_closures")
                          : boost::none;
        // bail if you specified both of these, too confusing to work out how to use both at once
        if (ignore_closures && exclude_closures) {
          throw valhalla_exception_t{143};
        }
        // do we actually want to filter closures on THIS location
        // NOTE: that ignore_closures takes precedence
        location->mutable_search_filter()->set_exclude_closures(
            ignore_closures ? !(*ignore_closures) : exclude_closures ? *exclude_closures : true);
        // set exclude_closures_disabled if any of the locations has the
        // search_filter.exclude_closures set as false
        if (!location->search_filter().exclude_closures()) {
          exclude_closures_disabled = true;
        }
      }
      // Forward valhalla_exception_t types as-is, since they contain a more
      // specific error message
      catch (const valhalla_exception_t& e) {
        throw e;
      } catch (...) { throw valhalla_exception_t{location_parse_error_code}; }
    }

    // first and last locations get the default type of break no matter what
    if (locations->size()) {
      locations->Mutable(0)->set_type(valhalla::Location::kBreak);
      locations->Mutable(locations->size() - 1)->set_type(valhalla::Location::kBreak);
    }

    // push the date time information down into the locations
    if (!had_date_time) {
      add_date_to_locations(options, *locations);
    }

    // If any of the locations had search_filter.exclude_closures set to false,
    // we tell the costing to let all closed roads through, so that we can do
    // a secondary per-location filtering using loki's search_filter
    // functionality
    if (exclude_closures_disabled) {
      for (auto& costing : *options.mutable_costing_options()) {
        costing.set_filter_closures(false);
      }
    }
  }
}

void parse_contours(const rapidjson::Document& doc,
                    google::protobuf::RepeatedPtrField<Contour>* contours) {

  // make sure the isoline definitions are valid
  auto json_contours = rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, "/contours");
  if (json_contours) {
    for (const auto& json_contour : *json_contours) {
      // Grab contour time and distance
      auto t = rapidjson::get_optional<float>(json_contour, "/time");
      auto d = rapidjson::get_optional<float>(json_contour, "/distance");

      // You need at least something
      if (!t && !d) {
        throw valhalla_exception_t{111};
      }

      // Set contour time/distance
      auto* contour = contours->Add();
      if (t) {
        contour->set_time(*t);
      }
      if (d) {
        contour->set_distance(*d);
      }

      // If specified, grab and set contour color
      auto color = rapidjson::get_optional<std::string>(json_contour, "/color");
      if (color) {
        contour->set_color(*color);
      }
    }
  }
}

void from_json(rapidjson::Document& doc, Options& options) {
  // TODO: stop doing this after a sufficient amount of time has passed
  // move anything nested in deprecated directions_options up to the top level
  auto deprecated = get_child_optional(doc, "/directions_options");
  auto& allocator = doc.GetAllocator();
  if (deprecated) {
    for (const auto& key : {"/units", "/narrative", "/format", "/language"}) {
      auto child = rapidjson::get_child_optional(*deprecated, key);
      if (child) {
        doc.AddMember(rapidjson::Value(&key[1], allocator), *child, allocator);
      }
    }
    // delete options if it existed
    doc.RemoveMember("directions_options");
  }

  auto fmt = rapidjson::get_optional<std::string>(doc, "/format");
  Options::Format format;
  if (fmt && Options_Format_Enum_Parse(*fmt, &format)) {
    options.set_format(format);
  }

  auto id = rapidjson::get_optional<std::string>(doc, "/id");
  if (id) {
    options.set_id(*id);
  }

  auto jsonp = rapidjson::get_optional<std::string>(doc, "/jsonp");
  if (jsonp) {
    options.set_jsonp(*jsonp);
  }

  auto units = rapidjson::get_optional<std::string>(doc, "/units");
  if (units) {
    if ((*units == "miles") || (*units == "mi")) {
      options.set_units(Options::miles);
    } else {
      options.set_units(Options::kilometers);
    }
  }

  auto language = rapidjson::get_optional<std::string>(doc, "/language");
  if (language && odin::get_locales().find(*language) != odin::get_locales().end()) {
    options.set_language(*language);
  }

  // deprecated
  auto narrative = rapidjson::get_optional<bool>(doc, "/narrative");
  if (narrative && !*narrative) {
    options.set_directions_type(DirectionsType::none);
  }

  auto dir_type = rapidjson::get_optional<std::string>(doc, "/directions_type");
  DirectionsType directions_type;
  if (dir_type && DirectionsType_Enum_Parse(*dir_type, &directions_type)) {
    options.set_directions_type(directions_type);
  }

  // date_time
  auto date_time_type = rapidjson::get_optional<unsigned int>(doc, "/date_time/type");
  if (date_time_type && Options::DateTimeType_IsValid(*date_time_type)) {
    // check the type is in bounds
    auto const v = static_cast<Options::DateTimeType>(*date_time_type);
    if (v >= Options::DateTimeType_ARRAYSIZE)
      throw valhalla_exception_t{163};
    options.set_date_time_type(static_cast<Options::DateTimeType>(v));
    // check the value exists for depart at and arrive by
    auto date_time_value = v != Options::current
                               ? rapidjson::get_optional<std::string>(doc, "/date_time/value")
                               : std::string("current");
    if (!date_time_value) {
      if (v == Options::depart_at)
        throw valhalla_exception_t{160};
      else if (v == Options::arrive_by)
        throw valhalla_exception_t{161};
      else if (v == Options::invariant)
        throw valhalla_exception_t{165};
    }
    // check the value is sane
    if (*date_time_value != "current" && !baldr::DateTime::is_iso_valid(*date_time_value))
      throw valhalla_exception_t{162};
    options.set_date_time(*date_time_value);
  } // not specified but you want transit, then we default to current
  else if (options.has_costing() &&
           (options.costing() == multimodal || options.costing() == transit)) {
    options.set_date_time_type(Options::current);
    options.set_date_time("current");
  }

  // failure scenarios with respect to time dependence
  if (options.has_date_time_type()) {
    if (options.date_time_type() == Options::arrive_by ||
        options.date_time_type() == Options::invariant) {
      if (options.costing() == multimodal || options.costing() == transit)
        throw valhalla_exception_t{141};
      if (options.action() == Options::isochrone)
        throw valhalla_exception_t{142};
    }
  }

  // Set the output precision for shape/geometry (polyline encoding). Defaults to polyline6
  // This also controls the input precision for encoded_polyline in height action
  // TODO - this just for OSRM compatibility at the moment but could be supported
  options.set_shape_format(polyline6);
  auto shape_format = rapidjson::get_optional<std::string>(doc, "/shape_format");
  if (shape_format) {
    if (*shape_format == "polyline6") {
      options.set_shape_format(polyline6);
    } else if (*shape_format == "polyline5") {
      options.set_shape_format(polyline5);
    } else if (*shape_format == "geojson") {
      options.set_shape_format(geojson);
    } else {
      // Throw an error if shape format is invalid
      throw valhalla_exception_t{164};
    }
  }

  // whether or not to output b64 encoded openlr
  auto linear_references = rapidjson::get_optional<bool>(doc, "/linear_references");
  if (linear_references) {
    options.set_linear_references(*linear_references);
  }

  // costing defaults to none which is only valid for locate
  auto costing_str = rapidjson::get<std::string>(doc, "/costing", "none");

  // auto_shorter is deprecated and will be turned into
  // shortest=true costing option. maybe remove in v4?
  if (costing_str == "auto_shorter") {
    costing_str = "auto";
    rapidjson::SetValueByPointer(doc, "/costing", "auto");
    auto json_options = rapidjson::GetValueByPointer(doc, "/costing_options/auto_shorter");
    if (json_options) {
      rapidjson::SetValueByPointer(doc, "/costing_options/auto", *json_options);
    }
    rapidjson::SetValueByPointer(doc, "/costing_options/auto/shortest", true);
  }

  // auto_data_fix is deprecated and will be turned into
  // ignore all the things costing option. maybe remove in v4?
  if (costing_str == "auto_data_fix") {
    costing_str = "auto";
    rapidjson::SetValueByPointer(doc, "/costing", "auto");
    auto json_options = rapidjson::GetValueByPointer(doc, "/costing_options/auto_data_fix");
    if (json_options) {
      rapidjson::SetValueByPointer(doc, "/costing_options/auto", *json_options);
    }
    rapidjson::SetValueByPointer(doc, "/costing_options/auto/ignore_restrictions", true);
    rapidjson::SetValueByPointer(doc, "/costing_options/auto/ignore_oneways", true);
    rapidjson::SetValueByPointer(doc, "/costing_options/auto/ignore_access", true);
    rapidjson::SetValueByPointer(doc, "/costing_options/auto/ignore_closures", true);
  }

  // whatever our costing is, check to see if we are going to ignore_closures
  std::stringstream ss;
  ss << "/costing_options/" << costing_str << "/ignore_closures";
  auto ignore_closures = costing_str != "multimodal"
                             ? rapidjson::get_optional<bool>(doc, ss.str().c_str())
                             : boost::none;

  // parse map matching location input and encoded_polyline for height actions
  auto encoded_polyline = rapidjson::get_optional<std::string>(doc, "/encoded_polyline");
  if (encoded_polyline) {
    options.set_encoded_polyline(*encoded_polyline);

    // Set the precision to use when decoding the polyline. For height actions (only)
    // either polyline6 (default) or polyline5 are supported. All other actions only
    // support polyline6 inputs at this time.
    double precision = 1e-6;
    if (options.action() == Options::height) {
      precision = options.shape_format() == valhalla::polyline5 ? 1e-5 : 1e-6;
    }

    auto decoded = midgard::decode<std::vector<midgard::PointLL>>(*encoded_polyline, precision);
    for (const auto& ll : decoded) {
      auto* sll = options.mutable_shape()->Add();
      sll->mutable_ll()->set_lat(ll.lat());
      sll->mutable_ll()->set_lng(ll.lng());
      // set type to via by default
      sll->set_type(valhalla::Location::kVia);
    }
    // first and last always get type break
    if (options.shape_size()) {
      options.mutable_shape(0)->set_type(valhalla::Location::kBreak);
      options.mutable_shape(options.shape_size() - 1)->set_type(valhalla::Location::kBreak);
    }
    // add the date time
    add_date_to_locations(options, *options.mutable_shape());
  } // fall back from encoded polyline to array of locations
  else {
    parse_locations(doc, options, "shape", 134, ignore_closures);

    // if no shape then try 'trace'
    if (options.shape().size() == 0) {
      parse_locations(doc, options, "trace", 135, ignore_closures);
    }
  }

  // Begin time for timestamps when entered given durations/delta times (defaults to 0)
  auto t = rapidjson::get_optional<unsigned int>(doc, "/begin_time");
  double begin_time = 0.0;
  if (t) {
    begin_time = *t;
  }

  // Use durations (per shape point pair) to set time
  auto durations = rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, "/durations");
  if (durations) {
    // Make sure durations is sized appropriately
    if (options.shape_size() > 0 && durations->Size() != (unsigned int)options.shape_size() - 1) {
      throw valhalla_exception_t{136};
    }

    // Set time to begin_time at the first trace point.
    options.mutable_shape()->Mutable(0)->set_time(begin_time);

    // Iterate through the durations and add to elapsed time - set time on
    // successive trace points.
    double current_time = begin_time;
    int index = 1;
    for (const auto& dur : *durations) {
      auto duration = dur.GetDouble();
      current_time += duration;
      options.mutable_shape()->Mutable(index)->set_time(current_time);
      ++index;
    }
  }

  // Option to use timestamps when computing elapsed time for matched routes
  options.set_use_timestamps(
      rapidjson::get_optional<bool>(doc, "/use_timestamps").get_value_or(false));

  // Throw an error if use_timestamps is set to true but there are no timestamps in the
  // trace (or no durations present)
  if (options.use_timestamps()) {
    bool has_time = false;
    for (const auto& s : options.shape()) {
      if (s.has_time()) {
        has_time = true;
        break;
      }
    }
    if (!has_time) {
      throw valhalla_exception_t{159};
    }
  }

  // TODO: remove this?
  options.set_do_not_track(rapidjson::get_optional<bool>(doc, "/healthcheck").get_value_or(false));

  // Elevation service options
  options.set_range(rapidjson::get(doc, "/range", false));
  constexpr uint32_t MAX_HEIGHT_PRECISION = 2;
  auto height_precision = rapidjson::get_optional<unsigned int>(doc, "/height_precision");
  if (height_precision && *height_precision <= MAX_HEIGHT_PRECISION) {
    options.set_height_precision(*height_precision);
  }

  options.set_verbose(rapidjson::get(doc, "/verbose", false));

  // try the string directly, some strings are keywords so add an underscore
  Costing costing;
  if (valhalla::Costing_Enum_Parse(costing_str, &costing)) {
    options.set_costing(costing);
  } else {
    throw valhalla_exception_t{125, "'" + costing_str + "'"};
  }

  // Parse all of the costing options in their specified order
  sif::ParseCostingOptions(doc, "/costing_options", options);

  // parse any named costings for re-costing a given path
  auto recostings = rapidjson::get_child_optional(doc, "/recostings");
  if (recostings && recostings->IsArray()) {
    for (size_t i = 0; i < recostings->GetArray().Size(); ++i) {
      // parse the options
      std::string key = "/recostings/" + std::to_string(i);
      sif::ParseCostingOptions(doc, key, options.add_recostings());
      if (!options.recostings().rbegin()->has_name()) {
        throw valhalla_exception_t{127};
      }
    }
    // TODO: throw if not all names are unique?
  }

  // get the locations in there
  parse_locations(doc, options, "locations", 130, ignore_closures);

  // get the sources in there
  parse_locations(doc, options, "sources", 131, ignore_closures);

  // get the targets in there
  parse_locations(doc, options, "targets", 132, ignore_closures);

  // get the avoids in there
  // TODO: remove "avoid_locations/polygons" after some while
  if (doc.HasMember("avoid_locations"))
    parse_locations(doc, options, "avoid_locations", 133, ignore_closures);
  else
    parse_locations(doc, options, "exclude_locations", 133, ignore_closures);

  // get the avoid polygons in there
  auto rings_req =
      rapidjson::get_child_optional(doc, doc.HasMember("avoid_polygons") ? "/avoid_polygons"
                                                                         : "/exclude_polygons");
  if (rings_req) {
    auto* rings_pbf = options.mutable_exclude_polygons();
    try {
      for (const auto& req_poly : rings_req->GetArray()) {
        auto* ring = rings_pbf->Add();
        parse_ring(ring, req_poly);
      }
    } catch (...) { throw valhalla_exception_t{137}; }
  }

  // if not a time dependent route/mapmatch disable time dependent edge speed/flow data sources
  if (!options.has_date_time_type() && (options.shape_size() == 0 || options.shape(0).time() == -1)) {
    for (auto& costing : *options.mutable_costing_options()) {
      costing.set_flow_mask(
          static_cast<uint8_t>(costing.flow_mask()) &
          ~(valhalla::baldr::kPredictedFlowMask | valhalla::baldr::kCurrentFlowMask));
    }
  }

  // get some parameters
  auto resample_distance = rapidjson::get_optional<double>(doc, "/resample_distance");
  if (resample_distance) {
    options.set_resample_distance(*resample_distance);
  }

  // get the contours in there
  parse_contours(doc, options.mutable_contours());

  // if specified, get the polygons boolean in there
  auto polygons = rapidjson::get_optional<bool>(doc, "/polygons");
  if (polygons) {
    options.set_polygons(*polygons);
  }

  // if specified, get the denoise in there
  auto denoise = rapidjson::get_optional<float>(doc, "/denoise");
  if (denoise) {
    options.set_denoise(std::max(std::min(*denoise, 1.f), 0.f));
  }

  // if specified, get the generalize value in there
  auto generalize = rapidjson::get_optional<float>(doc, "/generalize");
  if (generalize) {
    options.set_generalize(*generalize);
  }

  // if specified, get the show_locations boolean in there
  auto show_locations = rapidjson::get_optional<bool>(doc, "/show_locations");
  if (show_locations) {
    options.set_show_locations(*show_locations);
  }

  // if specified, get the shape_match in there
  auto shape_match_str = rapidjson::get_optional<std::string>(doc, "/shape_match");
  ShapeMatch shape_match;
  if (shape_match_str) {
    if (valhalla::ShapeMatch_Enum_Parse(*shape_match_str, &shape_match)) {
      options.set_shape_match(shape_match);
    } else {
      throw valhalla_exception_t{445};
    }
  }

  // if specified, get the best_paths in there
  auto best_paths = rapidjson::get_optional<uint32_t>(doc, "/best_paths");
  if (best_paths) {
    options.set_best_paths(*best_paths);
  }

  // if specified, get the trace gps_accuracy value in there
  auto gps_accuracy = rapidjson::get_optional<float>(doc, "/trace_options/gps_accuracy");
  if (gps_accuracy) {
    options.set_gps_accuracy(*gps_accuracy);
  }

  // if specified, get the trace search_radius value in there
  auto search_radius = rapidjson::get_optional<float>(doc, "/trace_options/search_radius");
  if (search_radius) {
    options.set_search_radius(*search_radius);
  }

  // if specified, get the trace turn_penalty_factor value in there
  auto turn_penalty_factor =
      rapidjson::get_optional<float>(doc, "/trace_options/turn_penalty_factor");
  if (turn_penalty_factor) {
    options.set_turn_penalty_factor(*turn_penalty_factor);
  }

  // if specified, get the breakage_distance value in there
  auto breakage_distance = rapidjson::get_optional<float>(doc, "/trace_options/breakage_distance");
  if (breakage_distance) {
    options.set_breakage_distance(*breakage_distance);
  }

  // if specified, get the interpolation_distance value in there
  auto interpolation_distance =
      rapidjson::get_optional<float>(doc, "/trace_options/interpolation_distance");
  if (interpolation_distance) {
    options.set_interpolation_distance(*interpolation_distance);
  }

  // if specified, get the filter_action value in there
  auto filter_action_str = rapidjson::get_optional<std::string>(doc, "/filters/action");
  FilterAction filter_action;
  if (filter_action_str && valhalla::FilterAction_Enum_Parse(*filter_action_str, &filter_action)) {
    options.set_filter_action(filter_action);
  }

  // if specified, get the filter_attributes value in there
  auto filter_attributes_json =
      rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, "/filters/attributes");
  if (filter_attributes_json) {
    for (const auto& filter_attribute : *filter_attributes_json) {
      options.add_filter_attributes(filter_attribute.GetString());
    }
  }

  // how many alternates are desired, default to none and if its multi point its also none
  options.set_alternates(rapidjson::get<uint32_t>(doc, "/alternates", 0));
  if (options.locations_size() > 2)
    options.set_alternates(0);

  // whether to return guidance_views, default false
  auto guidance_views = rapidjson::get_optional<bool>(doc, "/guidance_views");
  if (guidance_views) {
    options.set_guidance_views(*guidance_views);
  }

  // whether to include roundabout_exit maneuvers, default true
  auto roundabout_exits = rapidjson::get_optional<bool>(doc, "/roundabout_exits");
  options.set_roundabout_exits(true);
  if (roundabout_exits) {
    options.set_roundabout_exits(*roundabout_exits);
  }

  // force these into the output so its obvious what we did to the user
  doc.AddMember({"language", allocator}, {options.language(), allocator}, allocator);
  doc.AddMember({"format", allocator},
                {valhalla::Options_Format_Enum_Name(options.format()), allocator}, allocator);
}

} // namespace

namespace valhalla {

valhalla_exception_t::valhalla_exception_t(unsigned code, const boost::optional<std::string>& extra)
    : std::runtime_error(""), code(code), extra(extra) {
  auto code_iter = error_codes.find(code);
  message = (code_iter == error_codes.cend() ? "" : code_iter->second);
  message += (extra ? ":" + *extra : "");
  auto http_code_iter = ERROR_TO_STATUS.find(code);
  http_code = (http_code_iter == ERROR_TO_STATUS.cend() ? 0 : http_code_iter->second);
  auto http_message_iter = HTTP_STATUS_CODES.find(http_code);
  http_message = (http_message_iter == HTTP_STATUS_CODES.cend() ? "" : http_message_iter->second);
}

void ParseApi(const std::string& request, Options::Action action, valhalla::Api& api) {
  api.Clear();
  auto document = from_string(request, valhalla_exception_t{100});
  api.mutable_options()->set_action(action);
  from_json(document, *api.mutable_options());
}

std::string jsonify_error(const valhalla_exception_t& exception, const Api& request) {
  // get the http status
  std::stringstream body;

  // overwrite with osrm error response
  if (request.options().format() == Options::osrm) {
    auto found = OSRM_ERRORS_CODES.find(exception.code);
    if (found == OSRM_ERRORS_CODES.cend()) {
      found = OSRM_ERRORS_CODES.find(199);
    }
    body << (request.options().has_jsonp() ? request.options().jsonp() + "(" : "") << found->second
         << (request.options().has_jsonp() ? ")" : "");
  } // valhalla error response
  else {
    // build up the json map
    auto json_error = baldr::json::map({});
    json_error->emplace("status", exception.http_message);
    json_error->emplace("status_code", static_cast<uint64_t>(exception.http_code));
    json_error->emplace("error", std::string(exception.message));
    json_error->emplace("error_code", static_cast<uint64_t>(exception.code));
    body << (request.options().has_jsonp() ? request.options().jsonp() + "(" : "") << *json_error
         << (request.options().has_jsonp() ? ")" : "");
  }
  return body.str();
}

#ifdef HAVE_HTTP
void ParseApi(const http_request_t& request, valhalla::Api& api) {
  api.Clear();

  // block all but get and post
  if (request.method != method_t::POST && request.method != method_t::GET) {
    throw valhalla_exception_t{101};
  };

  rapidjson::Document document;
  auto& allocator = document.GetAllocator();
  // parse the input
  const auto& json = request.query.find("json");
  if (json != request.query.end() && json->second.size() && json->second.front().size()) {
    document.Parse(json->second.front().c_str());
    // no json parameter, check the body
  } else if (!request.body.empty()) {
    document.Parse(request.body.c_str());
    // no json at all
  } else {
    document.SetObject();
  }
  // if parsing failed
  if (document.HasParseError()) {
    throw valhalla_exception_t{100};
  };

  // throw the query params into the rapidjson doc
  for (const auto& kv : request.query) {
    // skip json or empty entries
    if (kv.first == "json" || kv.first.empty() || kv.second.empty() || kv.second.front().empty()) {
      continue;
    }

    // turn single value entries into single key value
    if (kv.second.size() == 1) {
      document.AddMember({kv.first, allocator}, {kv.second.front(), allocator}, allocator);
      continue;
    }

    // make an array of values for this key
    rapidjson::Value array{rapidjson::kArrayType};
    for (const auto& value : kv.second) {
      array.PushBack({value, allocator}, allocator);
    }
    document.AddMember({kv.first, allocator}, array, allocator);
  }

  auto& options = *api.mutable_options();

  // set the action
  Options::Action action;
  if (!request.path.empty() && Options_Action_Enum_Parse(request.path.substr(1), &action)) {
    options.set_action(action);
  }

  // parse out the options
  from_json(document, options);
}

const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type ATTACHMENT{"Content-Disposition", "attachment; filename=route.gpx"};

worker_t::result_t jsonify_error(const valhalla_exception_t& exception,
                                 http_request_info_t& request_info,
                                 const Api& request) {
  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(exception.http_code, exception.http_message,
                           jsonify_error(exception, request),
                           headers_t{CORS, request.options().has_jsonp() ? worker::JS_MIME
                                                                         : worker::JSON_MIME});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());

  return result;
}

worker_t::result_t to_response(const std::string& data,
                               http_request_info_t& request_info,
                               const Api& request,
                               const worker::content_type& mime_type,
                               const bool as_attachment) {

  worker_t::result_t result{false, std::list<std::string>(), ""};
  if (request.options().has_jsonp()) {
    std::ostringstream stream;
    stream << request.options().jsonp() << '(';
    stream << data;
    stream << ')';

    headers_t headers{CORS, worker::JS_MIME};
    if (as_attachment)
      headers.insert(ATTACHMENT);

    http_response_t response(200, "OK", stream.str(), headers);
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
  } else {
    headers_t headers{CORS, mime_type};
    if (as_attachment)
      headers.insert(ATTACHMENT);
    http_response_t response(200, "OK", data, headers);
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
  }
  return result;
}

#endif

service_worker_t::service_worker_t() : interrupt(nullptr) {
}
service_worker_t::~service_worker_t() {
}
void service_worker_t::set_interrupt(const std::function<void()>* interrupt_function) {
  interrupt = interrupt_function;
}

} // namespace valhalla
