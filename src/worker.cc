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

const std::unordered_map<unsigned, unsigned> ERROR_TO_STATUS{
    {100, 400}, {101, 405}, {106, 404}, {107, 501},

    {110, 400}, {111, 400}, {112, 400}, {113, 400}, {114, 400},

    {120, 400}, {121, 400}, {122, 400}, {123, 400}, {124, 400}, {125, 400}, {126, 400},

    {130, 400}, {131, 400}, {132, 400}, {133, 400}, {136, 400},

    {140, 400}, {141, 501}, {142, 501},

    {150, 400}, {151, 400}, {152, 400}, {153, 400}, {154, 400}, {155, 400}, {156, 400},
    {157, 400}, {158, 400}, {159, 400},

    {160, 400}, {161, 400}, {162, 400}, {163, 400}, {164, 400},

    {170, 400}, {171, 400}, {172, 400},

    {199, 400},

    {200, 500}, {201, 500}, {202, 500},

    {210, 400}, {211, 400}, {212, 400}, {213, 400},

    {220, 400},

    {230, 400}, {231, 400}, {232, 400},

    {299, 400},

    {304, 404}, {305, 501},

    {310, 400}, {311, 400}, {312, 400}, {313, 400}, {314, 400},

    {399, 400},

    {400, 400}, {401, 500},

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

    {140,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {141,
     R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
    {142,
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

    {170, R"({"code":"NoRoute","message":"Impossible route between points"})"},
    {171,
     R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})"},

    {199, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

    // odin project 2xx
    {200, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {201, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
    {202, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

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
    {401, R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

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
  d.Parse(json.c_str());
  if (d.HasParseError()) {
    throw e;
  }
  return d;
}

void add_date_to_locations(Options& options,
                           google::protobuf::RepeatedPtrField<valhalla::Location>& locations) {
  if (options.has_date_time() && locations.size()) {
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
      default:
        break;
    }
  }
}

void parse_locations(const rapidjson::Document& doc,
                     Options& options,
                     const std::string& node,
                     unsigned location_parse_error_code,
                     bool track) {

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
  } else if (node == "avoid_locations") {
    locations = options.mutable_avoid_locations();
  } else {
    return;
  }

  bool had_date_time = false;
  auto request_locations =
      rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, std::string("/" + node).c_str());
  if (request_locations) {
    for (const auto& r_loc : *request_locations) {
      try {
        auto* location = locations->Add();
        location->set_original_index(locations->size() - 1);

        auto lat = rapidjson::get_optional<float>(r_loc, "/lat");
        if (!lat) {
          throw std::runtime_error{"lat is missing"};
        };

        if (*lat < -90.0f || *lat > 90.0f) {
          throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
        }

        auto lon = rapidjson::get_optional<float>(r_loc, "/lon");
        if (!lon) {
          throw std::runtime_error{"lon is missing"};
        };

        lon = midgard::circular_range_clamp<float>(*lon, -180, 180);
        location->mutable_ll()->set_lat(*lat);
        location->mutable_ll()->set_lng(*lon);

        auto stop_type_json = rapidjson::get_optional<std::string>(r_loc, "/type");
        if (stop_type_json) {
          if (*stop_type_json == std::string("through"))
            location->set_type(valhalla::Location::kThrough);
          else if (*stop_type_json == std::string("via"))
            location->set_type(valhalla::Location::kVia);
          else if (*stop_type_json == std::string("break_through"))
            location->set_type(valhalla::Location::kBreakThrough);
        } // for map matching the default type is a through
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
        auto search_cutoff = rapidjson::get_optional<unsigned int>(r_loc, "/search_cutoff");
        if (search_cutoff) {
          location->set_search_cutoff(*search_cutoff);
        }
        auto street_side_tolerance =
            rapidjson::get_optional<unsigned int>(r_loc, "/street_side_tolerance");
        if (street_side_tolerance) {
          location->set_street_side_tolerance(*street_side_tolerance);
        }
      } catch (...) { throw valhalla_exception_t{location_parse_error_code}; }
    }

    // first and last locations get the default type of break no matter what
    if (locations->size()) {
      locations->Mutable(0)->set_type(valhalla::Location::kBreak);
      locations->Mutable(locations->size() - 1)->set_type(valhalla::Location::kBreak);
    }
    if (track) {
      midgard::logging::Log(node + "_count::" + std::to_string(request_locations->Size()),
                            " [ANALYTICS] ");
    }

    // push the date time information down into the locations
    if (!had_date_time) {
      add_date_to_locations(options, *locations);
    }
  }
}

void parse_contours(const rapidjson::Document& doc,
                    google::protobuf::RepeatedPtrField<Contour>* contours) {

  // make sure the isoline definitions are valid
  auto json_contours = rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, "/contours");
  if (json_contours) {
    float prev = 0.f;
    const float NO_TIME = -1.f;
    for (const auto& json_contour : *json_contours) {
      // Grab contour time and validate that it is increasing
      const float c = rapidjson::get_optional<float>(json_contour, "/time").get_value_or(NO_TIME);
      if (c < prev || c == NO_TIME) {
        throw valhalla_exception_t{111};
      }

      // Add new contour object to list
      auto* contour = contours->Add();
      // Set contour time
      contour->set_time(c);

      // If specified, grab and set contour color
      auto color = rapidjson::get_optional<std::string>(json_contour, "/color");
      if (color) {
        contour->set_color(*color);
      }
      prev = c;
    }
  }
}

void from_json(rapidjson::Document& doc, Options& options) {
  bool track = !options.has_do_not_track() || !options.do_not_track();

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
  auto date_time_value = rapidjson::get_optional<std::string>(doc, "/date_time/value");
  if (date_time_type && Options::DateTimeType_IsValid(*date_time_type)) {
    // check the type is in bounds
    auto const v = static_cast<Options::DateTimeType>(*date_time_type);
    if (v >= Options::DateTimeType_ARRAYSIZE)
      throw valhalla_exception_t{163};
    options.set_date_time_type(static_cast<Options::DateTimeType>(v));
    // check the value exists for depart at and arrive by
    if (!date_time_value) {
      if (v == Options::depart_at)
        throw valhalla_exception_t{160};
      else if (v == Options::arrive_by)
        throw valhalla_exception_t{161};
    }
    // check the value is sane for depart at and arrive by
    if (v != Options::current && !baldr::DateTime::is_iso_valid(*date_time_value))
      throw valhalla_exception_t{162};
    if (v != Options::current)
      options.set_date_time(*date_time_value);
    else
      options.set_date_time("current");
  } // not specified but you want transit, then we default to current
  else if (options.has_costing() &&
           (options.costing() == multimodal || options.costing() == transit)) {
    options.set_date_time_type(Options::current);
    options.set_date_time("current");
  }

  // parse map matching location input
  auto encoded_polyline = rapidjson::get_optional<std::string>(doc, "/encoded_polyline");
  if (encoded_polyline) {
    options.set_encoded_polyline(*encoded_polyline);
    auto decoded = midgard::decode<std::vector<midgard::PointLL>>(*encoded_polyline);
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
    parse_locations(doc, options, "shape", 134, false);

    // if no shape then try 'trace'
    if (options.shape().size() == 0) {
      parse_locations(doc, options, "trace", 135, false);
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

  // Set the output precision for shape/geometry (polyline encoding). Defaults to polyline6
  // TODO - is this just for OSRM compatibility?
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

  // TODO: remove this?
  options.set_do_not_track(rapidjson::get_optional<bool>(doc, "/healthcheck").get_value_or(false));

  options.set_range(rapidjson::get(doc, "/range", false));

  options.set_verbose(rapidjson::get(doc, "/verbose", false));

  // costing
  auto costing_str = rapidjson::get_optional<std::string>(doc, "/costing");
  if (costing_str) {
    // try the string directly, some strings are keywords so add an underscore
    Costing costing;
    if (valhalla::Costing_Enum_Parse(*costing_str, &costing)) {
      options.set_costing(costing);
    } else {
      throw valhalla_exception_t{125, "'" + *costing_str + "'"};
    }
  }

  // if specified, get the costing options in there
  // the order of costing must reflect the enum order
  for (const auto& costing : {auto_, auto_shorter, bicycle, bus, hov, motor_scooter, multimodal,
                              pedestrian, transit, truck, motorcycle, auto_data_fix, taxi}) {
    // Create the costing string
    auto costing_str = valhalla::Costing_Enum_Name(costing);
    // Create the costing options key
    const auto costing_options_key = "/costing_options/" + costing_str;

    switch (costing) {
      case auto_: {
        sif::ParseAutoCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case auto_shorter: {
        sif::ParseAutoShorterCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case bicycle: {
        sif::ParseBicycleCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case bus: {
        sif::ParseBusCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case hov: {
        sif::ParseHOVCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case taxi: {
        sif::ParseTaxiCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case motor_scooter: {
        sif::ParseMotorScooterCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case multimodal: {
        options.add_costing_options(); // Nothing to parse for this one
        break;
      }
      case pedestrian: {
        sif::ParsePedestrianCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case transit: {
        sif::ParseTransitCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case truck: {
        sif::ParseTruckCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case motorcycle: {
        sif::ParseMotorcycleCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
      case auto_data_fix: {
        sif::ParseAutoDataFixCostOptions(doc, costing_options_key, options.add_costing_options());
        break;
      }
    }
  }

  // get the locations in there
  parse_locations(doc, options, "locations", 130, track);

  // get the sources in there
  parse_locations(doc, options, "sources", 131, track);

  // get the targets in there
  parse_locations(doc, options, "targets", 132, track);

  // get the avoids in there
  parse_locations(doc, options, "avoid_locations", 133, track);

  // if not a time dependent route/mapmatch disable time dependent edge speed/flow data sources
  // TODO: this is because bidirectional a* defaults to middle of the day time for speed lookup
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

  // force these into the output so its obvious what we did to the user
  doc.AddMember({"language", allocator}, {options.language(), allocator}, allocator);
  doc.AddMember({"format", allocator},
                {valhalla::Options_Format_Enum_Name(options.format()), allocator}, allocator);
}

} // namespace

namespace valhalla {

bool Options_Action_Enum_Parse(const std::string& action, Options::Action* a) {
  static const std::unordered_map<std::string, Options::Action> actions{
      {"route", Options::route},
      {"locate", Options::locate},
      {"sources_to_targets", Options::sources_to_targets},
      {"optimized_route", Options::optimized_route},
      {"isochrone", Options::isochrone},
      {"trace_route", Options::trace_route},
      {"trace_attributes", Options::trace_attributes},
      {"height", Options::height},
      {"transit_available", Options::transit_available},
      {"expansion", Options::expansion},
  };
  auto i = actions.find(action);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

const std::string& Options_Action_Enum_Name(const Options::Action action) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> actions{
      {Options::route, "route"},
      {Options::locate, "locate"},
      {Options::sources_to_targets, "sources_to_targets"},
      {Options::optimized_route, "optimized_route"},
      {Options::isochrone, "isochrone"},
      {Options::trace_route, "trace_route"},
      {Options::trace_attributes, "trace_attributes"},
      {Options::height, "height"},
      {Options::transit_available, "transit_available"},
      {Options::expansion, "expansion"},
  };
  auto i = actions.find(action);
  return i == actions.cend() ? empty : i->second;
}

bool Costing_Enum_Parse(const std::string& costing, Costing* c) {
  static const std::unordered_map<std::string, Costing> costings{
      {"auto", Costing::auto_},
      {"auto_shorter", Costing::auto_shorter},
      {"bicycle", Costing::bicycle},
      {"bus", Costing::bus},
      {"hov", Costing::hov},
      {"taxi", Costing::taxi},
      {"motor_scooter", Costing::motor_scooter},
      {"multimodal", Costing::multimodal},
      {"pedestrian", Costing::pedestrian},
      {"transit", Costing::transit},
      {"truck", Costing::truck},
      {"motorcycle", Costing::motorcycle},
      {"auto_data_fix", Costing::auto_data_fix},
  };
  auto i = costings.find(costing);
  if (i == costings.cend())
    return false;
  *c = i->second;
  return true;
}

const std::string& Costing_Enum_Name(const Costing costing) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> costings{
      {Costing::auto_, "auto"},
      {Costing::auto_shorter, "auto_shorter"},
      {Costing::bicycle, "bicycle"},
      {Costing::bus, "bus"},
      {Costing::hov, "hov"},
      {Costing::taxi, "taxi"},
      {Costing::motor_scooter, "motor_scooter"},
      {Costing::multimodal, "multimodal"},
      {Costing::pedestrian, "pedestrian"},
      {Costing::transit, "transit"},
      {Costing::truck, "truck"},
      {Costing::motorcycle, "motorcycle"},
      {Costing::auto_data_fix, "auto_data_fix"},
  };
  auto i = costings.find(costing);
  return i == costings.cend() ? empty : i->second;
}

bool ShapeMatch_Enum_Parse(const std::string& match, ShapeMatch* s) {
  static const std::unordered_map<std::string, ShapeMatch> matches{
      {"edge_walk", ShapeMatch::edge_walk},
      {"map_snap", ShapeMatch::map_snap},
      {"walk_or_snap", ShapeMatch::walk_or_snap},
  };
  auto i = matches.find(match);
  if (i == matches.cend())
    return false;
  *s = i->second;
  return true;
}

const std::string& ShapeMatch_Enum_Name(const ShapeMatch match) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> matches{
      {ShapeMatch::edge_walk, "edge_walk"},
      {ShapeMatch::map_snap, "map_snap"},
      {ShapeMatch::walk_or_snap, "walk_or_snap"},
  };
  auto i = matches.find(match);
  return i == matches.cend() ? empty : i->second;
}

bool Options_Format_Enum_Parse(const std::string& format, Options::Format* f) {
  static const std::unordered_map<std::string, Options::Format> formats{
      {"json", Options::json},
      {"gpx", Options::gpx},
      {"osrm", Options::osrm},
  };
  auto i = formats.find(format);
  if (i == formats.cend())
    return false;
  *f = i->second;
  return true;
}

const std::string& Options_Format_Enum_Name(const Options::Format match) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> formats{
      {Options::json, "json"},
      {Options::gpx, "gpx"},
      {Options::osrm, "osrm"},
  };
  auto i = formats.find(match);
  return i == formats.cend() ? empty : i->second;
}

const std::string& Options_Units_Enum_Name(const Options::Units unit) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> units{
      {Options::kilometers, "kilometers"},
      {Options::miles, "miles"},
  };
  auto i = units.find(unit);
  return i == units.cend() ? empty : i->second;
}

bool FilterAction_Enum_Parse(const std::string& action, FilterAction* a) {
  static const std::unordered_map<std::string, FilterAction> actions{
      {"exclude", FilterAction::exclude},
      {"include", FilterAction::include},
  };
  auto i = actions.find(action);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

const std::string& FilterAction_Enum_Name(const FilterAction action) {
  static const std::string empty;
  static const std::unordered_map<int, std::string> actions{
      {FilterAction::exclude, "exclude"},
      {FilterAction::include, "include"},
  };
  auto i = actions.find(action);
  return i == actions.cend() ? empty : i->second;
}

bool DirectionsType_Enum_Parse(const std::string& dtype, DirectionsType* t) {
  static const std::unordered_map<std::string, DirectionsType> types{
      {"none", DirectionsType::none},
      {"maneuvers", DirectionsType::maneuvers},
      {"instructions", DirectionsType::instructions},
  };
  auto i = types.find(dtype);
  if (i == types.cend())
    return false;
  *t = i->second;
  return true;
}

bool PreferredSide_Enum_Parse(const std::string& pside, valhalla::Location::PreferredSide* p) {
  static const std::unordered_map<std::string, valhalla::Location::PreferredSide> types{
      {"either", valhalla::Location::either},
      {"same", valhalla::Location::same},
      {"opposite", valhalla::Location::opposite},
  };
  auto i = types.find(pside);
  if (i == types.cend())
    return false;
  *p = i->second;
  return true;
}

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

  // disable analytics
  auto do_not_track = request.headers.find("DNT");
  options.set_do_not_track(options.do_not_track() ||
                           (do_not_track != request.headers.cend() && do_not_track->second == "1"));

  // parse out the options
  from_json(document, options);
}

const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
const headers_t::value_type XML_MIME{"Content-type", "text/xml;charset=utf-8"};
const headers_t::value_type GPX_MIME{"Content-type", "application/gpx+xml;charset=utf-8"};
const headers_t::value_type ATTACHMENT{"Content-Disposition", "attachment; filename=route.gpx"};

worker_t::result_t jsonify_error(const valhalla_exception_t& exception,
                                 http_request_info_t& request_info,
                                 const Api& request) {
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

  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(exception.http_code, exception.http_message, body.str(),
                           headers_t{CORS, request.options().has_jsonp() ? JS_MIME : JSON_MIME});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());

  return result;
}

worker_t::result_t to_response(const baldr::json::ArrayPtr& array,
                               http_request_info_t& request_info,
                               const Api& request) {
  std::ostringstream stream;
  // jsonp callback if need be
  if (request.options().has_jsonp()) {
    stream << request.options().jsonp() << '(';
  }
  stream << *array;
  if (request.options().has_jsonp()) {
    stream << ')';
  }

  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(200, "OK", stream.str(),
                           headers_t{CORS, request.options().has_jsonp() ? JS_MIME : JSON_MIME});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());
  return result;
}

worker_t::result_t
to_response(const baldr::json::MapPtr& map, http_request_info_t& request_info, const Api& request) {
  std::ostringstream stream;
  // jsonp callback if need be
  if (request.options().has_jsonp()) {
    stream << request.options().jsonp() << '(';
  }
  stream << *map;
  if (request.options().has_jsonp()) {
    stream << ')';
  }

  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(200, "OK", stream.str(),
                           headers_t{CORS, request.options().has_jsonp() ? JS_MIME : JSON_MIME});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());
  return result;
}

worker_t::result_t
to_response_json(const std::string& json, http_request_info_t& request_info, const Api& request) {
  std::ostringstream stream;
  // jsonp callback if need be
  if (request.options().has_jsonp()) {
    stream << request.options().jsonp() << '(';
  }
  stream << json;
  if (request.options().has_jsonp()) {
    stream << ')';
  }

  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(200, "OK", stream.str(),
                           headers_t{CORS, request.options().has_jsonp() ? JS_MIME : JSON_MIME});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());
  return result;
}

worker_t::result_t
to_response_xml(const std::string& xml, http_request_info_t& request_info, const Api&) {
  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(200, "OK", xml, headers_t{CORS, GPX_MIME, ATTACHMENT});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());
  return result;
}

#endif

service_worker_t::service_worker_t() : interrupt(nullptr) {
}
service_worker_t::~service_worker_t() {
}
void service_worker_t::set_interrupt(const std::function<void()>& interrupt_function) {
  interrupt = &interrupt_function;
}

} // namespace valhalla
