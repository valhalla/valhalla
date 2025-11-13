#include "loki/worker.h"
#include "baldr/attributes_controller.h"
#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "exceptions.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "odin/util.h"
#include "proto_conversions.h"
#include "sif/hierarchylimits.h"
#include "worker.h"

#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cpp-statsd-client/StatsdClient.hpp>

#include <sstream>

using namespace valhalla;
#ifdef ENABLE_SERVICES
using namespace prime_server;

// returns the correct MIME type for a given format
static const worker::content_type& fmt_to_mime(const Options::Format& fmt) noexcept {
  switch (fmt) {
    case Options::gpx:
      return worker::GPX_MIME;
    case Options::pbf:
      return worker::PBF_MIME;
    case Options::geotiff:
      return worker::TIFF_MIME;
    default:
      return worker::JSON_MIME;
  }
};
#endif // ENABLE_SERVICES

namespace {

bool is_format_supported(Options::Action action, Options::Format format) {
  constexpr uint16_t kFormatActionSupport[] = {
      // json
      0xFFFF, // all actions support json
      // gpx
      (1 << Options::route) | (1 << Options::optimized_route) | (1 << Options::trace_route),
      // osrm
      (1 << Options::route) | (1 << Options::optimized_route) | (1 << Options::trace_route) |
          (1 << Options::trace_attributes) | (1 << Options::locate) | (1 << Options::status) |
          (1 << Options::sources_to_targets) | (1 << Options::expansion),
      // pbf
      (1 << Options::route) | (1 << Options::optimized_route) | (1 << Options::trace_route) |
          (1 << Options::centroid) | (1 << Options::trace_attributes) | (1 << Options::status) |
          (1 << Options::sources_to_targets) | (1 << Options::isochrone) | (1 << Options::expansion),
  // geotiff
#ifdef ENABLE_GEOTIFF
      (1 << Options::isochrone),
#else
      0,
#endif
  };
  static_assert(std::size(kFormatActionSupport) == Options::Format_ARRAYSIZE,
                "Please update format_action array to match Options::Action_ARRAYSIZE");

  if (Options::Action_IsValid(action) && Options::Format_IsValid(format)) {
    return (kFormatActionSupport[format] & (1 << action)) != 0;
  } else {
    return false;
  }
}

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

bool add_date_to_locations(Options& options,
                           google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                           const std::string& node) {
  if (options.has_date_time_case() && !locations.empty()) {
    auto dt = options.date_time_type();
    if (options.action() != Options::sources_to_targets) {
      switch (dt) {
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
    } else {
      if (node == (dt == Options::arrive_by ? "targets" : "sources")) {
        for (auto& loc : locations) {
          loc.set_date_time(dt == Options::current ? "current" : options.date_time());
        }
      }
    }
  }

  return std::find_if(locations.begin(), locations.end(), [](const valhalla::Location& loc) {
           return !loc.date_time().empty();
         }) != locations.end();
}

// Parses JSON rings of the form [[lon1, lat1], [lon2, lat2], ...]] and operates on
// PBF objects of the sort "repeated LatLng". Invalid rings will be corrected during search operation.
template <typename ring_pbf_t>
void parse_ring(ring_pbf_t* ring, const rapidjson::Value& coord_array) {

  // for protobuf we just validate what is there
  if (ring->coords_size()) {
    if (ring->coords_size() < 2)
      throw std::runtime_error("Polygon coordinates must consist of [Lon, Lat] arrays.");
    for (auto& coord : *ring->mutable_coords()) {
      if (!coord.has_lat_case() || !coord.has_lng_case())
        throw std::runtime_error("Polygon coordinates must consist of [Lon, Lat] arrays.");

      coord.set_lng(midgard::circular_range_clamp<double>(coord.lng(), -180, 180));
      if (coord.lat() < -90.0 || coord.lat() > 90.0) {
        throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
      }
    }
    return;
  }

  // for json we need to do some parsing
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
}

void parse_location(valhalla::Location* location,
                    const rapidjson::Value& r_loc,
                    Api& request,
                    const boost::optional<bool>& ignore_closures,
                    bool is_last_loc) {
  auto lat = rapidjson::get_optional<double>(r_loc, "/lat");
  if (location->has_ll() && location->ll().has_lat_case()) {
    lat = location->ll().lat();
  }
  if (!lat) {
    throw std::runtime_error{"lat is missing"};
  };

  if (*lat < -90.0 || *lat > 90.0) {
    throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
  }

  auto lon = rapidjson::get_optional<double>(r_loc, "/lon");
  if (location->has_ll() && location->ll().has_lng_case()) {
    lon = location->ll().lng();
  }
  if (!lon) {
    throw std::runtime_error{"lon is missing"};
  };

  lon = midgard::circular_range_clamp<double>(*lon, -180, 180);
  location->mutable_ll()->set_lat(*lat);
  location->mutable_ll()->set_lng(*lon);

  // trace attributes does not support legs or breaks at discontinuities
  auto stop_type_json = rapidjson::get_optional<std::string>(r_loc, "/type");
  if (request.options().action() == Options::trace_attributes) {
    location->set_type(valhalla::Location::kVia);
  } // other actions let you specify whatever type of stop you want
  else if (stop_type_json) {
    Location::Type type = Location::kBreak;
    Location_Type_Enum_Parse(*stop_type_json, &type);
    location->set_type(type);
  } // and if you didnt set it it defaulted to break which is not the default for trace_route
  else if (request.options().action() == Options::trace_route && !location->has_time_case()) {
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
  auto date_time = rapidjson::get_optional<std::string>(r_loc, "/date_time");
  if (date_time) {
    location->set_date_time(*date_time);
  }
  auto heading = rapidjson::get_optional<int>(r_loc, "/heading");
  if (heading) {
    location->set_heading(*heading);
  }
  auto heading_tolerance = rapidjson::get_optional<int>(r_loc, "/heading_tolerance");
  if (heading_tolerance) {
    location->set_heading_tolerance(*heading_tolerance);
  }
  auto preferred_layer = rapidjson::get_optional<int>(r_loc, "/preferred_layer");
  if (preferred_layer) {
    location->set_preferred_layer(*preferred_layer);
  }
  auto node_snap_tolerance = rapidjson::get_optional<float>(r_loc, "/node_snap_tolerance");
  if (node_snap_tolerance) {
    location->set_node_snap_tolerance(*node_snap_tolerance);
  }
  auto minimum_reachability = rapidjson::get_optional<unsigned int>(r_loc, "/minimum_reachability");
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
  auto time =
      rapidjson::get<double>(r_loc, "/time", location->has_time_case() ? location->time() : -1);
  location->set_time(time);
  auto rank_candidates =
      rapidjson::get<bool>(r_loc, "/rank_candidates", !location->skip_ranking_candidates());
  location->set_skip_ranking_candidates(!rank_candidates);
  auto preferred_side = rapidjson::get_optional<std::string>(r_loc, "/preferred_side");
  valhalla::Location::PreferredSide side;
  if (preferred_side && PreferredSide_Enum_Parse(*preferred_side, &side)) {
    location->set_preferred_side(side);
  }
  lat = rapidjson::get_optional<double>(r_loc, "/display_lat");
  if (location->has_display_ll() && location->display_ll().has_lat_case())
    lat = location->display_ll().lat();
  lon = rapidjson::get_optional<double>(r_loc, "/display_lon");
  if (location->has_display_ll() && location->display_ll().has_lng_case())
    lon = location->display_ll().lng();
  if (lat && lon && *lat >= -90.0 && *lat <= 90.0) {
    lon = midgard::circular_range_clamp<double>(*lon, -180, 180);
    location->mutable_display_ll()->set_lat(*lat);
    location->mutable_display_ll()->set_lng(*lon);
  } else
    location->clear_display_ll();
  auto search_cutoff = rapidjson::get_optional<unsigned int>(r_loc, "/search_cutoff");
  if (search_cutoff) {
    location->set_search_cutoff(*search_cutoff);
  }
  auto street_side_tolerance = rapidjson::get_optional<unsigned int>(r_loc, "/street_side_tolerance");
  if (street_side_tolerance) {
    location->set_street_side_tolerance(*street_side_tolerance);
  }
  auto street_side_max_distance =
      rapidjson::get_optional<unsigned int>(r_loc, "/street_side_max_distance");
  if (street_side_max_distance) {
    location->set_street_side_max_distance(*street_side_max_distance);
  }
  auto street_side_cutoff = rapidjson::get_optional<std::string>(r_loc, "/street_side_cutoff");
  if (street_side_cutoff) {
    valhalla::RoadClass cutoff_street_side;
    if (RoadClass_Enum_Parse(*street_side_cutoff, &cutoff_street_side)) {
      location->set_street_side_cutoff(cutoff_street_side);
    }
  }

  boost::optional<bool> exclude_closures;
  // is it json?
  auto search_filter = rapidjson::get_child_optional(r_loc, "/search_filter");
  if (search_filter) {
    // search_filter.min_road_class
    auto min_road_class =
        rapidjson::get<std::string>(*search_filter, "/min_road_class", "service_other");
    valhalla::RoadClass min_rc;
    if (RoadClass_Enum_Parse(min_road_class, &min_rc)) {
      location->mutable_search_filter()->set_min_road_class(min_rc);
    }
    // search_filter.max_road_class
    auto max_road_class = rapidjson::get<std::string>(*search_filter, "/max_road_class", "motorway");
    valhalla::RoadClass max_rc;
    if (RoadClass_Enum_Parse(max_road_class, &max_rc)) {
      location->mutable_search_filter()->set_max_road_class(max_rc);
    }
    // search_filter.exclude_tunnel
    location->mutable_search_filter()->set_exclude_tunnel(
        rapidjson::get<bool>(*search_filter, "/exclude_tunnel", false));
    // search_filter.exclude_bridge
    location->mutable_search_filter()->set_exclude_bridge(
        rapidjson::get<bool>(*search_filter, "/exclude_bridge", false));
    // search_filter.exclude_toll
    location->mutable_search_filter()->set_exclude_toll(
        rapidjson::get<bool>(*search_filter, "/exclude_toll", false));
    // search_filter.exclude_ramp
    location->mutable_search_filter()->set_exclude_ramp(
        rapidjson::get<bool>(*search_filter, "/exclude_ramp", false));
    // search_filter.exclude_ferry
    location->mutable_search_filter()->set_exclude_ferry(
        rapidjson::get<bool>(*search_filter, "/exclude_ferry", false));
    location->mutable_search_filter()->set_level(
        rapidjson::get<float>(*search_filter, "/level", baldr::kMaxLevel));
    // search_filter.exclude_closures
    exclude_closures = rapidjson::get_optional<bool>(*search_filter, "/exclude_closures");
  } // or is it pbf
  else if (location->has_search_filter()) {
    if (location->search_filter().has_min_road_class_case() &&
        !RoadClass_IsValid(location->search_filter().min_road_class()))
      location->mutable_search_filter()->clear_min_road_class();
    if (location->search_filter().has_max_road_class_case() &&
        !RoadClass_IsValid(location->search_filter().max_road_class()))
      location->mutable_search_filter()->clear_max_road_class();
    if (location->search_filter().has_exclude_closures_case())
      exclude_closures = location->search_filter().exclude_closures();
  }

  // if you specified both of these they may contradict so we throw up our hands
  if (ignore_closures && exclude_closures) {
    throw valhalla_exception_t{143};
  }
  // do we actually want to filter closures on THIS location
  // NOTE: that ignore_closures takes precedence
  location->mutable_search_filter()->set_exclude_closures(
      ignore_closures ? !(*ignore_closures) : (exclude_closures ? *exclude_closures : true));
  if (!location->search_filter().has_min_road_class_case()) {
    location->mutable_search_filter()->set_min_road_class(valhalla::kServiceOther);
  }
  if (!location->search_filter().has_max_road_class_case()) {
    location->mutable_search_filter()->set_max_road_class(valhalla::kMotorway);
  }
  if (!location->search_filter().has_level_case())
    location->mutable_search_filter()->set_level(baldr::kMaxLevel);

  float waiting_secs = rapidjson::get<float>(r_loc, "/waiting", 0.f);
  switch (location->type()) {
    case Location_Type_kBreak:
    case Location_Type_kBreakThrough:
      // set waiting_time to 0 on origin/destination
      {
        auto loc_idx = location->correlation().original_index();
        // TODO: waiting time can be less than 0
        location->set_waiting_secs(loc_idx == 0 || is_last_loc || waiting_secs < 0.f ? 0.f
                                                                                     : waiting_secs);
        break;
      }
    default:
      if (waiting_secs)
        add_warning(request, 203);
  }
}

/**
 * Parses the locations and sets some defaults
 *
 * @param doc                       The JSON body
 * @param options                   The request options to be filled in
 * @param node                      The type of locations passed
 * @param location_parse_error_code The code raised if this method throws
 * @param ignore_closures           Whether to ignore closures
 * @param had_date_time             Gets set to true if any location had a date_time string
 */
void parse_locations(const rapidjson::Document& doc,
                     Api& request,
                     const std::string& node,
                     unsigned location_parse_error_code,
                     const boost::optional<bool>& ignore_closures,
                     bool& had_date_time) {
  auto& options = *request.mutable_options();

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

  bool filter_closures = true;
  bool loc_had_time = false;
  try {
    // should we parse json?
    auto request_locations =
        rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, std::string("/" + node).c_str());
    if (request_locations) {
      int last_loc_idx = static_cast<int>(request_locations->Size()) - 1;
      for (const auto& r_loc : *request_locations) {
        auto* loc = locations->Add();
        auto loc_idx = locations->size() - 1;
        loc->mutable_correlation()->set_original_index(loc_idx);
        parse_location(loc, r_loc, request, ignore_closures, loc_idx == last_loc_idx);
        loc_had_time = loc_had_time || !loc->date_time().empty();
        // turn off filtering closures when any locations search filter allows closures
        filter_closures = filter_closures && loc->search_filter().exclude_closures();
      }
    } // maybe its deserialized pbf
    else if (!locations->empty()) {
      uint32_t i = 0;
      uint32_t locs_amount = locations->size() - 1;
      for (auto& loc : *locations) {
        bool is_last_edge = i == locs_amount;
        loc.mutable_correlation()->set_original_index(i++);
        parse_location(&loc, {}, request, ignore_closures, is_last_edge);
        loc_had_time = loc_had_time || !loc.date_time().empty();
        // turn off filtering closures when any locations search filter allows closures
        filter_closures = filter_closures && loc.search_filter().exclude_closures();
      }
    }

    if (locations->empty())
      return;

    // first and last locations get the default type of break no matter what
    locations->Mutable(0)->set_type(valhalla::Location::kBreak);
    locations->Mutable(locations->size() - 1)->set_type(valhalla::Location::kBreak);

    // push the date time information down into the locations
    if (!loc_had_time) {
      had_date_time = add_date_to_locations(options, *locations, node) || had_date_time;
    }
    had_date_time = loc_had_time || had_date_time;

    // If any of the locations had search_filter.exclude_closures set to false,
    // we tell the costing to let all closed roads through, so that we can do
    // a secondary per-location filtering using loki's search_filter
    // functionality. Otherwise we default to skipping closed roads
    for (auto& costing : *options.mutable_costings()) {
      costing.second.set_filter_closures(filter_closures);
    }
  }
  // Forward valhalla_exception_t types as-is, since they contain a more specific error message
  catch (const valhalla_exception_t& e) {
    throw e;
  } // generic exceptions and other stuff get a generic message
  catch (...) {
    throw valhalla_exception_t{location_parse_error_code};
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

  for (const auto& c : *contours) {
    // You need at least something
    if (!c.has_time_case() && !c.has_distance_case()) {
      throw valhalla_exception_t{111};
    }
  }
}

// parse all costings needed to fulfill the request, including recostings
void parse_recostings(const rapidjson::Document& doc,
                      const std::string& key,
                      valhalla::Options& options) {
  // make sure we only have unique recosting names in the end
  std::unordered_set<std::string> names;
  auto check_name = [&names](const valhalla::Costing& recosting) -> void {
    if (!recosting.has_name_case()) {
      throw valhalla_exception_t{127};
    } else if (!names.insert(recosting.name()).second) {
      throw valhalla_exception_t{128};
    }
  };

  // look either in JSON & PBF
  auto recostings = rapidjson::get_child_optional(doc, "/recostings");
  if (recostings && recostings->IsArray()) {
    names.reserve(recostings->GetArray().Size());
    for (size_t i = 0; i < recostings->GetArray().Size(); ++i) {
      // parse the options
      std::string key = "/recostings/" + std::to_string(i);
      sif::ParseCosting(doc, key, options.add_recostings());
      check_name(*options.recostings().rbegin());
    }
  } else if (options.recostings().size()) {
    for (auto& recosting : *options.mutable_recostings()) {
      check_name(recosting);
      sif::ParseCosting(doc, key, &recosting, recosting.type());
    }
  }
}

/**
 * This function takes a json document and parses it into an options (request pbf) object.
 * The implementation is such that if you passed an already filled out options object the
 * function will still validate it and set the defaults, but if json is provided it will
 * override anything this is in the options object
 * @param doc      the rapidjson request doc
 * @param action   which request action will be performed
 * @param options  the options to fill out or validate if they are already filled out
 */
void from_json(rapidjson::Document& doc, Options::Action action, Api& api) {
  // if its a pbf request we want to keep the options and clear the rest
  bool pbf = false;
  if (api.has_options() && doc.ObjectEmpty()) {
    api.clear_trip();
    api.clear_directions();
    api.clear_status();
    api.clear_info();
    pbf = true;
  } // when its json we start with a blank slate and fill it all in
  else {
    api.Clear();
  }

  // set the action
  auto& options = *api.mutable_options();
  if (Options::Action_IsValid(action))
    options.set_action(action);

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

  if (!is_format_supported(options.action(), options.format())) {
    options.set_format(Options::json);
    add_warning(api, 211);
  }
  if (options.format() == Options::pbf) {
    // jsonp wont work because javascript doesnt support byte arrays
    options.clear_jsonp();
  }

  auto units = rapidjson::get_optional<std::string>(doc, "/units");
  if (units && ((*units == "miles") || (*units == "mi"))) {
    options.set_units(Options::miles);
  }

  // Whether or not to run isochrones in reverse in absence of time dependence
  options.set_reverse(rapidjson::get<bool>(doc, "/reverse", false));

  auto language = rapidjson::get_optional<std::string>(doc, "/language");
  if (language && odin::get_locales().find(*language) != odin::get_locales().end()) {
    options.set_language(*language);
  }
  if (!options.has_language_case()) {
    options.set_language("en-US");
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

  // costing defaults to none which is only valid for locate
  auto costing_str =
      rapidjson::get<std::string>(doc, "/costing",
                                  pbf ? Costing_Enum_Name(options.costing_type()) : "none");

  // auto_shorter is deprecated and will be turned into
  // shortest=true costing option. maybe remove in v4?
  if (costing_str == "auto_shorter") {

    add_warning(api, 100);

    costing_str = "auto";
    rapidjson::SetValueByPointer(doc, "/costing", "auto");
    auto json_options = rapidjson::GetValueByPointer(doc, "/costing_options/auto_shorter");
    if (json_options) {
      rapidjson::SetValueByPointer(doc, "/costing_options/auto", *json_options);
    }
    rapidjson::SetValueByPointer(doc, "/costing_options/auto/shortest", true);
  }

  // hov costing is deprecated and will be turned into auto costing with
  // include_hov2=true costing option.
  if (costing_str == "hov") {

    // add warning for hov costing
    add_warning(api, 101);

    costing_str = "auto";
    rapidjson::SetValueByPointer(doc, "/costing", "auto");
    auto json_options = rapidjson::GetValueByPointer(doc, "/costing_options/hov");
    if (json_options) {
      rapidjson::SetValueByPointer(doc, "/costing_options/auto", *json_options);
    }
    rapidjson::SetValueByPointer(doc, "/costing_options/auto/include_hov2", true);
  }

  // auto_data_fix is deprecated and will be turned into
  // ignore all the things costing option. maybe remove in v4?
  if (costing_str == "auto_data_fix") {

    // warning for auto data fix
    add_warning(api, 102);

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

  // set the costing based on the name given and parse its costing options
  Costing::Type costing;
  if (!valhalla::Costing_Enum_Parse(costing_str, &costing))
    throw valhalla_exception_t{125, "'" + costing_str + "'"};
  else
    options.set_costing_type(costing);

  // date_time
  auto date_time_type = rapidjson::get_optional<unsigned int>(doc, "/date_time/type");
  if (date_time_type && Options::DateTimeType_IsValid(*date_time_type + 1)) {
    options.set_date_time_type(static_cast<Options::DateTimeType>(*date_time_type + 1));
  }
  if (options.date_time_type() != Options::no_time) {
    // check the type is in bounds
    auto v = options.date_time_type();
    if (v >= Options::DateTimeType_ARRAYSIZE)
      throw valhalla_exception_t{163};
    // check the value exists for depart at and arrive by
    auto date_time_value =
        v != Options::current
            ? rapidjson::get<std::string>(doc, "/date_time/value", options.date_time())
            : std::string("current");
    if (date_time_value.empty()) {
      if (v == Options::depart_at)
        throw valhalla_exception_t{160};
      else if (v == Options::arrive_by)
        throw valhalla_exception_t{161};
      else if (v == Options::invariant)
        throw valhalla_exception_t{165};
    }
    // check the value is sane
    if (date_time_value != "current" && !baldr::DateTime::is_iso_valid(date_time_value))
      throw valhalla_exception_t{162};
    options.set_date_time(date_time_value);
  } // not specified but you want transit, then we default to current
  else if (options.costing_type() == Costing::multimodal ||
           options.costing_type() == Costing::transit) {
    options.set_date_time_type(Options::current);
    options.set_date_time("current");
  }

  // failure scenarios with respect to time dependence
  if (options.date_time_type() != Options::no_time) {
    if (options.date_time_type() == Options::arrive_by ||
        options.date_time_type() == Options::invariant) {
      if (options.costing_type() == Costing::multimodal || options.costing_type() == Costing::transit)
        throw valhalla_exception_t{141};
      if (options.action() == Options::isochrone)
        throw valhalla_exception_t{142};
    }
  }

  // Set the output precision for shape/geometry (polyline encoding). Defaults to polyline6
  // This also controls the input precision for encoded_polyline in height action
  // TODO - this just for OSRM compatibility at the moment but could be supported
  auto shape_format = rapidjson::get_optional<std::string>(doc, "/shape_format");
  if (shape_format) {
    if (*shape_format == "polyline6") {
      options.set_shape_format(polyline6);
    } else if (*shape_format == "polyline5") {
      options.set_shape_format(polyline5);
    } else if (*shape_format == "geojson") {
      options.set_shape_format(geojson);
    } else if (*shape_format == "no_shape") {
      if (action == Options::height) {
        throw valhalla_exception_t{164};
      }
      options.set_shape_format(no_shape);
    } else {
      // Throw an error if shape format is invalid
      throw valhalla_exception_t{164};
    }
  } else if (action == Options::sources_to_targets) {
    options.set_shape_format(options.has_shape_format_case() ? options.shape_format() : no_shape);
  }

  // whether or not to output b64 encoded openlr
  auto linear_references = rapidjson::get_optional<bool>(doc, "/linear_references");
  if (linear_references) {
    options.set_linear_references(*linear_references);
  }

  auto admin_crossings = rapidjson::get_optional<bool>(doc, "/admin_crossings");
  if (admin_crossings) {
    options.set_admin_crossings(*admin_crossings);
  }

  // whatever our costing is, check to see if we are going to ignore_closures
  std::stringstream ss;
  ss << "/costing_options/" << costing_str << "/ignore_closures";
  auto ignore_closures = costing_str != "multimodal"
                             ? rapidjson::get_optional<bool>(doc, ss.str().c_str())
                             : boost::none;
  for (const auto& co : options.costings()) {
    if (co.second.type() == options.costing_type() &&
        co.second.options().has_ignore_closures_case()) {
      ignore_closures = co.second.options().ignore_closures();
      break;
    }
  }

  // TODO(nils): if we parse the costing options before the ignore_closures logic,
  //   the gurka_closure_penalty test fails. investigate why.. intuitively it makes no sense,
  //   as in the above logic the costing options aren't even parsed yet,
  //   so how can it determine "ignore_closures" there?
  sif::ParseCosting(doc, "/costing_options", options);

  // if any of the locations params have a date_time object in their locations, we'll remember
  // only /sources_to_targets will parse more than one location collection and there it's fine
  bool had_date_time = false;

  // parse map matching location input and encoded_polyline for height actions
  auto encoded_polyline = rapidjson::get_optional<std::string>(doc, "/encoded_polyline");
  if (encoded_polyline) {
    options.set_encoded_polyline(*encoded_polyline);
  }
  if (options.has_encoded_polyline_case()) {

    // Set the precision to use when decoding the polyline. For height actions (only)
    // either polyline6 (default) or polyline5 are supported. All other actions only
    // support polyline6 inputs at this time.
    double precision = 1e-6;
    if (options.action() == Options::height) {
      precision = options.shape_format() == valhalla::polyline5 ? 1e-5 : 1e-6;
    }

    options.mutable_shape()->Clear();
    auto decoded =
        midgard::decode<std::vector<midgard::PointLL>>(options.encoded_polyline(), precision);
    for (const auto& ll : decoded) {
      auto* sll = options.mutable_shape()->Add();
      sll->mutable_ll()->set_lat(ll.lat());
      sll->mutable_ll()->set_lng(ll.lng());
      // set type to via by default
      sll->set_type(valhalla::Location::kVia);
      sll->set_time(-1);
    }
    // first and last always get type break
    if (options.shape_size()) {
      options.mutable_shape(0)->set_type(valhalla::Location::kBreak);
      options.mutable_shape(options.shape_size() - 1)->set_type(valhalla::Location::kBreak);
    }
    // add the date time
    add_date_to_locations(options, *options.mutable_shape(), "shape");
  } // fall back from encoded polyline to array of locations
  else {
    parse_locations(doc, api, "shape", 134, ignore_closures, had_date_time);

    // if no shape then try 'trace'
    if (options.shape().size() == 0) {
      parse_locations(doc, api, "trace", 135, ignore_closures, had_date_time);
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
  options.set_use_timestamps(rapidjson::get<bool>(doc, "/use_timestamps", options.use_timestamps()));

  // Option to prioritize bidirectional a* over timedependent forward when depart_at is set.
  options.set_prioritize_bidirectional(
      rapidjson::get<bool>(doc, "/prioritize_bidirectional", options.prioritize_bidirectional()));

  // Throw an error if use_timestamps is set to true but there are no timestamps in the
  // trace (or no durations present)
  if (options.use_timestamps()) {
    bool has_time = false;
    for (const auto& s : options.shape()) {
      if (s.has_time_case()) {
        has_time = true;
        break;
      }
    }
    if (!has_time) {
      throw valhalla_exception_t{159};
    }
  }

  // Get the elevation interval for returning elevation along the path in a route or
  // trace attribute call. Defaults to 0.0 (no elevation is returned)
  constexpr float kMaxElevationInterval = 1000.0f;
  auto elevation_interval = rapidjson::get_optional<float>(doc, "/elevation_interval");
  if (elevation_interval) {
    options.set_elevation_interval(
        std::max(std::min(*elevation_interval, kMaxElevationInterval), 0.0f));
  } else {
    // Constrain to range [0-kMaxElevationInterval]
    options.set_elevation_interval(
        std::max(std::min(options.elevation_interval(), kMaxElevationInterval), 0.0f));
  }

  // Elevation service options
  options.set_range(rapidjson::get(doc, "/range", options.range()));
  constexpr uint32_t MAX_HEIGHT_PRECISION = 2;
  auto height_precision = rapidjson::get_optional<unsigned int>(doc, "/height_precision");
  if (height_precision && *height_precision <= MAX_HEIGHT_PRECISION) {
    options.set_height_precision(*height_precision);
  }

  // matrix can be slimmed down but shouldn't by default for backwards-compatibility reasons
  if (options.action() == Options::sources_to_targets) {
    options.set_verbose(
        rapidjson::get(doc, "/verbose", options.has_verbose_case() ? options.verbose() : true));
  } else {
    options.set_verbose(rapidjson::get(doc, "/verbose", options.verbose()));
  }

  // parse any named costings for re-costing a given path
  parse_recostings(doc, "/recostings", options);

  // get the locations in there
  parse_locations(doc, api, "locations", 130, ignore_closures, had_date_time);

  // get the sources in there
  parse_locations(doc, api, "sources", 131, ignore_closures, had_date_time);

  // get the targets in there
  parse_locations(doc, api, "targets", 132, ignore_closures, had_date_time);

  // if not a time dependent route/mapmatch disable time dependent edge speed/flow data sources
  if (options.date_time_type() == Options::no_time && !had_date_time &&
      (options.shape_size() == 0 || options.shape(0).time() == -1)) {
    for (auto& costing : *options.mutable_costings()) {
      costing.second.mutable_options()->set_flow_mask(
          static_cast<uint8_t>(costing.second.options().flow_mask()) &
          ~(valhalla::baldr::kPredictedFlowMask | valhalla::baldr::kCurrentFlowMask));
    }
  }

  // get the avoids in there
  // TODO: remove "avoid_locations/polygons" after some while
  if (doc.HasMember("avoid_locations"))
    parse_locations(doc, api, "avoid_locations", 133, ignore_closures, had_date_time);
  else
    parse_locations(doc, api, "exclude_locations", 133, ignore_closures, had_date_time);

  // Get the matrix_loctions option and set if sources or targets size is one
  // (option is only supported with one to many or many to one matrix requests)
  // TODO(nils): Why is that? IMO this makes a lot of sense for a many:many call
  // of timedistancematrix as well no?
  auto matrix_locations = rapidjson::get_optional<int>(doc, "/matrix_locations");
  if (matrix_locations && (options.sources_size() == 1 || options.targets_size() == 1)) {
    options.set_matrix_locations(*matrix_locations);
  } else if (!options.has_matrix_locations_case()) {
    options.set_matrix_locations(std::numeric_limits<uint32_t>::max());
  }

  // get the avoid polygons in there
  auto rings_req =
      rapidjson::get_child_optional(doc, doc.HasMember("avoid_polygons") ? "/avoid_polygons"
                                                                         : "/exclude_polygons");
  if (rings_req) {
    if (!rings_req->IsArray()) {
      add_warning(api, 204);
    } else {
      auto* rings_pbf = options.mutable_exclude_polygons();
      try {
        for (const auto& req_poly : rings_req->GetArray()) {
          if (!req_poly.IsArray() || (req_poly.IsArray() && req_poly.GetArray().Empty())) {
            continue;
          }
          auto* ring = rings_pbf->Add();
          parse_ring(ring, req_poly);
        }
      } catch (...) { throw valhalla_exception_t{137}; }
    }
  } // if it was there in the pbf already
  else if (options.exclude_polygons_size()) {
    for (auto& ring : *options.mutable_exclude_polygons()) {
      parse_ring(&ring, rapidjson::Value{});
    }
  }

  // get some parameters
  auto resample_distance = rapidjson::get_optional<double>(doc, "/resample_distance");
  if (resample_distance) {
    options.set_resample_distance(*resample_distance);
  }

  // expansion action
  auto exp_action_str = rapidjson::get_optional<std::string>(doc, "/action");
  Options::Action exp_action;
  if (exp_action_str) {
    if (!Options_ExpansionAction_Enum_Parse(*exp_action_str, &exp_action)) {
      throw valhalla_exception_t(144, *exp_action_str);
    }
    options.set_expansion_action(exp_action);
  }
  if (options.action() == Options::expansion && options.expansion_action() == Options::no_action) {
    throw valhalla_exception_t(115, std::string("action"));
  }

  // expansion response properties
  auto exp_props_req = rapidjson::get_child_optional(doc, "/expansion_properties");
  auto* exp_props_pbf = options.mutable_expansion_properties();
  Options::ExpansionProperties exp_prop;
  if (exp_props_req && exp_props_req->IsArray()) {
    for (const auto& prop : exp_props_req->GetArray()) {
      if (!valhalla::Options_ExpansionProperties_Enum_Parse(std::string(prop.GetString()),
                                                            &exp_prop)) {
        throw valhalla_exception_t(168, std::string(prop.GetString()));
      }
      exp_props_pbf->Add(exp_prop);
    }
  }

  // should the expansion track opposites?
  options.set_skip_opposites(rapidjson::get<bool>(doc, "/skip_opposites", options.skip_opposites()));

  // should the expansion be less verbose, printing each edge only once, default false
  options.set_dedupe(rapidjson::get<bool>(doc, "/dedupe", options.dedupe()));

  // get the contours in there
  parse_contours(doc, options.mutable_contours());

  // if specified, get the polygons boolean in there
  options.set_polygons(rapidjson::get<bool>(doc, "/polygons", options.polygons()));

  // if specified, get the denoise in there
  auto denoise =
      rapidjson::get<float>(doc, "/denoise", options.has_denoise_case() ? options.denoise() : 1.0);
  options.set_denoise(std::max(std::min(denoise, 1.f), 0.f));

  // if specified, get the generalize value in there
  auto generalize = rapidjson::get_optional<float>(doc, "/generalize");
  if (generalize) {
    options.set_generalize(*generalize);
  }

  // if specified, get the show_locations boolean in there
  options.set_show_locations(rapidjson::get<bool>(doc, "/show_locations", options.show_locations()));

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
      std::string attribute = filter_attribute.GetString();
      // we renamed `edge.tagged_names` to `thor::kEdgeTaggedValues` and do it for backward
      // compatibility
      if (attribute == "edge.tagged_names") {
        attribute = baldr::kEdgeTaggedValues;
      }
      options.add_filter_attributes(attribute);
    }
  }

  // add warning for deprecated best_paths
  if (rapidjson::get_optional<uint32_t>(doc, "/best_paths")) {
    add_warning(api, 103);
  }
  // deprecated best_paths for map matching top k
  auto best_paths = std::max(uint32_t(1), rapidjson::get<uint32_t>(doc, "/best_paths", 1));

  // how many alternates are desired, default to none and if its multi point its also none
  options.set_alternates(rapidjson::get<uint32_t>(doc, "/alternates",
                                                  options.has_alternates_case() ? options.alternates()
                                                                                : best_paths - 1));
  if (options.action() != Options::trace_attributes && options.locations_size() > 2)
    options.set_alternates(0);

  // whether to return guidance_views, default false
  options.set_guidance_views(rapidjson::get<bool>(doc, "/guidance_views", options.guidance_views()));

  // whether to return bannerInstructions in OSRM serializer, default false
  options.set_banner_instructions(
      rapidjson::get<bool>(doc, "/banner_instructions", options.banner_instructions()));

  // whether to return voiceInstructions in OSRM serializer, default false
  options.set_voice_instructions(
      rapidjson::get<bool>(doc, "/voice_instructions", options.voice_instructions()));

  options.set_turn_lanes(rapidjson::get<bool>(doc, "/turn_lanes", options.turn_lanes()));

  // whether to include roundabout_exit maneuvers, default true
  auto roundabout_exits =
      rapidjson::get<bool>(doc, "/roundabout_exits",
                           options.has_roundabout_exits_case() ? options.roundabout_exits() : true);
  options.set_roundabout_exits(roundabout_exits);

  // force these into the output so its obvious what we did to the user
  doc.AddMember({"language", allocator}, {options.language(), allocator}, allocator);
  doc.AddMember({"format", allocator},
                {valhalla::Options_Format_Enum_Name(options.format()), allocator}, allocator);
}

} // namespace

namespace valhalla {

std::string serialize_error(const valhalla_exception_t& exception, Api& request) {
  // get the http status
  std::stringstream body;

  // overwrite with osrm error response
  if (request.options().format() == Options::osrm) {
    body << (request.options().has_jsonp_case() ? request.options().jsonp() + "(" : "")
         << exception.osrm_error << (request.options().has_jsonp_case() ? ")" : "");
  } // valhalla json error response
  else if (request.options().format() != Options::pbf) {
    // build up the json map
    auto json_error = baldr::json::map({});
    json_error->emplace("status", exception.http_message);
    json_error->emplace("status_code", static_cast<uint64_t>(exception.http_code));
    json_error->emplace("error", std::string(exception.message));
    json_error->emplace("error_code", static_cast<uint64_t>(exception.code));
    body << (request.options().has_jsonp_case() ? request.options().jsonp() + "(" : "") << *json_error
         << (request.options().has_jsonp_case() ? ")" : "");
  }

  // keep track of what the error was
  auto* err = request.mutable_info()->mutable_errors()->Add();
  err->set_description(exception.message);
  err->set_code(exception.code);

  // write a few stats about the error
  auto worker = exception.code < 200 || (exception.code >= 300 && exception.code < 400)
                    ? ".loki."
                    : (exception.code >= 400 && exception.code <= 500 ? ".thor." : ".odin.");
  const auto& action = Options_Action_Enum_Name(request.options().action());
  auto level = 500 <= exception.http_code && exception.http_code < 600 ? ".error" : ".warn";

  auto* err_stat = request.mutable_info()->mutable_statistics()->Add();
  err_stat->set_key(action + level + worker + exception.statsd_key);
  err_stat->set_value(1);
  err_stat->set_type(count);

  // pbf format output, we only send back the info with errors in it
  if (request.options().format() == Options::pbf) {
    Api error_only;
    error_only.mutable_info()->Swap(request.mutable_info());
    auto bytes = error_only.SerializeAsString();
    // if we are handling a service request we need the request intact
    if (error_only.info().is_service())
      error_only.mutable_info()->Swap(request.mutable_info());
    // otherwise we can blank the object save for the info
    else
      request.Swap(&error_only);
    return bytes;
  }

  // json
  return body.str();
}

void ParseApi(const std::string& request, Options::Action action, valhalla::Api& api) {
  // maybe parse some json
  auto document = from_string(request, valhalla_exception_t{100});
  from_json(document, action, api);
}

hierarchy_limits_config_t
parse_hierarchy_limits_from_config(const boost::property_tree::ptree& config,
                                   const std::string& algorithm,
                                   const bool uses_dist) {
  std::vector<HierarchyLimits> max_hierarchy_limits;
  std::vector<HierarchyLimits> default_hierarchy_limits;
  default_hierarchy_limits.reserve(baldr::TileHierarchy::levels().size());
  max_hierarchy_limits.reserve(baldr::TileHierarchy::levels().size());
  bool found = true;
  bool is_bidir = algorithm != "unidirectional_astar";
  // get the default and max allowed values for each level
  for (auto it = baldr::TileHierarchy::levels().begin(); it != baldr::TileHierarchy::levels().end();
       ++it) {

    // get the service limits
    HierarchyLimits max_hl;
    auto max_allowed_up_transitions =
        config.get_child_optional("service_limits.hierarchy_limits." + algorithm +
                                  ".max_allowed_up_transitions." + std::to_string(it->level));
    found = found && ((it->level == 0) || max_allowed_up_transitions);
    max_hl.set_max_up_transitions(
        max_allowed_up_transitions
            ? max_allowed_up_transitions->get_value<uint32_t>(kDefaultMaxUpTransitions[it->level])
            : kDefaultMaxUpTransitions[it->level]);

    // if the algorithm uses distance to decide whether to expand a given level, set that property as
    // well
    if (uses_dist) {
      auto max_expand_within_dist =
          config.get_child_optional("service_limits.hierarchy_limits." + algorithm +
                                    ".max_expand_within_distance." + std::to_string(it->level));
      found = found && (algorithm == "costmatrix" || max_expand_within_dist);
      max_hl.set_expand_within_dist(
          max_expand_within_dist
              ? max_expand_within_dist->get_value<float>(kDefaultExpansionWithinDist[it->level])
          : is_bidir ? kDefaultExpansionWithinDistBidir[it->level]
                     : kDefaultExpansionWithinDist[it->level]);
    }
    max_hierarchy_limits.push_back(max_hl);

    // now the defaults
    HierarchyLimits default_hl;
    auto default_max_up_transitions = config.get_child_optional(
        "thor." + algorithm + ".hierarchy_limits.max_up_transitions." + std::to_string(it->level));
    found = found && ((it->level == 0) || default_max_up_transitions);
    default_hl.set_max_up_transitions(
        default_max_up_transitions
            ? default_max_up_transitions->get_value<uint32_t>(kDefaultMaxUpTransitions[it->level])
            : kDefaultMaxUpTransitions[it->level]);

    // if the algorithm uses distance to decide whether to expand a given level, set that property
    // as well
    if (uses_dist) {
      auto default_expand_within_dist = config.get_child_optional(
          "thor." + algorithm + ".hierarchy_limits.expand_within_distance." +
          std::to_string(it->level));
      found = found && (algorithm == "costmatrix" || default_expand_within_dist);
      default_hl.set_expand_within_dist(
          default_expand_within_dist
              ? default_expand_within_dist->get_value<float>(kDefaultExpansionWithinDist[it->level])
          : is_bidir ? kDefaultExpansionWithinDistBidir[it->level]
                     : kDefaultExpansionWithinDist[it->level]);
    }
    default_hierarchy_limits.push_back(default_hl);
  }

  if (!found) {
    LOG_WARN("Incomplete config for hierarchy limits found for " + algorithm +
             ". Falling back to defaults");
  }
  return {max_hierarchy_limits, default_hierarchy_limits};
};

bool check_hierarchy_limits(std::vector<HierarchyLimits>& hierarchy_limits,
                            sif::cost_ptr_t& cost,
                            const valhalla::Costing_Options& options,
                            const hierarchy_limits_config_t& config,
                            const bool allow_modifications,
                            const bool use_hierarchy_limits) {

  // keep track whether we need to mess with user provided limits
  bool add_warning = false;

  // for backwards compatibility, we need to track if the defaults are used. This matters in
  // unidirectional astar, where hierarchy limits are modified based on the astar heuristic
  bool default_limits = true;
  for (size_t i = 0; i < hierarchy_limits.size(); ++i) {
    HierarchyLimits& limits = hierarchy_limits[i];

    // special case: hierarchy culling option is enabled (checked in loki)
    if (options.disable_hierarchy_pruning()) {
      limits.set_max_up_transitions(kUnlimitedTransitions);
      continue;
    }

    // use defaults if modification is not allowed by the service or if user did not specify any
    // limits;
    if (!allow_modifications || (limits.max_up_transitions() == kUnlimitedTransitions &&
                                 limits.expand_within_dist() == kMaxDistance)) {
      add_warning = add_warning || (limits.max_up_transitions() != kUnlimitedTransitions ||
                                    limits.expand_within_dist() != kMaxDistance);
      if (use_hierarchy_limits) {
        limits.set_max_up_transitions(config.default_limits[i].max_up_transitions());
        limits.set_expand_within_dist(config.default_limits[i].expand_within_dist());
      }
      continue;
    }
    default_limits = false;
    // clamp to max values defined in service_limits
    if (limits.max_up_transitions() > config.max_limits[i].max_up_transitions()) {
      limits.set_max_up_transitions(config.max_limits[i].max_up_transitions());
      add_warning = true;
    }

    if (limits.expand_within_dist() < 0 || // float might be negative
        limits.expand_within_dist() > config.max_limits[i].expand_within_dist()) {
      limits.set_expand_within_dist(config.max_limits[i].expand_within_dist());
      add_warning = true;
    }
  }

  cost->SetDefaultHierarchyLimits(default_limits);

  return add_warning;
}

#ifdef ENABLE_SERVICES
void ParseApi(const http_request_t& request, valhalla::Api& api) {
  // block all but get and post
  if (request.method != method_t::POST && request.method != method_t::GET) {
    throw valhalla_exception_t{101};
  };

  // this is a service request
  api.Clear();
  api.mutable_info()->set_is_service(true);

  // get the action
  Options::Action action = static_cast<Options::Action>(Options::Action_ARRAYSIZE);
  if (!request.path.empty())
    Options_Action_Enum_Parse(request.path.substr(1), &action);

  // if its a protobuf mime go with that
  auto pbf_content = request.headers.find("Content-Type");
  if (pbf_content != request.headers.end() && pbf_content->second == worker::PBF_MIME.second) {
    if (!api.ParseFromString(request.body)) {
      throw valhalla_exception_t{103};
    }
    // validate the options
    rapidjson::Document dummy;
    dummy.SetObject();
    from_json(dummy, action, api);
    return;
  }

  // parse the json input
  rapidjson::Document document;
  auto& allocator = document.GetAllocator();
  const auto& json = request.query.find("json");
  if (json != request.query.end() && json->second.size() && json->second.front().size()) {
    document.Parse(json->second.front().c_str());
  } // no json parameter, check the body
  else if (!request.body.empty()) {
    document.Parse(request.body.c_str());
  } // no json at all
  else {
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

  // parse out the options
  from_json(document, action, api);
}

const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type ATTACHMENT{"Content-Disposition", "attachment; filename=route.gpx"};

worker_t::result_t serialize_error(const valhalla_exception_t& exception,
                                   http_request_info_t& request_info,
                                   Api& request) {
  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(exception.http_code, exception.http_message,
                           serialize_error(exception, request),
                           headers_t{CORS, request.options().has_jsonp_case() ? worker::JS_MIME
                                                                              : worker::JSON_MIME});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());

  return result;
}

worker_t::result_t
to_response(const std::string& data, http_request_info_t& request_info, const Api& request) {
  // try to get all the proper headers
  auto fmt = request.options().format();

  headers_t headers{CORS, fmt_to_mime(fmt)};
  if (fmt == Options::gpx)
    headers.insert(ATTACHMENT);

  // jsonp needs wrapped in a javascript function call
  worker_t::result_t result{false, std::list<std::string>(), ""};
  if (request.options().has_jsonp_case()) {
    headers.insert(worker::JS_MIME); // reset content type to javascript
    std::ostringstream stream;
    stream << request.options().jsonp() << '(';
    stream << data;
    stream << ')';

    http_response_t response(200, "OK", stream.str(), headers);
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
  } // everything else is bytes already
  else {
    http_response_t response(200, "OK", data, headers);
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
  }
  return result;
}

#endif

// TODO: when we want to use this in mjolnir too we can move this into a private header
// this is a wrapper of a third party lib that provides a client for statsd integration
// since metrics are important both for on- and offline processing we keep the impl here
// running services can use it and also data ETL that lives in mjolnir can use it
struct statsd_client_t : public Statsd::StatsdClient {
  statsd_client_t(const boost::property_tree::ptree& conf)
      : Statsd::StatsdClient(conf.get<std::string>("statsd.host", ""),
                             conf.get<int>("statsd.port", 8125),
                             conf.get<std::string>("statsd.prefix", ""),
                             conf.get<uint64_t>("statsd.batch_size", 500),
                             0) {
    auto host = conf.get<std::string>("statsd.host", "");
    if (!errorMessage().empty() && !host.empty()) {
      LOG_ERROR(errorMessage());
    }
    auto added_tags = conf.get_child_optional("statsd.tags");
    if (added_tags) {
      for (const auto& tag : *added_tags) {
        tags.push_back(tag.second.data());
      }
    }
  }
  std::vector<std::string> tags;
};

service_worker_t::service_worker_t(const boost::property_tree::ptree& conf) : interrupt(nullptr) {
  if (conf.count("statsd")) {
    statsd_client = std::make_unique<statsd_client_t>(conf);
  }
}
service_worker_t::~service_worker_t() {
}
void service_worker_t::set_interrupt(const std::function<void()>* interrupt_function) {
  interrupt = interrupt_function;
}
void service_worker_t::cleanup() {
  if (statsd_client) {
    // sends metrics to statsd server over udp
    statsd_client->flush();
  }
}
void service_worker_t::enqueue_statistics(Api& api) const {
  // nothing to do without stats
  if (!statsd_client || !api.has_info() || api.info().statistics().empty())
    return;

  // these have been filled out as the request progressed through the system
  for (const auto& stat : api.info().statistics()) {
    float frequency = stat.frequency() ? stat.frequency() : 1.f;
    switch (stat.type()) {
      case count:
        statsd_client->count(stat.key(), static_cast<int>(stat.value() + 0.5), frequency,
                             statsd_client->tags);
        break;
      case gauge:
        statsd_client->gauge(stat.key(), static_cast<unsigned int>(stat.value() + 0.5), frequency,
                             statsd_client->tags);
        break;
      case timing:
        statsd_client->timing(stat.key(), static_cast<unsigned int>(stat.value() + 0.5), frequency,
                              statsd_client->tags);
        break;
      case set:
        statsd_client->set(stat.key(), static_cast<unsigned int>(stat.value() + 0.5), frequency,
                           statsd_client->tags);
        break;
      // Handle protobuf sentinel values to avoid compiler warnings
      case StatisticType_INT_MIN_SENTINEL_DO_NOT_USE_:
      case StatisticType_INT_MAX_SENTINEL_DO_NOT_USE_:
        break;
    }
  }

  // before we are done with the request, if this was not an error we log it was ok
  if (api.info().errors().empty()) {
    const auto& action = Options_Action_Enum_Name(api.options().action());
    statsd_client->count(action + ".info." + service_name() + ".ok", 1, 1.f, statsd_client->tags);
  }
}

midgard::Finally<std::function<void()>> service_worker_t::measure_scope_time(Api& api) const {
  // we copy the captures that could go out of scope
  auto start = std::chrono::steady_clock::now();
  return midgard::Finally<std::function<void()>>([this, &api, start]() {
    auto elapsed = std::chrono::steady_clock::now() - start;
    auto e = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(elapsed).count();
    const auto& action = Options_Action_Enum_Name(api.options().action());

    auto* stat = api.mutable_info()->mutable_statistics()->Add();
    stat->set_key(action + ".info." + service_name() + ".latency_ms");
    stat->set_value(e);
    stat->set_type(timing);
  });
}

void service_worker_t::started() {
  if (statsd_client) {
    statsd_client->count("none.info." + service_name() + ".worker_started", 1, 1.f,
                         statsd_client->tags);
  }
}

} // namespace valhalla
