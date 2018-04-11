#include <iostream>
#include <sstream>
#include <unordered_map>

#include "worker.h"
#include "baldr/datetime.h"
#include "baldr/location.h"
#include "odin/util.h"
#include "midgard/util.h"
#include "midgard/logging.h"
#include "midgard/encoded.h"

using namespace valhalla;

namespace {
  // Credits: http://werkzeug.pocoo.org/
  const std::unordered_map<unsigned, std::string> HTTP_STATUS_CODES {
    // 1xx
    {100,"Continue"},
    {101,"Switching Protocols"},
    {102,"Processing"},

    // 2xx
    {200,"OK"},
    {201,"Created"},
    {202,"Accepted"},
    {203,"Non Authoritative Information"},
    {204,"No Content"},
    {205,"Reset Content"},
    {206,"Partial Content"},
    {207,"Multi Status"},
    {226,"IM Used"},  // see RFC 322

    // 3xx
    {300,"Multiple Choices"},
    {301,"Moved Permanently"},
    {302,"Found"},
    {303,"See Other"},
    {304,"Not Modified"},
    {305,"Use Proxy"},
    {307,"Temporary Redirect"},

    // 4xx
    {400,"Bad Request"},
    {401,"Unauthorized"},
    {402,"Payment Required"},  // unuse
    {403,"Forbidden"},
    {404,"Not Found"},
    {405,"Method Not Allowed"},
    {406,"Not Acceptable"},
    {407,"Proxy Authentication Required"},
    {408,"Request Timeout"},
    {409,"Conflict"},
    {410,"Gone"},
    {411,"Length Required"},
    {412,"Precondition Failed"},
    {413,"Request Entity Too Large"},
    {414,"Request URI Too Long"},
    {415,"Unsupported Media Type"},
    {416,"Requested Range Not Satisfiable"},
    {417,"Expectation Failed"},
    {418,"I\'m a teapot"},  // see RFC 232
    {422,"Unprocessable Entity"},
    {423,"Locked"},
    {424,"Failed Dependency"},
    {426,"Upgrade Required"},
    {428,"Precondition Required"},  // see RFC 658
    {429,"Too Many Requests"},
    {431,"Request Header Fields Too Large"},
    {449,"Retry With"},  // proprietary MS extension

    // 5xx
    {500,"Internal Server Error"},
    {501,"Not Implemented"},
    {502,"Bad Gateway"},
    {503,"Service Unavailable"},
    {504,"Gateway Timeout"},
    {505,"HTTP Version Not Supported"},
    {507,"Insufficient Storage"},
    {510,"Not Extended"},
  };

  const std::unordered_map<unsigned, unsigned> ERROR_TO_STATUS {
    {100, 400},
    {101, 405},
    {106, 404},
    {107, 501},

    {110, 400},
    {111, 400},
    {112, 400},
    {113, 400},
    {114, 400},

    {120, 400},
    {121, 400},
    {122, 400},
    {123, 400},
    {124, 400},
    {125, 400},
    {126, 400},

    {130, 400},
    {131, 400},
    {132, 400},
    {133, 400},

    {140, 400},
    {141, 501},
    {142, 501},

    {150, 400},
    {151, 400},
    {152, 400},
    {153, 400},
    {154, 400},
    {155, 400},
    {156, 400},
    {157, 400},
    {158, 400},

    {160, 400},
    {161, 400},
    {162, 400},
    {163, 400},

    {170, 400},
    {171, 400},
    {172, 400},

    {199, 400},

    {200, 500},
    {201, 500},
    {202, 500},

    {210, 400},
    {211, 400},
    {212, 400},
    {213, 400},

    {220, 400},

    {230, 400},
    {231, 400},
    {232, 400},

    {299, 400},

    {304, 404},
    {305, 501},

    {310, 400},
    {311, 400},
    {312, 400},
    {313, 400},
    {314, 400},

    {399, 400},

    {400, 400},
    {401, 500},

    {420, 400},
    {421, 400},
    {422, 400},
    {423, 400},
    {424, 400},

    {430, 400},

    {440, 400},
    {441, 400},
    {442, 400},
    {443, 400},
    {444, 400},
    {445, 400},

    {499, 400},

    {500, 500},
    {501, 500},
    {502, 400},

    {599, 400},
  };

  const std::unordered_map<unsigned, std::string> OSRM_ERRORS_CODES {
	// loki project 1xx
	{100,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{101,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{106,R"({"code":"InvalidService","message":"Service name is invalid."})"},
	{107,R"({"code":"InvalidService","message":"Service name is invalid."})"},
	{110,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{111,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{112,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{113,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{114,R"({"code":"InvalidOptions","message":"Options are invalid."})"},

	{120,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{121,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{122,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{123,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{124,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{125,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{126,R"({"code":"InvalidOptions","message":"Options are invalid."})"},

	{130,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{131,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{132,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{133,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

	{140,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{141,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{142,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

	{150,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{151,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{152,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{153,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{154,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{155,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{156,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{157,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{158,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

	{160,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{161,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{162,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{163,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

	{170,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{171,R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})"},

	{199,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	// odin project 2xx
	{200,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{201,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{202,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{210,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{211,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{212,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{213,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{220,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{230,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{231,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{232,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{299,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	// skadi project 3xx
	{304,R"({"code":"InvalidService","message":"Service name is invalid."})"},
	{305,R"({"code":"InvalidService","message":"Service name is invalid."})"},

	{310,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{311,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{312,R"({"code":"InvalidOptions","message":"Options are invalid."})"},
	{313,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{314,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

	{399,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	// thor project 4xx
	{400,R"({"code":"InvalidService","message":"Service name is invalid."})"},
	{401,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{420,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{421,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{422,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{423,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},
	{424,R"({"code":"InvalidValue","message":"The successfully parsed query parameters are invalid."})"},

	{430,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{440,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{441,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{442,R"({"code":"NoRoute","message":"Impossible route between points"})"},
	{443,R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})"},
	{444,R"({"code":"NoSegment","message":"One of the supplied input coordinates could not snap to street segment."})"},
	{445,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{499,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	// tyr project 5xx
	{500,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{501,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
	{502,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},

	{599,R"({"code":"InvalidUrl","message":"URL string is invalid."})"}
  };

  rapidjson::Document from_string(const std::string& json, const std::exception& e) {
    rapidjson::Document d;
    d.Parse(json.c_str());
    if (d.HasParseError())
      throw e;
    return d;
  }

  void parse_locations(const rapidjson::Document& doc, google::protobuf::RepeatedPtrField<odin::Location>* locations,
      const std::string& node, unsigned location_parse_error_code, bool track) {

    auto request_locations = rapidjson::get_optional<rapidjson::Value::ConstArray>(doc, std::string("/" + node).c_str());
    if (request_locations) {
      for(const auto& r_loc : *request_locations) {
        try {
          auto* location = locations->Add();
          location->set_original_index(locations->size() - 1);

          auto lat = rapidjson::get_optional<float>(r_loc, "/lat");
          if (! lat) throw std::runtime_error{"lat is missing"};

          if (*lat < -90.0f || *lat > 90.0f)
            throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");

          auto lon = rapidjson::get_optional<float>(r_loc, "/lon");
          if (! lon) throw std::runtime_error{"lon is missing"};

          lon = midgard::circular_range_clamp<float>(*lon, -180, 180);
          location->mutable_ll()->set_lat(*lat);
          location->mutable_ll()->set_lng(*lon);

          auto stop_type_json = rapidjson::get_optional<std::string>(r_loc, "/type");
          if (stop_type_json && *stop_type_json == std::string("through")){
            location->set_type(odin::Location::kThrough);
          }

          auto name = rapidjson::get_optional<std::string>(r_loc, "/name");
          if(name) location->set_name(*name);
          auto street = rapidjson::get_optional<std::string>(r_loc, "/street");
          if(street) location->set_street(*street);
          auto city = rapidjson::get_optional<std::string>(r_loc, "/city");
          if(city) location->set_city(*city);
          auto state = rapidjson::get_optional<std::string>(r_loc, "/state");
          if(state) location->set_state(*state);
          auto zip = rapidjson::get_optional<std::string>(r_loc, "/postal_code");
          if(zip) location->set_postal_code(*zip);
          auto country = rapidjson::get_optional<std::string>(r_loc, "/country");
          if(country) location->set_country(*country);
          auto phone = rapidjson::get_optional<std::string>(r_loc, "/phone");
          if(phone) location->set_phone(*phone);
          auto url = rapidjson::get_optional<std::string>(r_loc, "/url");
          if(url) location->set_url(*url);

          auto date_time = rapidjson::get_optional<std::string>(r_loc, "/date_time");
          if(date_time) location->set_date_time(*date_time);
          auto heading = rapidjson::get_optional<int>(r_loc, "/heading");
          if(heading) location->set_heading(*heading);
          auto heading_tolerance = rapidjson::get_optional<int>(r_loc, "/heading_tolerance");
          if(heading_tolerance) location->set_heading_tolerance(*heading_tolerance);
          auto node_snap_tolerance = rapidjson::get_optional<float>(r_loc, "/node_snap_tolerance");
          if(node_snap_tolerance) location->set_node_snap_tolerance(*node_snap_tolerance);
          auto way_id = rapidjson::get_optional<uint64_t>(r_loc, "/way_id");
          if(way_id) location->set_way_id(*way_id);
          auto minimum_reachability = rapidjson::get_optional<unsigned int>(r_loc, "/minimum_reachability");
          if(minimum_reachability) location->set_minimum_reachability(*minimum_reachability);
          auto radius = rapidjson::get_optional<unsigned int>(r_loc, "/radius");
          if(radius) location->set_radius(*radius);
          auto accuracy = rapidjson::get_optional<unsigned int>(r_loc, "/accuracy");
          if(accuracy) location->set_accuracy(*accuracy);
          auto time = rapidjson::get_optional<unsigned int>(r_loc, "/time");
          if(time) location->set_time(*time);
          auto rank_candidates = rapidjson::get_optional<bool>(r_loc, "/rank_candidates");
          if(rank_candidates) location->set_rank_candidates(*rank_candidates);
        }
        catch (...) { throw valhalla_exception_t{location_parse_error_code}; }
      }
      if(track)
        midgard::logging::Log(node + "_count::" + std::to_string(request_locations->Size()), " [ANALYTICS] ");
    }
  }

  void from_json(rapidjson::Document& doc, odin::DirectionsOptions& options) {
    bool track = !options.has_do_not_track() || !options.do_not_track();

    //TODO: stop doing this after a sufficient amount of time has passed
    //move anything nested in deprecated directions_options up to the top level
    auto deprecated = get_child_optional(doc, "/directions_options");
    auto& allocator = doc.GetAllocator();
    if(deprecated) {
      for(const auto& key : {"/units", "/narrative", "/format", "/language"}) {
        auto child = rapidjson::get_child_optional(*deprecated, key);
        if(child)
          doc.AddMember(rapidjson::Value(&key[1], allocator), *child, allocator);
      }
      //delete directions_options if it existed
      doc.RemoveMember("directions_options");
    }

    auto fmt = rapidjson::get_optional<std::string>(doc, "/format");
    odin::DirectionsOptions::Format format;
    if (fmt && odin::DirectionsOptions::Format_Parse(*fmt, &format))
      options.set_format(format);

    auto id = rapidjson::get_optional<std::string>(doc, "/id");
    if(id)
      options.set_id(*id);

    auto jsonp = rapidjson::get_optional<std::string>(doc, "/jsonp");
    if(jsonp)
      options.set_jsonp(*jsonp);

    auto units = rapidjson::get_optional<std::string>(doc, "/units");
    if(units) {
      if((*units == "miles") || (*units == "mi"))
        options.set_units(odin::DirectionsOptions::miles);
      else
        options.set_units(odin::DirectionsOptions::kilometers);
    }

    auto language = rapidjson::get_optional<std::string>(doc, "/language");
    if(language && odin::get_locales().find(*language) != odin::get_locales().end())
      options.set_language(*language);

    auto narrative = rapidjson::get_optional<bool>(doc, "/narrative");
    if(narrative)
      options.set_narrative(*narrative);

    auto encoded_polyline = rapidjson::get_optional<std::string>(doc, "/encoded_polyline");
    if(encoded_polyline) {
      options.set_encoded_polyline(*encoded_polyline);
      auto decoded = midgard::decode<std::vector<midgard::PointLL> >(*encoded_polyline);
      for(const auto& ll : decoded) {
        auto* sll = options.mutable_shape()->Add();
        sll->mutable_ll()->set_lat(ll.lat());
        sll->mutable_ll()->set_lng(ll.lng());
      }
    }
    else
      parse_locations(doc, options.mutable_shape(), "shape", 134, false);

    //TODO: remove this?
    options.set_do_not_track(rapidjson::get_optional<bool>(doc, "/healthcheck").get_value_or(false));

    options.set_range(rapidjson::get(doc, "/range", false));

    options.set_verbose(rapidjson::get(doc, "/verbose",false));

    //costing
    auto costing_str = rapidjson::get_optional<std::string>(doc, "/costing");
    if(costing_str) {
      //try the string directly, some strings are keywords so add an underscore
      odin::DirectionsOptions::Costing costing;
      if(odin::DirectionsOptions::Costing_Parse(*costing_str, &costing))
        options.set_costing(costing);
      else if(odin::DirectionsOptions::Costing_Parse(*costing_str + '_', &costing))
        options.set_costing(costing);
      else
        throw valhalla_exception_t{125, "'" + *costing_str + "'"};
    }

    //TODO: costing options

    //get the locations in there
    parse_locations(doc, options.mutable_locations(), "locations", 130, track);

    //get the sources in there
    parse_locations(doc, options.mutable_sources(), "sources", 131, track);

    //get the targets in there
    parse_locations(doc, options.mutable_targets(), "targets", 132, track);

    //get the avoids in there
    parse_locations(doc, options.mutable_avoid_locations(), "avoid_locations", 133, track);

    //time type
    auto date_time_type = rapidjson::get_optional<float>(doc, "/date_time/type");
    if(date_time_type && odin::DirectionsOptions::DateTimeType_IsValid(*date_time_type)) {
      options.set_date_time_type(static_cast<odin::DirectionsOptions::DateTimeType>(*date_time_type));
    }//not specified but you want transit, then we default to current
    else if(options.has_costing() && (options.costing() == odin::DirectionsOptions::multimodal ||
        options.costing() == odin::DirectionsOptions::transit))
      options.set_date_time_type(odin::DirectionsOptions::current);

    //time value
    if(options.has_date_time_type()) {
      auto date_time_value = rapidjson::get_optional<std::string>(doc, "/date_time/value");
      switch(options.date_time_type()) {
      case odin::DirectionsOptions::current:
        options.set_date_time("current");
        options.mutable_locations(0)->set_date_time("current");
        break;
      case odin::DirectionsOptions::depart_at:
        if(!date_time_value)
          throw valhalla_exception_t{160};
        if(!baldr::DateTime::is_iso_local(*date_time_value))
          throw valhalla_exception_t{162};
        options.set_date_time(*date_time_value);
        options.mutable_locations(0)->set_date_time(*date_time_value);
        break;
      case odin::DirectionsOptions::arrive_by:
        //not yet for transit
        if(options.costing() == odin::DirectionsOptions::multimodal ||
            options.costing() == odin::DirectionsOptions::transit)
          throw valhalla_exception_t{141};
        if(!date_time_value)
          throw valhalla_exception_t{161};
        if(!baldr::DateTime::is_iso_local(*date_time_value))
          throw valhalla_exception_t{162};
        options.set_date_time(*date_time_value);
        options.mutable_locations()->rbegin()->set_date_time(*date_time_value);
        break;
      default:
        throw valhalla_exception_t{163};
      }
    }

    //get some parameters
    auto resample_distance = rapidjson::get_optional<double>(doc, "/resample_distance");
    if(resample_distance)
      options.set_resample_distance(*resample_distance);

    //force these into the output so its obvious what we did to the user
    doc.AddMember({"language", allocator}, {options.language(), allocator}, allocator);
    doc.AddMember({"format", allocator},
      {odin::DirectionsOptions::Format_Name(options.format()), allocator}, allocator);
  }
}

namespace valhalla {

  valhalla_request_t::valhalla_request_t(){
    document.SetObject();
  }
  void valhalla_request_t::parse(const std::string& request, odin::DirectionsOptions::Action action) {
    document = from_string(request, valhalla_exception_t{100});
    options.set_action(action);
    from_json(document, options);
    //TODO: sanity check the parsed values
  }
  void valhalla_request_t::parse(const std::string& request, const std::string& serialized_options){
    document = from_string(request, valhalla_exception_t{100});
    options.ParseFromString(serialized_options);
    //TODO: sanity check the parsed values
  }

#ifdef HAVE_HTTP
  void valhalla_request_t::parse(const http_request_t& request) {

    //block all but get and post
    if(request.method != method_t::POST && request.method != method_t::GET)
      throw valhalla_exception_t{101};

    auto& allocator = document.GetAllocator();
    //parse the input
    const auto& json = request.query.find("json");
    if (json != request.query.end() && json->second.size() && json->second.front().size())
      document.Parse(json->second.front().c_str());
    //no json parameter, check the body
    else if(!request.body.empty())
      document.Parse(request.body.c_str());
    //no json at all
    else
      document.SetObject();
    //if parsing failed
    if (document.HasParseError())
      throw valhalla_exception_t{100};

    //throw the query params into the rapidjson doc
    for(const auto& kv : request.query) {
      //skip json or empty entries
      if(kv.first == "json" || kv.first.empty() || kv.second.empty() || kv.second.front().empty())
        continue;

      //turn single value entries into single key value
      if(kv.second.size() == 1) {
        document.AddMember({kv.first, allocator}, {kv.second.front(), allocator}, allocator);
        continue;
      }

      //make an array of values for this key
      rapidjson::Value array{rapidjson::kArrayType};
      for(const auto& value : kv.second) {
        array.PushBack({value, allocator}, allocator);
      }
      document.AddMember({kv.first, allocator}, array, allocator);
    }

    //set the action
    odin::DirectionsOptions::Action action;
    if(!request.path.empty() && odin::DirectionsOptions::Action_Parse(request.path.substr(1), &action))
      options.set_action(action);

    //disable analytics
    auto do_not_track = request.headers.find("DNT");
    options.set_do_not_track(options.do_not_track() ||
      (do_not_track != request.headers.cend() && do_not_track->second == "1"));

    //parse out the options
    from_json(document, options);
  }

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
  const headers_t::value_type XML_MIME{"Content-type", "text/xml;charset=utf-8"};
  const headers_t::value_type GPX_MIME{"Content-type", "application/gpx+xml;charset=utf-8"};
  const headers_t::value_type ATTACHMENT{"Content-Disposition", "attachment; filename=route.gpx"};

  worker_t::result_t jsonify_error(const valhalla_exception_t& exception, http_request_info_t& request_info, const valhalla_request_t& request) {
    //get the http status
    auto status = ERROR_TO_STATUS.find(exception.code)->second;
    auto message = HTTP_STATUS_CODES.find(status)->second;
    std::stringstream body;

    //overwrite with osrm error response
    if(request.options.format() == odin::DirectionsOptions::osrm) {
      auto found = OSRM_ERRORS_CODES.find(exception.code);
      if(found == OSRM_ERRORS_CODES.cend())
        found = OSRM_ERRORS_CODES.find(199);
      body << (request.options.has_jsonp() ? request.options.jsonp() + "(" : "") << found->second << (request.options.has_jsonp() ? ")" : "");
    }//valhalla error response
    else {
      //build up the json map
      auto json_error = baldr::json::map({});
      json_error->emplace("status", message);
      json_error->emplace("status_code", static_cast<uint64_t>(status));
      json_error->emplace("error", std::string(exception.message));
      json_error->emplace("error_code", static_cast<uint64_t>(exception.code));
      body << (request.options.has_jsonp() ? request.options.jsonp() + "(" : "") << *json_error << (request.options.has_jsonp() ? ")" : "");
    }

    worker_t::result_t result{false};
    http_response_t response(status, message, body.str(), headers_t{CORS, request.options.has_jsonp() ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());

    return result;
  }

  worker_t::result_t to_response(baldr::json::ArrayPtr array, http_request_info_t& request_info, const valhalla_request_t& request) {
    std::ostringstream stream;
    //jsonp callback if need be
    if(request.options.has_jsonp())
      stream << request.options.jsonp() << '(';
    stream << *array;
    if(request.options.has_jsonp())
      stream << ')';

    worker_t::result_t result{false};
    http_response_t response(200, "OK", stream.str(), headers_t{CORS, request.options.has_jsonp() ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
    return result;
  }

  worker_t::result_t to_response(baldr::json::MapPtr map, http_request_info_t& request_info, const valhalla_request_t& request) {
    std::ostringstream stream;
    //jsonp callback if need be
    if(request.options.has_jsonp())
      stream << request.options.jsonp() << '(';
    stream << *map;
    if(request.options.has_jsonp())
      stream << ')';

    worker_t::result_t result{false};
    http_response_t response(200, "OK", stream.str(), headers_t{CORS, request.options.has_jsonp() ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
    return result;
  }

  worker_t::result_t to_response_json(const std::string& json, http_request_info_t& request_info, const valhalla_request_t& request) {
    std::ostringstream stream;
    //jsonp callback if need be
    if(request.options.has_jsonp())
      stream << request.options.jsonp() << '(';
    stream << json;
    if(request.options.has_jsonp())
      stream << ')';

    worker_t::result_t result{false};
    http_response_t response(200, "OK", stream.str(), headers_t{CORS, request.options.has_jsonp() ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
    return result;
  }

  worker_t::result_t to_response_xml(const std::string& xml, http_request_info_t& request_info, const valhalla_request_t& request) {
    worker_t::result_t result{false};
    http_response_t response(200, "OK", xml, headers_t{CORS, GPX_MIME, ATTACHMENT});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
    return result;
  }


#endif

  service_worker_t::service_worker_t(): interrupt(nullptr) {}
  service_worker_t::~service_worker_t() {}
  void service_worker_t::set_interrupt(const std::function<void ()>& interrupt_function) {
    interrupt = &interrupt_function;
  };

}
