#include <iostream>
#include <sstream>
#include <unordered_map>
#include <boost/property_tree/json_parser.hpp>

#include "worker.h"
#include "baldr/location.h"
#include "odin/util.h"

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
	{442,R"({"code":"InvalidUrl","message":"URL string is invalid."})"},
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
    auto& allocator = d.GetAllocator();
    d.Parse(json.c_str());
    if (d.HasParseError())
      throw e;
    return d;
  }

  valhalla::odin::DirectionsOptions from_json(rapidjson::Document& doc) {
    valhalla::odin::DirectionsOptions options;

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

    auto units = rapidjson::get_optional<std::string>(doc, "/units");
    if(units) {
      if((*units == "miles") || (*units == "mi"))
        options.set_units(valhalla::odin::DirectionsOptions::miles);
      else
        options.set_units(valhalla::odin::DirectionsOptions::kilometers);
    }

    auto language = rapidjson::get_optional<std::string>(doc, "/language");
    if(language && valhalla::odin::get_locales().find(*language) != valhalla::odin::get_locales().end())
      options.set_language(*language);

    auto narrative = rapidjson::get_optional<bool>(doc, "/narrative");
    if(narrative)
      options.set_narrative(*narrative);

    auto fmt = rapidjson::get_optional<std::string>(doc, "/format");
    valhalla::odin::DirectionsOptions::Format format;
    if (fmt && valhalla::odin::DirectionsOptions::Format_Parse(*fmt, &format))
      options.set_format(format);

    auto encoded_polyline = rapidjson::get_optional<std::string>(doc, "/encoded_polyline");
    if(encoded_polyline)
      options.set_encoded_polyline(*encoded_polyline);

    //TODO: remove this?
    options.set_do_not_track(rapidjson::get_optional<bool>(doc, "/healthcheck").get_value_or(false));

    options.set_range(rapidjson::get(doc, "/range", false));

    options.set_verbose(rapidjson::get(doc, "/verbose",false));

    //force these into the output so its obvious what we did to the user
    doc.AddMember({"language", allocator}, {options.language(), allocator}, allocator);
    doc.AddMember({"format", allocator},
      {valhalla::odin::DirectionsOptions::Format_Name(options.format()), allocator}, allocator);

    return options;
  }
}

namespace valhalla {

  valhalla_request_t::valhalla_request_t(){
    document.SetObject();
  }
  valhalla_request_t::valhalla_request_t(const std::string& request, odin::DirectionsOptions::Action action) {
    document = from_string(request, valhalla_exception_t{100});
    options = from_json(document);
    options.set_action(action);
  }
  valhalla_request_t::valhalla_request_t(const std::string& request, const std::string& serialized_options){
    document = from_string(request, valhalla_exception_t{100});
    options.ParseFromString(serialized_options);
  }

#ifdef HAVE_HTTP
  valhalla_request_t::valhalla_request_t(const http_request_t& request) {

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

    //parse out the options
    options = from_json(document);

    //set the action
    odin::DirectionsOptions::Action action;
    if(!request.path.empty() && odin::DirectionsOptions::Action_Parse(request.path.substr(1), &action))
      options.set_action(action);

    //disable analytics
    auto do_not_track = request.headers.find("DNT");
    options.set_do_not_track(options.do_not_track() ||
      (do_not_track != request.headers.cend() && do_not_track->second == "1"));
  }

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
  const headers_t::value_type XML_MIME{"Content-type", "text/xml;charset=utf-8"};

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
    http_response_t response(200, "OK", xml, headers_t{CORS, XML_MIME});
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
