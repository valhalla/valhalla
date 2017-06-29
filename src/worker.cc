#include <sstream>

#include "worker.h"

namespace valhalla {

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

    {300, 400},
    {301, 405},
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

#ifdef HAVE_HTTP
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  worker_t::result_t jsonify_error(const valhalla_exception_t& exception, http_request_info_t& request_info, const boost::optional<std::string>& jsonp) {
    //get the http status
    auto status = ERROR_TO_STATUS.find(exception.code)->second;
    auto body = HTTP_STATUS_CODES.find(status)->second;


    //build up the json map
    auto json_error = baldr::json::map({});
    json_error->emplace("status", body);
    json_error->emplace("status_code", static_cast<uint64_t>(status));
    json_error->emplace("error", std::string(exception.message));
    json_error->emplace("error_code", static_cast<uint64_t>(exception.code));

    //serialize it
    std::stringstream ss;
    if(jsonp)
      ss << *jsonp << '(';
    ss << *json_error;
    if(jsonp)
      ss << ')';

    worker_t::result_t result{false};
    http_response_t response(status, body, ss.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());

    return result;
  }

  worker_t::result_t to_response(baldr::json::ArrayPtr array, const boost::optional<std::string>& jsonp, http_request_info_t& request_info) {
    std::ostringstream stream;
    //jsonp callback if need be
    if(jsonp)
      stream << *jsonp << '(';
    stream << *array;
    if(jsonp)
      stream << ')';

    worker_t::result_t result{false};
    http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
    return result;
  }

  worker_t::result_t to_response(baldr::json::MapPtr map, const boost::optional<std::string>& jsonp, http_request_info_t& request_info) {
    std::ostringstream stream;
    //jsonp callback if need be
    if(jsonp)
      stream << *jsonp << '(';
    stream << *map;
    if(jsonp)
      stream << ')';

    worker_t::result_t result{false};
    http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
    return result;
  }

#endif

}
