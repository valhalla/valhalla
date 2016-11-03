#ifndef __VALHALLA_BALDR_ERRORCODE_UTIL_H__
#define __VALHALLA_BALDR_ERRORCODE_UTIL_H__

#include <functional>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace baldr {

  // Credits: http://werkzeug.pocoo.org/
  const std::unordered_map<unsigned, std::string> kHttpStatusCodes {
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
    {449,"Retry With"},  // proprietary MS extensio

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

  const std::unordered_map<unsigned, std::string> error_codes {
    // loki project 1xx
    {100,"Failed to parse json request"},
    {101,"Try a POST or GET request instead"},
    {102,"The config actions for Loki are incorrectly loaded"},
    {103,"Missing max_locations configuration"},
    {104,"Missing max_distance configuration"},
    {105,"Path action not supported"},
    {106,"Try any of"},
    {107,"Not Implemented"},

    {110,"Insufficiently specified required parameter 'locations'"},
    {111,"Insufficiently specified required parameter 'time'"},
    {112,"Insufficiently specified required parameter 'locations' or 'sources & targets'"},
    {113,"Insufficiently specified required parameter 'contours'"},
    {114,"Insufficiently specified required parameter 'shape' or 'encoded_polyline'"},

    {120,"Insufficient number of locations provided"},
    {121,"Insufficient number of sources provided"},
    {122,"Insufficient number of targets provided"},
    {123,"Insufficient shape provided"},
    {124,"No edge/node costing provided"},
    {125,"No costing method found"},
    {126,"No shape provided"},

    {130,"Failed to parse location"},
    {131,"Failed to parse source"},
    {132,"Failed to parse target"},

    {140,"Action does not support multimodal costing"},
    {141,"Arrive by for multimodal not implemented yet"},
    {142,"Arrive by not implemented for isochrones"},

    {150,"Exceeded max locations"},
    {151,"Exceeded max time"},
    {152,"Exceeded max contours"},
    {153,"Too many shape points"},
    {154,"Path distance exceeds the max distance limit"},
    {155,"Outside the valid walking distance at the beginning or end of a multimodal route"},
    {156,"Outside the valid walking distance between stops of a multimodal route"},

    {160,"Date and time required for origin for date_type of depart at"},
    {161,"Date and time required for destination for date_type of arrive by"},
    {162,"Date and time is invalid.  Format is YYYY-MM-DDTHH:MM"},
    {163,"Invalid date_type"},

    {170,"Locations are in unconnected regions. Go check/edit the map at osm.org"},
    {171,"No suitable edges near location"},

    {199,"Unknown"},

    // odin project 2xx
    {200,"Failed to parse intermediate request format"},
    {201,"Failed to parse TripPath"},
    {202,"Could not build directions for TripPath"},

    {210,"Trip path does not have any nodes"},
    {211,"Trip path has only one node"},
    {212,"Trip must have at least 2 locations"},
    {213,"Error - No shape or invalid node count"},

    {220,"Turn degree out of range for cardinal direction"},

    {230,"Invalid TripDirections_Maneuver_Type in method FormTurnInstruction"},
    {231,"Invalid TripDirections_Maneuver_Type in method FormRelativeTwoDirection"},
    {232,"Invalid TripDirections_Maneuver_Type in method FormRelativeThreeDirection"},

    {299,"Unknown"},

    // skadi project 3xx
    {300,"Failed to parse json request"},
    {301,"Try a POST or GET request instead"},
    {302,"The config actions for Skadi are incorrectly loaded"},
    {303,"Path action not supported"},
    {304,"Try any of"},
    {305,"Not Implemented"},

    {310,"No shape provided"},
    {311,"Insufficient shape provided"},
    {312,"Insufficiently specified required parameter 'shape' or 'encoded_polyline'"},
    {313,"'resample_distance' must be >= "},
    {314,"Too many shape points"},

    {399,"Unknown"},

    // thor project 4xx
    {400,"Unknown action"},
    {401,"Failed to parse intermediate request format"},

    {410,"Insufficiently specified required parameter 'locations'"},
    {411,"Insufficiently specified required parameter 'shape'"},
    {412,"No costing method found"},

    {420,"Failed to parse correlated location"},
    {421,"Failed to parse location"},
    {422,"Failed to parse source"},
    {423,"Failed to parse target"},
    {424,"Failed to parse shape"},

    {430,"Exceeded max iterations in CostMatrix::SourceToTarget"},

    {440,"Cannot reach destination - too far from a transit stop"},
    {441,"Location is unreachable"},
    {442,"No path could be found for input"},

    {499,"Unknown"},

    // tyr project 5xx
    {500,"Failed to parse intermediate request format"},
    {501,"Failed to parse TripDirections"},

    {599,"Unknown"}
  };

  struct valhalla_exception_t: public std::runtime_error {
    valhalla_exception_t(unsigned status_code, unsigned error_code, const boost::optional<std::string>& extra=boost::none)
      :std::runtime_error(""),
      error_code(error_code),
      status_code(status_code),
      extra(extra){
      auto code_itr = error_codes.find(error_code);
      error_code_message = (code_itr == error_codes.cend() ? "" : code_itr->second);
      error_code_message += (extra ? ":" + *extra : "");
      code_itr = kHttpStatusCodes.find(status_code);
      status_code_body = code_itr == kHttpStatusCodes.cend() ? "" : code_itr->second;

    }
    unsigned error_code;
    unsigned status_code;
    std::string error_code_message;
    std::string status_code_body;
    boost::optional<std::string> extra;
  };

}
}

#endif //__VALHALLA_BALDR_ERRORCODE_UTIL_H__
