#ifndef __VALHALLA_EXCEPTION_H__
#define __VALHALLA_EXCEPTION_H__

#include <string>
#include <stdexcept>
#include <unordered_map>
#include <boost/optional.hpp>

namespace valhalla {
  const std::unordered_map<unsigned, std::string> error_codes {
    // loki project 1xx
    {100,"Failed to parse json request"},
    {101,"Try a POST or GET request instead"},
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
    {133,"Failed to parse avoid"},

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
    {157,"Exceeded max avoid locations"},
    {158,"Input trace option is out of bounds"},

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

    {420,"Failed to parse correlated location"},
    {421,"Failed to parse location"},
    {422,"Failed to parse source"},
    {423,"Failed to parse target"},
    {424,"Failed to parse shape"},

    {430,"Exceeded max iterations in CostMatrix::SourceToTarget"},

    {440,"Cannot reach destination - too far from a transit stop"},
    {441,"Location is unreachable"},
    {442,"No path could be found for input"},
    {443,"Exact route match algorithm failed to find path"},
    {444,"Map Match algorithm failed to find path"},
    {445,"Shape match algorithm specification in api request is incorrect. Please see documentation for valid shape_match input."},

    {499,"Unknown"},

    // tyr project 5xx
    {500,"Failed to parse intermediate request format"},
    {501,"Failed to parse TripDirections"},
    {502,"Maneuver index not found for specified shape index"},

    {599,"Unknown"}
  };

  struct valhalla_exception_t: public std::runtime_error {
    valhalla_exception_t(unsigned code, const boost::optional<std::string>& extra=boost::none)
      :std::runtime_error(""),
       code(code),
      extra(extra){
      auto code_itr = error_codes.find(code);
      message = (code_itr == error_codes.cend() ? "" : code_itr->second);
      message += (extra ? ":" + *extra : "");
    }
    const char* what() const noexcept override {
      return message.c_str();
    }
    unsigned code;
    std::string message;
    boost::optional<std::string> extra;
  };
}

#endif //__VALHALLA_EXCEPTION_H__
