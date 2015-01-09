#include "tyr/handler.h"

#include <stdexcept>
#include <utility>
#include <boost/property_tree/json_parser.hpp>

namespace valhalla {
namespace tyr {

Handler::Handler(const boost::python::dict& dict_request) {
  //we require locations
  if(!dict_request.has_key("loc"))
    throw std::runtime_error("required parameter `loc' was not provided");
  for(const auto& location : dict_request["loc"]) {
    //TODO: worry about whether it was a break point or a through point
    locations_.emplace_back(std::move(baldr::Location::FromCsv(location)));
  }
}

}
}
