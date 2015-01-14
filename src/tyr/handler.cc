#include "tyr/handler.h"

#include <stdexcept>
#include <utility>
#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/str.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace valhalla {
namespace tyr {

Handler::Handler(const boost::python::dict& dict_request) {
  //we require locations
  if(!dict_request.has_key("loc"))
    throw std::runtime_error("required parameter `loc' was not provided");
  boost::python::list locations_list = boost::python::extract<boost::python::list>(dict_request["loc"]);
  for(ssize_t i = 0; i < boost::python::len(locations_list); ++i) {
    //get the python string
    boost::python::str location_str = boost::python::str(locations_list[i]);
    //TODO: worry about whether it was a break point or a through point
    std::string location = boost::python::extract<std::string>(location_str);
    locations_.emplace_back(std::move(baldr::Location::FromCsv(location)));
  }
}

Handler::~Handler() {

}

}
}
