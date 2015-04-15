#include "tyr/handler.h"

#include <stdexcept>
#include <utility>
#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/str.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace valhalla {
namespace tyr {

Handler::Handler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& request):config_(config) {
  //jsonp callback is optional
  auto maybe_jsonp = request.get_child_optional("jsonp");
  std::string jsonp;
  if(maybe_jsonp && (jsonp = maybe_jsonp->get_value<std::string>()).size() > 0) {
    jsonp_ = jsonp;
  }
}

Handler::~Handler() {

}

}
}
