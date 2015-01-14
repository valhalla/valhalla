#include "tyr/nearest_handler.h"

namespace valhalla {
namespace tyr {

NearestHandler::NearestHandler(const boost::python::dict& dict_request) : Handler(dict_request) {

}

NearestHandler::~NearestHandler() {

}

std::string NearestHandler::Action() {
  throw std::runtime_error("Nearest is not yet supported");
}

}
}
