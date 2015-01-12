#include "tyr/locate_handler.h"

namespace valhalla {
namespace tyr {

LocateHandler::LocateHandler(const boost::python::dict& dict_request) : Handler(dict_request) {

}

LocateHandler::~LocateHandler() {

}

std::string LocateHandler::Action() {
  throw std::runtime_error("Locate is not yet supported");
}

}
}
