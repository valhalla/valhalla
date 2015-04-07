#include "tyr/locate_handler.h"

namespace valhalla {
namespace tyr {

LocateHandler::LocateHandler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& request) : Handler(config, request) {

}

LocateHandler::~LocateHandler() {

}

std::string LocateHandler::Action() {
  throw std::runtime_error("Locate is not yet supported");
}

}
}
