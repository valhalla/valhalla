#include "tyr/nearest_handler.h"

namespace valhalla {
namespace tyr {

NearestHandler::NearestHandler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& request) : Handler(config, request) {

}

NearestHandler::~NearestHandler() {

}

std::string NearestHandler::Action() {
  throw std::runtime_error("Nearest is not yet supported");
}

}
}
