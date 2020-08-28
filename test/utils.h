#pragma once

#include "baldr/rapidjson_utils.h"
#include "worker.h"

#include <boost/property_tree/ptree.hpp>
#include <sstream>

namespace test {

inline boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

} // namespace test
