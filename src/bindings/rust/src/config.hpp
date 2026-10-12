#pragma once

#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/midgard/logging.h>
#include <boost/property_tree/ptree.hpp>

#include "rust/cxx.h"

inline std::unique_ptr<boost::property_tree::ptree> from_file(rust::slice<const uint8_t> path) {
  auto pt = std::make_unique<boost::property_tree::ptree>();
  try {
    std::string str(reinterpret_cast<const char*>(path.data()), path.size());
    rapidjson::read_json(str, *pt);
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to read config file: " + std::string(e.what()));
  }
  return pt;
}

inline std::unique_ptr<boost::property_tree::ptree> from_json(rust::str config) {
  rapidjson::Document d;
  d.Parse(config.data(), config.size());
  if (d.HasParseError()) {
    throw std::runtime_error("Could not parse json, error at offset: " + std::to_string(d.GetErrorOffset()));
  }

  auto pt = std::make_unique<boost::property_tree::ptree>();
  if (d.IsObject()) {
    rapidjson::add_object(const_cast<const rapidjson::Document*>(&d)->GetObject(), *pt);
  } else if (d.IsArray()) {
    rapidjson::add_array(const_cast<const rapidjson::Document*>(&d)->GetArray(), *pt);
  } else {
    throw std::runtime_error("Json is not an object or array");
  }
  return pt;
}

// Config builder helpers: set values in a ptree using boost's dotted-path syntax.
inline void ptree_put_str(boost::property_tree::ptree& pt, rust::Str path, rust::Str value) {
  pt.put(std::string(path.data(), path.size()), std::string(value.data(), value.size()));
}

inline void ptree_put_bool(boost::property_tree::ptree& pt, rust::Str path, bool value) {
  pt.put(std::string(path.data(), path.size()), value);
}

inline void ptree_put_int(boost::property_tree::ptree& pt, rust::Str path, int64_t value) {
  pt.put(std::string(path.data(), path.size()), value);
}

inline void ptree_put_float(boost::property_tree::ptree& pt, rust::Str path, double value) {
  pt.put(std::string(path.data(), path.size()), value);
}

inline std::unique_ptr<boost::property_tree::ptree> ptree_new() {
  return std::make_unique<boost::property_tree::ptree>();
}

inline void ptree_put_str_array(boost::property_tree::ptree& pt, rust::Str path, rust::Slice<const rust::String> values) {
  boost::property_tree::ptree children;
  for (const auto& v : values) {
    boost::property_tree::ptree child;
    child.put_value(std::string(v.data(), v.size()));
    children.push_back({"", child});
  }
  pt.put_child(std::string(path.data(), path.size()), children);
}

inline void ptree_put_int_array(boost::property_tree::ptree& pt, rust::Str path, rust::Slice<const int64_t> values) {
  boost::property_tree::ptree children;
  for (auto v : values) {
    boost::property_tree::ptree child;
    child.put_value(v);
    children.push_back({"", child});
  }
  pt.put_child(std::string(path.data(), path.size()), children);
}

/// Initialize Valhalla's logger with the given config.
inline void configure_logging(const boost::property_tree::ptree& pt) {
  if (pt.get_child_optional("logging")) {
    valhalla::midgard::logging::ConfigureFromPtree(pt);
  }
}
