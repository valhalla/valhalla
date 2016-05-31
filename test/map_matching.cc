// -*- mode: c++ -*-

//enable asserts for this test
#undef NDEBUG

#include <cassert>
#include <iostream>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/sif/costconstants.h>

#include "meili/universal_cost.h"
#include "meili/map_matching.h"

using namespace valhalla;

using ptree = boost::property_tree::ptree;


void TestMapMatcherFactory(const ptree& root)
{
  // Test configuration priority
  {
    // Copy it so we can change it
    auto config = root;
    config.put<std::string>("meili.auto.hello", "world");
    config.put<std::string>("meili.default.hello", "default world");
    meili::MapMatcherFactory factory(config);
    auto matcher = factory.Create("auto");
    assert(matcher->travelmode() == sif::TravelMode::kDrive);
    assert(matcher->config().get<std::string>("hello") == "world");
    delete matcher;
  }

  // Test configuration priority
  {
    auto config = root;
    config.put<std::string>("meili.default.hello", "default world");
    meili::MapMatcherFactory factory(config);
    auto matcher = factory.Create("bicycle");
    assert(matcher->travelmode() == sif::TravelMode::kBicycle);
    assert(matcher->config().get<std::string>("hello") == "default world");
    delete matcher;
  }

  // Test configuration priority
  {
    auto config = root;
    meili::MapMatcherFactory factory(config);
    ptree preferences;
    preferences.put<std::string>("hello", "preferred world");
    config.put<std::string>("meili.auto.hello", "world");
    config.put<std::string>("meili.default.hello", "default world");
    auto matcher = factory.Create(sif::TravelMode::kPedestrian, preferences);
    assert(matcher->travelmode() == sif::TravelMode::kPedestrian);
    assert(matcher->config().get<std::string>("hello") == "preferred world");
    delete matcher;
  }

  // Test configuration priority
  {
    meili::MapMatcherFactory factory(root);
    ptree preferences;
    preferences.put<std::string>("hello", "preferred world");
    auto matcher = factory.Create("multimodal", preferences);
    assert(matcher->travelmode() == meili::kUniversalTravelMode);
    assert(matcher->config().get<std::string>("hello") == "preferred world");
    delete matcher;
  }

  // Test default mode
  {
    meili::MapMatcherFactory factory(root);
    ptree preferences;
    auto matcher = factory.Create(preferences);
    assert(matcher->travelmode() == factory.NameToTravelMode(root.get<std::string>("meili.mode")));
    delete matcher;
  }

  // Test preferred mode
  {
    meili::MapMatcherFactory factory(root);
    ptree preferences;
    preferences.put<std::string>("mode", "pedestrian");
    auto matcher = factory.Create(preferences);
    assert(matcher->travelmode() == sif::TravelMode::kPedestrian);
    delete matcher;

    preferences.put<std::string>("mode", "bicycle");
    matcher = factory.Create(preferences);
    assert(matcher->travelmode() == sif::TravelMode::kBicycle);
    delete matcher;
  }

  // Transport names
  {
    meili::MapMatcherFactory factory(root);

    assert(factory.NameToTravelMode("auto") == sif::TravelMode::kDrive);
    assert(factory.NameToTravelMode("bicycle") == sif::TravelMode::kBicycle);
    assert(factory.NameToTravelMode("pedestrian") == sif::TravelMode::kPedestrian);
    assert(factory.NameToTravelMode("multimodal") == meili::kUniversalTravelMode);

    assert(factory.TravelModeToName(sif::TravelMode::kDrive) == "auto");
    assert(factory.TravelModeToName(sif::TravelMode::kBicycle) == "bicycle");
    assert(factory.TravelModeToName(sif::TravelMode::kPedestrian) == "pedestrian");
    assert(factory.TravelModeToName(meili::kUniversalTravelMode) == "multimodal");
  }

  // Invalid transport mode name
  {
    meili::MapMatcherFactory factory(root);
    meili::MapMatcher* matcher = nullptr;
    bool happen = false;

    try {
      matcher = factory.Create("invalid_mode");
    } catch (const std::invalid_argument& ex) {
      happen = true;
    }
    assert(!matcher);
    assert(happen);

    matcher = nullptr;
    happen = false;
    try {
      matcher = factory.Create("");
    } catch (const std::invalid_argument& ex) {
      happen = true;
    }
    assert(!matcher);
    assert(happen);

    matcher = nullptr;
    happen = false;
    try {
      matcher = factory.Create(static_cast<sif::TravelMode>(7));
    } catch (const std::invalid_argument& ex) {
      happen = true;
    }
    assert(!matcher);
    assert(happen);
  }
}


void TestMapMatcher(const ptree& root)
{
  // Nothing special to test for the moment

  meili::MapMatcherFactory factory(root);
  auto auto_matcher = factory.Create("auto");
  auto pedestrian_matcher = factory.Create("pedestrian");

  // Share the same pool
  assert(&auto_matcher->graphreader() == &pedestrian_matcher->graphreader());
  assert(&auto_matcher->rangequery() == &pedestrian_matcher->rangequery());

  delete auto_matcher;
  delete pedestrian_matcher;
}


int main(int argc, char *argv[])
{
  ptree config;
  boost::property_tree::read_json("test/mm.json", config);

  // Do it thousand times to check memory leak
  for (size_t i = 0; i < 3000; i++) {
    TestMapMatcherFactory(config);
  }

  TestMapMatcher(config);

  std::cout << "all tests passed" << std::endl;
  return 0;
}
