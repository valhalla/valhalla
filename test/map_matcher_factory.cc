// -*- mode: c++ -*-
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "sif/costconstants.h"

#include "test.h"
#include "meili/universal_cost.h"
#include "meili/map_matcher_factory.h"


using namespace valhalla;

using ptree = boost::property_tree::ptree;


void TestMapMatcherFactory()
{
  ptree root;
  boost::property_tree::read_json("test/valhalla.json", root);

  // Do it thousand times to check memory leak
  for (size_t i = 0; i < 3000; i++) {

    // Test configuration priority
    {
      // Copy it so we can change it
      auto config = root;
      config.put<std::string>("meili.auto.hello", "world");
      config.put<std::string>("meili.default.hello", "default world");
      meili::MapMatcherFactory factory(config);
      auto matcher = factory.Create("auto");
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kDrive,
                        "travel mode should be drive");
      test::assert_bool(matcher->config().get<std::string>("hello") == "world",
                        "config for auto should override default");
      delete matcher;
    }

    // Test configuration priority
    {
      auto config = root;
      config.put<std::string>("meili.default.hello", "default world");
      meili::MapMatcherFactory factory(config);
      auto matcher = factory.Create("bicycle");
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kBicycle,
                        "travel mode must be bicycle");
      test::assert_bool(matcher->config().get<std::string>("hello") == "default world",
                        "config for drive should override default");
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
      auto matcher = factory.Create("pedestrian", preferences);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "travel mode should be pedestrian");
      test::assert_bool(matcher->config().get<std::string>("hello") == "preferred world",
                        "preference for pedestrian should override pedestrian config");
      delete matcher;
    }

    // Test configuration priority
    {
      meili::MapMatcherFactory factory(root);
      ptree preferences;
      preferences.put<std::string>("hello", "preferred world");
      auto matcher = factory.Create("multimodal", preferences);
      test::assert_bool(matcher->travelmode() == meili::kUniversalTravelMode,
                        "travel mode should be universal");
      test::assert_bool(matcher->config().get<std::string>("hello") == "preferred world",
                        "preference for universal should override config");
      delete matcher;
    }

    // Test default mode
    {
      meili::MapMatcherFactory factory(root);
      ptree preferences;
      auto matcher = factory.Create(preferences);
      test::assert_bool(matcher->travelmode() == meili::kUniversalTravelMode,
                        "should read default mode in the meili.mode correctly");
      delete matcher;
    }

    // Test preferred mode
    {
      meili::MapMatcherFactory factory(root);
      ptree preferences;
      preferences.put<std::string>("mode", "pedestrian");
      auto matcher = factory.Create(preferences);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "should read mode in preferences correctly");
      delete matcher;

      preferences.put<std::string>("mode", "bicycle");
      matcher = factory.Create(preferences);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kBicycle,
                        "should read mode in preferences correctly again");
      delete matcher;
    }

    // Invalid transport mode name
    {
      meili::MapMatcherFactory factory(root);

      test::assert_throw<std::runtime_error>([&factory]() {
          factory.Create("invalid_mode");
        }, "invalid_mode shuold be invalid mode");


      test::assert_throw<std::runtime_error>([&factory]() {
          factory.Create("");
        }, "empty string should be invalid mode");

    }
  }
}


void TestMapMatcher()
{
  ptree root;
  boost::property_tree::read_json("test/valhalla.json", root);

  // Nothing special to test for the moment

  meili::MapMatcherFactory factory(root);
  auto auto_matcher = factory.Create("auto");
  auto pedestrian_matcher = factory.Create("pedestrian");

  // Share the same pool
  test::assert_bool(&auto_matcher->graphreader() == &pedestrian_matcher->graphreader(),
                    "graph reader shoule be shared among matchers");
  test::assert_bool(&auto_matcher->candidatequery() == &pedestrian_matcher->candidatequery(),
                    "range query should be shared among matchers");

  delete auto_matcher;
  delete pedestrian_matcher;
}


int main(int argc, char *argv[])
{
  test::suite suite("map matching");

  suite.test(TEST_CASE(TestMapMatcherFactory));

  suite.test(TEST_CASE(TestMapMatcher));

  return suite.tear_down();
}
