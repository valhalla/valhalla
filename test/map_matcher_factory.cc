// -*- mode: c++ -*-
#include <string>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "sif/costconstants.h"

#include "meili/map_matcher_factory.h"
#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

using ptree = boost::property_tree::ptree;

void TestMapMatcherFactory() {
  ptree root;
  boost::property_tree::read_json(VALHALLA_SOURCE_DIR "test/valhalla.json", root);

  // Do it thousand times to check memory leak
  for (size_t i = 0; i < 3000; i++) {

    // Test configuration priority
    {
      // Copy it so we can change it
      auto config = root;
      config.put<std::string>("meili.auto.hello", "world");
      config.put<std::string>("meili.default.hello", "default world");
      meili::MapMatcherFactory factory(config);
      auto matcher = factory.Create(odin::Costing::auto_);
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
      auto matcher = factory.Create(odin::Costing::bicycle);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kBicycle,
                        "travel mode must be bicycle");
      test::assert_bool(matcher->config().get<std::string>("hello") == "default world",
                        "config for bicycle should use default");
      delete matcher;
    }

    // Test configuration priority
    {
      auto config = root;
      meili::MapMatcherFactory factory(config);
      odin::DirectionsOptions options;
      int preferred_search_radius = 3;
      int incorrect_search_radius = 2;
      int default_search_radius = 1;
      options.set_search_radius(preferred_search_radius);
      config.put<int>("meili.auto.search_radius", incorrect_search_radius);
      config.put<int>("meili.default.search_radius", default_search_radius);
      auto matcher = factory.Create(odin::Costing::pedestrian, options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "travel mode should be pedestrian");
      test::assert_bool(matcher->config().get<int>("search_radius") == preferred_search_radius,
                        "preference for pedestrian should override pedestrian config");
      delete matcher;
    }

    // Test configuration priority
    {
      meili::MapMatcherFactory factory(root);
      odin::DirectionsOptions options;
      int preferred_search_radius = 3;
      options.set_search_radius(preferred_search_radius);
      auto matcher = factory.Create(odin::Costing::multimodal, options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "travel mode should be pedestrian");
      test::assert_bool(matcher->config().get<int>("search_radius") == preferred_search_radius,
                        "preference for universal should override config");
      delete matcher;
    }

    // Test default mode
    {
      meili::MapMatcherFactory factory(root);
      auto matcher = factory.Create(odin::DirectionsOptions());
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kDrive,
                        "should read default mode in the meili.mode correctly");
      delete matcher;
    }

    // Test preferred mode
    {
      meili::MapMatcherFactory factory(root);
      odin::DirectionsOptions options;
      options.set_costing(odin::Costing::pedestrian);
      auto matcher = factory.Create(options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "should read costing in options correctly");
      delete matcher;

      options.set_costing(odin::Costing::bicycle);
      matcher = factory.Create(options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kBicycle,
                        "should read costing in options correctly again");
      delete matcher;
    }

    // Test custom costing
    {
      meili::MapMatcherFactory factory(root);
      odin::DirectionsOptions options;

      options.set_costing(odin::Costing::pedestrian);
      auto matcher = factory.Create(options);
      test::assert_bool(matcher->costing()->travel_type() != (int)sif::PedestrianType::kSegway,
                        "should not have custom costing options when not set in preferences");
      delete matcher;

      options.mutable_costing_options(static_cast<int>(odin::Costing::pedestrian))
          ->set_transport_type("segway");
      matcher = factory.Create(options);
      test::assert_bool(matcher->costing()->travel_type() == (int)sif::PedestrianType::kSegway,
                        "should read custom costing options in preferences correctly");
      delete matcher;
    }
  }
}

void TestMapMatcher() {
  ptree root;
  boost::property_tree::read_json(VALHALLA_SOURCE_DIR "test/valhalla.json", root);

  // Nothing special to test for the moment

  meili::MapMatcherFactory factory(root);
  auto auto_matcher = factory.Create(odin::Costing::auto_);
  auto pedestrian_matcher = factory.Create(odin::Costing::pedestrian);

  // Share the same pool
  test::assert_bool(&auto_matcher->graphreader() == &pedestrian_matcher->graphreader(),
                    "graph reader should be shared among matchers");
  test::assert_bool(&auto_matcher->candidatequery() == &pedestrian_matcher->candidatequery(),
                    "range query should be shared among matchers");

  delete auto_matcher;
  delete pedestrian_matcher;
}

int main(int argc, char* argv[]) {
  test::suite suite("map matching");

  suite.test(TEST_CASE(TestMapMatcherFactory));

  suite.test(TEST_CASE(TestMapMatcher));

  return suite.tear_down();
}
