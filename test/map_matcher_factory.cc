// -*- mode: c++ -*-
#include <string>

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

#include "sif/costconstants.h"
#include "sif/costfactory.h"

#include "meili/map_matcher_factory.h"
#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

using ptree = boost::property_tree::ptree;

// TODO - is there a better way to set these?
void create_costing_options(Options& options) {
  // Add options in the order specified
  //  for (const auto costing : {auto_, auto_shorter, bicycle, bus, hov,
  //                              motor_scooter, multimodal, pedestrian, transit,
  //                              truck, motorcycle, auto_data_fix}) {
  // TODO - accept RapidJSON as argument.
  const rapidjson::Document doc;
  sif::ParseAutoCostOptions(doc, "/costing_options/auto", options.add_costing_options());
  sif::ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter",
                                   options.add_costing_options());
  sif::ParseBicycleCostOptions(doc, "/costing_options/bicycle", options.add_costing_options());
  sif::ParseBusCostOptions(doc, "/costing_options/bus", options.add_costing_options());
  sif::ParseHOVCostOptions(doc, "/costing_options/hov", options.add_costing_options());
  sif::ParseTaxiCostOptions(doc, "/costing_options/taxi", options.add_costing_options());
  sif::ParseMotorScooterCostOptions(doc, "/costing_options/motor_scooter",
                                    options.add_costing_options());
  options.add_costing_options();
  sif::ParsePedestrianCostOptions(doc, "/costing_options/pedestrian", options.add_costing_options());
  sif::ParseTransitCostOptions(doc, "/costing_options/transit", options.add_costing_options());
  sif::ParseTruckCostOptions(doc, "/costing_options/truck", options.add_costing_options());
  sif::ParseMotorcycleCostOptions(doc, "/costing_options/motorcycle", options.add_costing_options());
  sif::ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter",
                                   options.add_costing_options());
  sif::ParseAutoDataFixCostOptions(doc, "/costing_options/auto_data_fix",
                                   options.add_costing_options());
}

void TestMapMatcherFactory() {
  ptree root;
  rapidjson::read_json(VALHALLA_SOURCE_DIR "test/valhalla.json", root);

  // Do it thousand times to check memory leak
  for (size_t i = 0; i < 3000; i++) {

    // Test configuration priority
    {
      // Copy it so we can change it
      Options options;
      create_costing_options(options);
      auto config = root;
      config.put<std::string>("meili.auto.hello", "world");
      config.put<std::string>("meili.default.hello", "default world");
      meili::MapMatcherFactory factory(config);
      auto matcher = factory.Create(Costing::auto_, options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kDrive,
                        "travel mode should be drive");
      // NOT SURE WHAT THIS IS SUPPOSED TO DO?
      //      test::assert_bool(matcher->config().get<std::string>("hello") == "world",
      //                        "config for auto should override default");
      delete matcher;
    }

    // Test configuration priority
    {
      Options options;
      create_costing_options(options);
      auto config = root;
      config.put<std::string>("meili.default.hello", "default world");
      meili::MapMatcherFactory factory(config);
      auto matcher = factory.Create(Costing::bicycle, options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kBicycle,
                        "travel mode must be bicycle");
      // NOT SURE WHAT THIS IS SUPPOSED TO DO?
      //      test::assert_bool(matcher->config().get<std::string>("hello") == "default world",
      //                        "config for bicycle should use default");
      delete matcher;
    }

    // Test configuration priority
    {
      Options options;
      create_costing_options(options);
      auto config = root;
      meili::MapMatcherFactory factory(config);
      float preferred_search_radius = 3;
      float incorrect_search_radius = 2;
      float default_search_radius = 1;
      options.set_search_radius(preferred_search_radius);
      config.put<int>("meili.auto.search_radius", incorrect_search_radius);
      config.put<int>("meili.default.search_radius", default_search_radius);
      auto matcher = factory.Create(Costing::pedestrian, options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "travel mode should be pedestrian");
      test::assert_bool(matcher->config().get<float>("search_radius") == preferred_search_radius,
                        "preference for pedestrian should override pedestrian config");
      delete matcher;
    }

    // Test configuration priority
    {
      Options options;
      create_costing_options(options);
      meili::MapMatcherFactory factory(root);
      float preferred_search_radius = 3;
      options.set_search_radius(preferred_search_radius);
      auto matcher = factory.Create(Costing::pedestrian, options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "travel mode should be pedestrian");
      test::assert_bool(matcher->config().get<float>("search_radius") == preferred_search_radius,
                        "preference for universal should override config");
      delete matcher;
    }

    // Test default mode
    {
      Options options;
      create_costing_options(options);
      meili::MapMatcherFactory factory(root);
      auto matcher = factory.Create(options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kDrive,
                        "should read default mode in the meili.mode correctly");
      delete matcher;
    }

    // Test preferred mode
    {
      Options options;
      create_costing_options(options);
      meili::MapMatcherFactory factory(root);
      options.set_costing(Costing::pedestrian);
      auto matcher = factory.Create(options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kPedestrian,
                        "should read costing in options correctly");
      delete matcher;

      options.set_costing(Costing::bicycle);
      matcher = factory.Create(options);
      test::assert_bool(matcher->travelmode() == sif::TravelMode::kBicycle,
                        "should read costing in options correctly again");
      delete matcher;
    }

    // Test custom costing
    {
      Options options;
      create_costing_options(options);
      meili::MapMatcherFactory factory(root);
      options.set_costing(Costing::pedestrian);
      auto matcher = factory.Create(options);
      test::assert_bool(matcher->costing()->travel_type() != (int)sif::PedestrianType::kSegway,
                        "should not have custom costing options when not set in preferences");
      delete matcher;

      options.mutable_costing_options(static_cast<int>(Costing::pedestrian))
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
  rapidjson::read_json(VALHALLA_SOURCE_DIR "test/valhalla.json", root);

  // Nothing special to test for the moment

  meili::MapMatcherFactory factory(root);
  Options options;
  create_costing_options(options);
  auto auto_matcher = factory.Create(Costing::auto_, options);
  auto pedestrian_matcher = factory.Create(Costing::pedestrian, options);

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
