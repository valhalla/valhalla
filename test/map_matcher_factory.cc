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

namespace {

using namespace valhalla;

using ptree = boost::property_tree::ptree;

void create_costing_options(Costing::Type costing, Options& options) {
  const rapidjson::Document doc;
  sif::ParseCosting(doc, "/costing_options", options);
  options.set_costing_type(costing);
}

TEST(MapMatcherFactory, TestMapMatcherFactory) {
  const auto root = test::make_config("/data/valhala");

  // Do it thousand times to check memory leak
  for (size_t i = 0; i < 3000; i++) {

    // Test configuration priority
    {
      // Copy it so we can change it
      Options options;
      create_costing_options(Costing::auto_, options);
      auto config = root;
      config.put<std::string>("meili.auto.hello", "world");
      config.put<std::string>("meili.default.hello", "default world");
      meili::MapMatcherFactory factory(config);
      auto matcher = factory.Create(options);
      EXPECT_EQ(matcher->travelmode(), sif::TravelMode::kDrive);

      // NOT SURE WHAT THIS IS SUPPOSED TO DO?
      //      EXPECT_EQ(matcher->config().get<std::string>("hello"), "world")
      //                       << "config for auto should override default");
      delete matcher;
    }

    // Test configuration priority
    {
      Options options;
      create_costing_options(Costing::bicycle, options);
      auto config = root;
      config.put<std::string>("meili.default.hello", "default world");
      meili::MapMatcherFactory factory(config);
      auto matcher = factory.Create(options);
      EXPECT_EQ(matcher->travelmode(), sif::TravelMode::kBicycle);

      // NOT SURE WHAT THIS IS SUPPOSED TO DO?
      //      EXPECT_EQ(matcher->config().get<std::string>("hello"), "default world")
      //                        << "config for bicycle should use default");
      delete matcher;
    }

    // Test configuration priority
    {
      Options options;
      create_costing_options(Costing::pedestrian, options);
      auto config = root;
      meili::MapMatcherFactory factory(config);
      float preferred_search_radius = 3;
      float incorrect_search_radius = 2;
      float default_search_radius = 1;
      options.set_search_radius(preferred_search_radius);
      config.put<int>("meili.auto.search_radius", incorrect_search_radius);
      config.put<int>("meili.default.search_radius", default_search_radius);
      auto matcher = factory.Create(options);
      EXPECT_EQ(matcher->travelmode(), sif::TravelMode::kPedestrian);
      EXPECT_EQ(matcher->config().candidate_search.search_radius_meters, preferred_search_radius)
          << "preference for pedestrian should override pedestrian config";

      delete matcher;
    }

    // Test configuration priority
    {
      Options options;
      create_costing_options(Costing::pedestrian, options);
      meili::MapMatcherFactory factory(root);
      float preferred_search_radius = 3;
      options.set_search_radius(preferred_search_radius);
      auto matcher = factory.Create(options);
      EXPECT_EQ(matcher->travelmode(), sif::TravelMode::kPedestrian);
      EXPECT_EQ(matcher->config().candidate_search.search_radius_meters, preferred_search_radius)
          << "preference for universal should override config";
      delete matcher;
    }

    // Test default mode
    {
      Options options;
      create_costing_options(Costing::auto_, options);
      meili::MapMatcherFactory factory(root);
      auto matcher = factory.Create(options);
      EXPECT_EQ(matcher->travelmode(), sif::TravelMode::kDrive)
          << "should read default mode in the meili.mode correctly";

      delete matcher;
    }

    // Test preferred mode
    {
      Options options;
      create_costing_options(Costing::pedestrian, options);
      meili::MapMatcherFactory factory(root);
      auto matcher = factory.Create(options);
      EXPECT_EQ(matcher->travelmode(), sif::TravelMode::kPedestrian)
          << "should read costing in options correctly";

      delete matcher;

      options.set_costing_type(Costing::bicycle);
      matcher = factory.Create(options);
      EXPECT_EQ(matcher->travelmode(), sif::TravelMode::kBicycle)
          << "should read costing in options correctly again";

      delete matcher;
    }

    // Test custom costing
    {
      Options options;
      create_costing_options(Costing::pedestrian, options);
      meili::MapMatcherFactory factory(root);
      options.set_costing_type(Costing::pedestrian);
      auto matcher = factory.Create(options);
      EXPECT_NE(matcher->costing()->travel_type(), (int)sif::PedestrianType::kSegway)
          << "should not have custom costing options when not set in preferences";

      delete matcher;

      options.mutable_costings()
          ->find(Costing::pedestrian)
          ->second.mutable_options()
          ->set_transport_type("segway");
      matcher = factory.Create(options);

      EXPECT_EQ(matcher->costing()->travel_type(), (int)sif::PedestrianType::kSegway)
          << "should read custom costing options in preferences correctly";

      delete matcher;
    }
  }
}

TEST(MapMatcherFactory, TestMapMatcher) {
  const auto root = test::make_config("/data/valhalla");

  // Nothing special to test for the moment

  meili::MapMatcherFactory factory(root);
  Options options;
  create_costing_options(Costing::auto_, options);
  auto auto_matcher = factory.Create(options);
  options.set_costing_type(Costing::pedestrian);
  auto pedestrian_matcher = factory.Create(options);

  // Share the same pool

  EXPECT_EQ(&auto_matcher->graphreader(), &pedestrian_matcher->graphreader())
      << "graph reader should be shared among matchers";
  EXPECT_EQ(&auto_matcher->candidatequery(), &pedestrian_matcher->candidatequery())
      << "range query should be shared among matchers";

  delete auto_matcher;
  delete pedestrian_matcher;
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
