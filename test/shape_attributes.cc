#include "test.h"

#include "midgard/distanceapproximator.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::midgard;

namespace {

// fake config
const auto conf = test::make_config("test/data/utrecht_tiles");

TEST(ShapeAttributes, test_shape_attributes_included) {
  tyr::actor_t actor(conf);

  auto result_json = actor.trace_attributes(
      R"({"shape":[
        {"lat":52.09110,"lon":5.09806},
        {"lat":52.09050,"lon":5.09769},
        {"lat":52.09098,"lon":5.09679}
      ],"costing":"auto","shape_match":"map_snap",
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})");

  rapidjson::Document doc;
  doc.Parse(result_json);
  EXPECT_FALSE(doc.HasParseError()) << "Could not parse json response";

  auto shape =
      midgard::decode<std::vector<PointLL>>(rapidjson::Pointer("/shape").Get(doc)->GetString());
  auto shape_attributes_time = rapidjson::Pointer("/shape_attributes/time").Get(doc)->GetArray();
  auto shape_attributes_length = rapidjson::Pointer("/shape_attributes/length").Get(doc)->GetArray();
  auto shape_attributes_speed = rapidjson::Pointer("/shape_attributes/speed").Get(doc)->GetArray();
  auto edges = rapidjson::Pointer("/edges").Get(doc)->GetArray();

  EXPECT_EQ(shape_attributes_time.Size(), shape.size() - 1);
  EXPECT_EQ(shape_attributes_length.Size(), shape.size() - 1);
  EXPECT_EQ(shape_attributes_speed.Size(), shape.size() - 1);

  // Measures the length between point
  for (size_t i = 1; i < shape.size(); i++) {
    auto distance = shape[i].Distance(shape[i - 1]) * .001f;

    // Measuring that the length between shape pts is approx. to the shape attributes length
    EXPECT_NEAR(distance, shape_attributes_length[i - 1].GetFloat(), .01f);
  }

  // Assert that the shape attributes (time, length, speed) are equal to their corresponding edge
  // attributes
  for (rapidjson::SizeType e = 0; e < edges.Size(); e++) {
    auto edge_length = edges[e]["length"].GetDouble();
    auto edge_speed = edges[e]["speed"].GetDouble();

    double sum_times = 0;
    double sum_lengths = 0;
    for (int j = edges[e]["begin_shape_index"].GetInt(); j < edges[e]["end_shape_index"].GetInt();
         j++) {
      sum_times += shape_attributes_time[j].GetDouble();
      sum_lengths += shape_attributes_length[j].GetDouble();

      EXPECT_NEAR(edge_speed, shape_attributes_speed[j].GetDouble(), .15);
    }

    // Can't assert that sum of shape times equals edge's elapsed_time because elapsed_time includes
    // transition costs and shape times do not.
    EXPECT_NEAR(3600 * edge_length / edge_speed, sum_times, .1);
    EXPECT_NEAR(edge_length, sum_lengths, .1);
  }
}

TEST(ShapeAttributes, test_shape_attributes_duplicated_point) {
  tyr::actor_t actor(conf);

  auto result_json = actor.trace_attributes(
      R"({"shape":[
        {"lat":52.09110,"lon":5.09806},
        {"lat":52.09110,"lon":5.09806},
        {"lat":52.09050,"lon":5.09769},
        {"lat":52.09098,"lon":5.09679}
      ],"costing":"auto","shape_match":"map_snap",
      "filters":{"attributes":["edge.length","edge.speed","edge.begin_shape_index",
      "edge.end_shape_index","shape","shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
      "action":"include"}})");

  rapidjson::Document doc;
  doc.Parse(result_json);
  EXPECT_FALSE(doc.HasParseError()) << "Could not parse json response";

  auto shape =
      midgard::decode<std::vector<PointLL>>(rapidjson::Pointer("/shape").Get(doc)->GetString());
  auto shape_attributes_time = rapidjson::Pointer("/shape_attributes/time").Get(doc)->GetArray();
  auto shape_attributes_length = rapidjson::Pointer("/shape_attributes/length").Get(doc)->GetArray();
  auto shape_attributes_speed = rapidjson::Pointer("/shape_attributes/speed").Get(doc)->GetArray();
  auto edges = rapidjson::Pointer("/edges").Get(doc)->GetArray();

  EXPECT_EQ(shape_attributes_time.Size(), shape.size() - 1);
  EXPECT_EQ(shape_attributes_length.Size(), shape.size() - 1);
  EXPECT_EQ(shape_attributes_speed.Size(), shape.size() - 1);

  // Measures the length between point
  for (size_t i = 1; i < shape.size(); i++) {
    auto distance = shape[i].Distance(shape[i - 1]) * .001f;

    // Measuring that the length between shape pts is approx. to the shape attributes length
    EXPECT_NEAR(distance, shape_attributes_length[i - 1].GetFloat(), .01f);
  }

  // Assert that the shape attributes (time, length, speed) are equal to their corresponding edge
  // attributes
  for (rapidjson::SizeType e = 0; e < edges.Size(); e++) {
    auto edge_length = edges[e]["length"].GetDouble();
    auto edge_speed = edges[e]["speed"].GetDouble();

    double sum_times = 0;
    double sum_lengths = 0;
    for (int j = edges[e]["begin_shape_index"].GetInt(); j < edges[e]["end_shape_index"].GetInt();
         j++) {
      sum_times += shape_attributes_time[j].GetDouble();
      sum_lengths += shape_attributes_length[j].GetDouble();

      EXPECT_NEAR(edge_speed, shape_attributes_speed[j].GetDouble(), .15);
    }

    // Can't assert that sum of shape times equals edge's elapsed_time because elapsed_time includes
    // transition costs and shape times do not.
    EXPECT_NEAR(3600 * edge_length / edge_speed, sum_times, .1);
    EXPECT_NEAR(edge_length, sum_lengths, .1);
  }
}

TEST(ShapeAttributes, test_shape_attributes_no_turncosts) {
  tyr::actor_t actor(conf);
  auto result_json = actor.trace_attributes(
      R"({"shape":[
         {"lat":52.09110,"lon":5.09806},
         {"lat":52.091050,"lon":5.097556}
        ],"costing":"auto","shape_match":"map_snap",
        "filters":{"attributes":["edge.length","edge.speed","node.elapsed_time",
          "edge.begin_shape_index","edge.end_shape_index","shape",
          "shape_attributes.length","shape_attributes.time","shape_attributes.speed"],
        "action":"include"}})");

  rapidjson::Document doc;
  doc.Parse(result_json);
  EXPECT_FALSE(doc.HasParseError()) << "Could not parse json response";

  auto shape =
      midgard::decode<std::vector<PointLL>>(rapidjson::Pointer("/shape").Get(doc)->GetString());
  auto shape_attributes_time = rapidjson::Pointer("/shape_attributes/time").Get(doc)->GetArray();
  auto shape_attributes_length = rapidjson::Pointer("/shape_attributes/length").Get(doc)->GetArray();
  auto shape_attributes_speed = rapidjson::Pointer("/shape_attributes/speed").Get(doc)->GetArray();
  auto edges = rapidjson::Pointer("/edges").Get(doc)->GetArray();

  EXPECT_EQ(shape_attributes_time.Size(), shape.size() - 1);
  EXPECT_EQ(shape_attributes_length.Size(), shape.size() - 1);
  EXPECT_EQ(shape_attributes_speed.Size(), shape.size() - 1);

  // Measures the length between point
  for (size_t i = 1; i < shape.size(); i++) {
    auto distance = shape[i].Distance(shape[i - 1]) * .001f;

    // Measuring that the length between shape pts is approx. to the shape attributes length
    EXPECT_NEAR(distance, shape_attributes_length[i - 1].GetFloat(), .01f);
  }

  // Assert that the shape attributes (time, length, speed) are equal to their corresponding edge
  // attributes
  auto edge_length = edges[0]["length"].GetDouble();
  auto edge_speed = edges[0]["speed"].GetDouble();
  auto edge_elapsed_time = edges[0]["end_node"]["elapsed_time"].GetDouble();

  double sum_times = 0;
  double sum_lengths = 0;

  sum_times += shape_attributes_time[0].GetDouble();
  sum_lengths += shape_attributes_length[0].GetDouble();

  EXPECT_NEAR(edge_speed, shape_attributes_speed[0].GetDouble(), .15);

  // Can't assert that sum of shape times equals edge's elapsed_time because elapsed_time includes
  // transition costs and shape times do not.
  EXPECT_NEAR(edge_elapsed_time, sum_times, .1);
  EXPECT_NEAR(edge_length, sum_lengths, .1);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
