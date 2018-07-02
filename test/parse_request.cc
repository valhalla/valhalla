#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "worker.h"

#include <valhalla/proto/directions_options.pb.h>

#include "test.h"

namespace {

///////////////////////////////////////////////////////////////////////////////
// validate by type methods
void validate(const std::string& key,
              const bool expected_value,
              const bool has_pbf_value,
              const bool pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("bool value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key + " bool | expected_value=" +
                             std::string(expected_value ? "true" : "false") +
                             " | found=" + std::string(pbf_value ? "true" : "false"));
  }
}

void validate(const std::string& key,
              const float expected_value,
              const bool has_pbf_value,
              const float pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("float value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " float | expected_value=" + std::to_string(expected_value) +
                             " | found=" + std::to_string(pbf_value));
  }
}

void validate(const std::string& key,
              const uint32_t expected_value,
              const bool has_pbf_value,
              const uint32_t pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("uint32_t value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " uint32_t | expected_value=" + std::to_string(expected_value) +
                             " | found=" + std::to_string(pbf_value));
  }
}

void validate(const std::string& key,
              const std::string& expected_value,
              const bool has_pbf_value,
              const std::string& pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("string value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key + " string | expected_value=" + expected_value +
                             " | found=" + pbf_value);
  }
}

void validate(const std::string& key,
              const valhalla::odin::ShapeMatch expected_value,
              const bool has_pbf_value,
              const valhalla::odin::ShapeMatch pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("ShapeMatch value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " ShapeMatch | expected_value=" + ShapeMatch_Name(expected_value) +
                             " | found=" + ShapeMatch_Name(pbf_value));
  }
}

void validate(const std::string& key,
              const valhalla::odin::FilterAction expected_value,
              const bool has_pbf_value,
              const valhalla::odin::FilterAction pbf_value) {
  if (!has_pbf_value) {
    throw std::runtime_error("FilterAction value not found in pbf for key=" + key);
  }
  if (pbf_value != expected_value) {
    throw std::runtime_error("Incorrect " + key +
                             " FilterAction | expected_value=" + FilterAction_Name(expected_value) +
                             " | found=" + FilterAction_Name(pbf_value));
  }
}

void validate(const std::string& key,
              const std::vector<std::string>& expected_values,
              const bool has_pbf_values,
              const google::protobuf::RepeatedPtrField<std::string>& pbf_values) {
  if (!has_pbf_values) {
    throw std::runtime_error("string values not found in pbf for key=" + key);
  }
  if (expected_values.size() != pbf_values.size()) {
    throw std::runtime_error("invalid count in pbf for key=" + key);
  }
  for (size_t i = 0; i < expected_values.size(); ++i) {
    if (pbf_values.Get(i) != expected_values.at(i)) {
      throw std::runtime_error("Incorrect " + key + " string | expected_value=" +
                               expected_values.at(i) + " | found=" + pbf_values.Get(i));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// get request methods
std::string get_request_str(const std::string& key, const bool expected_value) {
  return R"({")" + key + R"(":)" + std::string(expected_value ? "true" : "false") + R"(})";
}

std::string get_request_str(const std::string& key, const float expected_value) {
  return R"({")" + key + R"(":)" + std::to_string(expected_value) + R"(})";
}

std::string
get_request_str(const std::string& parent_key, const std::string& key, const float expected_value) {
  return R"({")" + parent_key + R"(":{")" + key + R"(":)" + std::to_string(expected_value) + R"(}})";
}

std::string get_request_str(const std::string& key, const uint32_t expected_value) {
  return R"({")" + key + R"(":)" + std::to_string(expected_value) + R"(})";
}

std::string get_request_str(const std::string& key, const std::string& expected_value) {
  return R"({")" + key + R"(":")" + expected_value + R"("})";
}

std::string get_request_str(const std::string& key, const valhalla::odin::ShapeMatch expected_value) {
  return R"({")" + key + R"(":")" + ShapeMatch_Name(expected_value) + R"("})";
}

std::string get_request_str(const std::string& parent_key,
                            const std::string& key,
                            const valhalla::odin::FilterAction expected_value) {
  return R"({")" + parent_key + R"(":{")" + key + R"(":")" + FilterAction_Name(expected_value) +
         R"("}})";
}

std::string get_request_str(const std::string& parent_key,
                            const std::string& key,
                            const std::vector<std::string>& expected_values) {
  std::string request_str = R"({")" + parent_key + R"(":{")" + key + R"(":[)";
  bool first = true;
  for (const auto& expected_value : expected_values) {
    if (first) {
      request_str += R"(")";
      first = false;
    } else {
      request_str += R"(,")";
    }
    request_str += expected_value + R"(")";
  }
  request_str += R"(]}})";
  return request_str;
}

valhalla::valhalla_request_t get_request(const std::string& request_str,
                                         const valhalla::odin::DirectionsOptions::Action action) {
  // std::cout << ">>>>> request_str=" << request_str << "<<<<<" << std::endl;
  valhalla::valhalla_request_t request;
  request.parse(request_str, action);
  return request;
}

///////////////////////////////////////////////////////////////////////////////
// test parsing methods
void test_polygons_parsing(const bool expected_value,
                           const valhalla::odin::DirectionsOptions::Action action =
                               valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "polygons";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_polygons(), request.options.polygons());
}

void test_denoise_parsing(const float expected_value,
                          const valhalla::odin::DirectionsOptions::Action action =
                              valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "denoise";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_denoise(), request.options.denoise());
}

void test_generalize_parsing(const float expected_value,
                             const valhalla::odin::DirectionsOptions::Action action =
                                 valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "generalize";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_generalize(), request.options.generalize());
}

void test_show_locations_parsing(const bool expected_value,
                                 const valhalla::odin::DirectionsOptions::Action action =
                                     valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "show_locations";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_show_locations(),
           request.options.show_locations());
}

void test_shape_match_parsing(const valhalla::odin::ShapeMatch expected_value,
                              const valhalla::odin::DirectionsOptions::Action action) {
  const std::string key = "shape_match";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_shape_match(), request.options.shape_match());
}

void test_best_paths_parsing(const uint32_t expected_value,
                             const valhalla::odin::DirectionsOptions::Action action =
                                 valhalla::odin::DirectionsOptions::isochrone) {
  const std::string key = "best_paths";
  valhalla::valhalla_request_t request = get_request(get_request_str(key, expected_value), action);
  validate(key, expected_value, request.options.has_best_paths(), request.options.best_paths());
}

void test_gps_accuracy_parsing(const float expected_value,
                               const valhalla::odin::DirectionsOptions::Action action =
                                   valhalla::odin::DirectionsOptions::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "gps_accuracy";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_gps_accuracy(), request.options.gps_accuracy());
}

void test_search_radius_parsing(const float expected_value,
                                const valhalla::odin::DirectionsOptions::Action action =
                                    valhalla::odin::DirectionsOptions::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "search_radius";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_search_radius(), request.options.search_radius());
}

void test_turn_penalty_factor_parsing(const float expected_value,
                                      const valhalla::odin::DirectionsOptions::Action action =
                                          valhalla::odin::DirectionsOptions::isochrone) {
  const std::string parent_key = "trace_options";
  const std::string key = "turn_penalty_factor";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_turn_penalty_factor(),
           request.options.turn_penalty_factor());
}

void test_filter_action_parsing(const valhalla::odin::FilterAction expected_value,
                                const valhalla::odin::DirectionsOptions::Action action =
                                    valhalla::odin::DirectionsOptions::trace_attributes) {
  const std::string parent_key = "filters";
  const std::string key = "action";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_value), action);
  validate(key, expected_value, request.options.has_filter_action(), request.options.filter_action());
}

void test_filter_attributes_parsing(const std::vector<std::string>& expected_values,
                                    const valhalla::odin::DirectionsOptions::Action action =
                                        valhalla::odin::DirectionsOptions::trace_attributes) {
  const std::string parent_key = "filters";
  const std::string key = "attributes";
  valhalla::valhalla_request_t request =
      get_request(get_request_str(parent_key, key, expected_values), action);
  validate(key, expected_values, (request.options.filter_attributes_size() > 0),
           request.options.filter_attributes());
}

///////////////////////////////////////////////////////////////////////////////
// test by key methods
void test_polygons() {
  test_polygons_parsing(true);
  test_polygons_parsing(false);
}

void test_denoise() {
  test_denoise_parsing(0.0f);
  test_denoise_parsing(0.5f);
  test_denoise_parsing(1.0f);
}

void test_generalize() {
  test_generalize_parsing(20.f);
  test_generalize_parsing(50.f);
}

void test_show_locations() {
  test_show_locations_parsing(true);
  test_show_locations_parsing(false);
}

void test_shape_match() {
  test_shape_match_parsing(valhalla::odin::ShapeMatch::map_snap,
                           valhalla::odin::DirectionsOptions::trace_route);
  test_shape_match_parsing(valhalla::odin::ShapeMatch::map_snap,
                           valhalla::odin::DirectionsOptions::trace_attributes);
  test_shape_match_parsing(valhalla::odin::ShapeMatch::edge_walk,
                           valhalla::odin::DirectionsOptions::trace_route);
  test_shape_match_parsing(valhalla::odin::ShapeMatch::edge_walk,
                           valhalla::odin::DirectionsOptions::trace_attributes);
}

void test_best_paths() {
  test_best_paths_parsing(1);
  test_best_paths_parsing(2);
  test_best_paths_parsing(4);
}

void test_gps_accuracy() {
  test_gps_accuracy_parsing(5.f);
  test_gps_accuracy_parsing(30.f);
}

void test_search_radius() {
  test_search_radius_parsing(10.f);
  test_search_radius_parsing(40.f);
}

void test_turn_penalty_factor() {
  test_turn_penalty_factor_parsing(50.f);
  test_turn_penalty_factor_parsing(100.f);
}

void test_filter_action() {
  test_filter_action_parsing(valhalla::odin::FilterAction::exclude);
  test_filter_action_parsing(valhalla::odin::FilterAction::include);
}

void test_filter_attributes() {
  test_filter_attributes_parsing({"edge.names", "edge.id", "edge.weighted_grade", "edge.speed"});
}

} // namespace

int main() {
  test::suite suite("parse_request");

  // TODO repeated Contour contours

  // polygons
  suite.test(TEST_CASE(test_polygons));

  // denoise
  suite.test(TEST_CASE(test_denoise));

  // generalize
  suite.test(TEST_CASE(test_generalize));

  // show_locations
  suite.test(TEST_CASE(test_show_locations));

  // shape_match
  suite.test(TEST_CASE(test_shape_match));

  // best_paths
  suite.test(TEST_CASE(test_best_paths));

  // gps_accuracy
  suite.test(TEST_CASE(test_gps_accuracy));

  // search_radius
  suite.test(TEST_CASE(test_search_radius));

  // turn_penalty_factor
  suite.test(TEST_CASE(test_turn_penalty_factor));

  // filter_action
  suite.test(TEST_CASE(test_filter_action));

  // filter_attributes
  suite.test(TEST_CASE(test_filter_attributes));

  return suite.tear_down();
}
