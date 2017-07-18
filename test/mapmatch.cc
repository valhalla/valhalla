#include "test.h"

#include <random>
#include <utility>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "meili/map_matcher_factory.h"
#include "meili/map_matcher.h"
#include "tyr/actor.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "midgard/logging.h"
#include "baldr/json.h"
#include "midgard/distanceapproximator.h"
#include "midgard/polyline2.h"
#include "mjolnir/util.h"

using namespace valhalla;

namespace std {
  std::string to_string(const midgard::PointLL& p) {
    return "[" + to_string(p.first) + "," + to_string(p.second) + "]";
  }
}

namespace {

  boost::property_tree::ptree json_to_pt(const std::string& json) {
    std::stringstream ss; ss << json;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt;
  }

  //fake config
  const auto conf = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["locate","route","one_to_many","many_to_one","many_to_many","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0}
    },
    "thor":{"logging":{"long_request": 110}},
    "skadi":{"actons":["height"],"logging":{"long_request": 5}},
    "meili":{"customizable": ["breakage_distance"],
             "mode":"auto","grid":{"cache_size":100240,"size":500},
             "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,
                        "interpolation_distance":10,"max_route_distance_factor":3,"max_search_radius":100,
                        "route":true,"search_radius":50,"sigma_z":4.07,"turn_penalty_factor":200}},
    "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");

  std::string json_escape(const std::string& unescaped) {
    std::stringstream ss;
    baldr::json::OstreamVisitor v(ss);
    v(unescaped);
    std::string escaped = ss.str().substr(1);
    escaped.pop_back();
    return escaped;
  }

  template <class container_t>
  std::string print(const container_t& container) {
    std::string output;
    for(const auto& e : container)
      output += std::to_string(e) + ",";
    return output;
  }

  template <typename T>
  struct ring_queue_t {
    ring_queue_t(size_t limit):limit(limit), i(0) {
      v.reserve(limit);
    }
    void emplace_back(T&& t){
      if(v.size() < limit) v.emplace_back(t);
      else v[i] = t;
      i = (i + 1) % limit;
    };
    const T& front() const { return i < v.size() ? v[i] : v[0]; }
    const T& back() const { return v[i - 1]; }
    size_t size() const { return v.size(); }
    bool full() const { return v.size() == limit; }

    size_t limit, i;
    std::vector<T> v;

    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    iterator begin() { return v.begin(); }
    const_iterator begin() const { return v.begin(); }
    iterator end() { return v.end(); }
    const_iterator end() const { return v.end(); }
  };

  std::vector<midgard::PointLL> resample_at_1hz(const boost::property_tree::ptree& edges, const std::vector<midgard::PointLL>& shape) {
    std::vector<midgard::PointLL> resampled;
    float time_remainder = 0.0;
    for(const auto& edge_item: edges) {
      const auto& edge = edge_item.second;
      //get the portion of the shape that applies to this edge
      std::vector<midgard::PointLL> edge_shape(shape.cbegin() + edge.get<size_t>("begin_shape_index"),
        shape.cbegin() + edge.get<size_t>("end_shape_index") + 1);
      //get the speed of this edge
      auto meters = midgard::Polyline2<PointLL>::Length(edge_shape);
      auto speed = (edge.get<float>("speed") * 1e3) / 3600.f;
      //trim the shape to account of the portion of the previous second that bled onto this edge
      auto to_trim = speed * time_remainder;
      auto trimmed = midgard::trim_polyline(edge_shape.cbegin(), edge_shape.cend(), to_trim / meters, 1.f);
      //resample it at 1 second intervals
      auto second_interval = midgard::resample_spherical_polyline(trimmed, speed, false);
      resampled.insert(resampled.end(), second_interval.begin(), second_interval.end());
      //figure out how much of the last second will bleed into the next edge
      double intpart;
      time_remainder = std::modf((meters - to_trim) / speed, &intpart);
    }
    return resampled;
  }

  std::vector<midgard::PointLL> simulate_gps(const boost::property_tree::ptree& edges, const std::vector<midgard::PointLL>& shape,
      float smoothing = 30, float accuracy = 5.f, size_t sample_rate = 1) {
    //resample the coords along a given edge at one second intervals
    auto resampled = resample_at_1hz(edges, shape);

    //a way to get noise but only allow for slow change
    std::default_random_engine generator(0);
    std::uniform_real_distribution<float> distribution(-1, 1);
    ring_queue_t<std::pair<float, float> > noises(smoothing);
    auto get_noise = [&]() {
      //we generate a vector whose magnitude is no more than accuracy
      auto lon_adj = distribution(generator);
      auto lat_adj = distribution(generator);
      auto len = std::sqrt((lon_adj * lon_adj) + (lat_adj * lat_adj));
      lon_adj /= len; lat_adj /= len; //norm
      auto scale = (distribution(generator) + 1.f) / 2.f;
      lon_adj *= scale * accuracy;  lat_adj *= scale * accuracy; //random scale <= accuracy
      noises.emplace_back(std::make_pair(lon_adj, lat_adj));
      //average over last n to smooth
      std::pair<float, float> noise{0, 0};
      std::for_each(noises.begin(), noises.end(),
        [&noise](const std::pair<float, float>& n) { noise.first += n.first; noise.second += n.second; });
      noise.first /= noises.size();
      noise.second /= noises.size();
      return noise;
    };
    //fill up the noise queue so the first points arent unsmoothed
    while(!noises.full()) get_noise();

    //for each point of the 1hz shape
    std::vector<midgard::PointLL> simulated;
    for(size_t i = 0; i < resampled.size(); ++i) {
      const auto& p = resampled[i];
      //is this a harmonic of the desired sampling rate
      if(i % sample_rate == 0) {
        //meters of noise with extremely low likelihood its larger than accuracy
        auto noise = get_noise();
        //use the number of meters per degree in both axis to offset the point by the noise
        auto metersPerDegreeLon = DistanceApproximator::MetersPerLngDegree(p.second);
        simulated.emplace_back(midgard::PointLL(p.first + noise.first / metersPerDegreeLon,
          p.second + noise.second / kMetersPerDegreeLat));
      }
    }
    return simulated;
  }

  int seed = 973; int bound = 81;
  std::string make_test_case() {
    static std::default_random_engine generator(seed);
    static std::uniform_real_distribution<float> distribution(0, 1);
    PointLL start,end;
    float distance = 0;
    do {
      //get two points in and around utrecht
      start = PointLL(5.0819f + .053f * distribution(generator), 52.0698f + .0334f * distribution(generator));
      end = PointLL(5.0819f + .053f * distribution(generator), 52.0698f + .0334f * distribution(generator));
      distance = start.Distance(end);
      //try again if they are too close or too far apart
    }while(distance < 1000 || distance > 2000);
    return R"({"costing":"auto","locations":[{"lat":)" + std::to_string(start.second) + R"(,"lon":)" + std::to_string(start.first) +
      R"(},{"lat":)" + std::to_string(end.second) + R"(,"lon":)" + std::to_string(end.first) + "}]}";
  }

  void test_matcher() {
    //some edges should have no matches and most will have no segments
    tyr::actor_t actor(conf, true);
    int tested = 0;
    while(tested < bound) {
      //get a route shape
      auto test_case = make_test_case();
      boost::property_tree::ptree route;
      try { route = json_to_pt(actor.route(tyr::ROUTE, test_case)); }
      catch (...) { continue; }
      auto encoded_shape = route.get_child("trip.legs").front().second.get<std::string>("shape");
      auto shape = midgard::decode<std::vector<midgard::PointLL> >(encoded_shape);
      //skip any routes that have loops in them as edge walk fails in that case...
      //TODO: fix edge walk
      std::unordered_set<std::string> names;
      bool looped = false;
      const auto& maneuvers = route.get_child("trip.legs").front().second.get_child("maneuvers");
      for(const auto& maneuver : maneuvers) {
        if(maneuver.second.find("street_names") == maneuver.second.not_found())
          continue;
        for(const auto& name : maneuver.second.get_child("street_names"))
          looped = looped || !names.insert(name.second.get_value<std::string>()).second;
      }
      if(looped)
        continue;
      //get the edges along that route shape
      auto walked = json_to_pt(actor.trace_attributes(
        R"({"costing":"auto","shape_match":"edge_walk","encoded_polyline":")" + json_escape(encoded_shape) + "\"}"));
      std::vector<uint64_t> walked_edges;
      for(const auto& edge : walked.get_child("edges"))
        walked_edges.push_back(edge.second.get<uint64_t>("id"));
      //simulate gps from the route shape
      auto simulation = simulate_gps(walked.get_child("edges"), shape, 50, 100.f);
      auto encoded_simulation = midgard::encode(simulation);
      //get a trace-attributes from the simulated gps
      auto matched = json_to_pt(actor.trace_attributes(
        R"({"costing":"auto","shape_match":"map_snap","encoded_polyline":")" + json_escape(encoded_simulation) + "\"}"));
      std::vector<uint64_t> matched_edges;
      for(const auto& edge : matched.get_child("edges"))
        matched_edges.push_back(edge.second.get<uint64_t>("id"));
      //because of noise we can have off by 1 happen at the beginning or end so we trim to make sure
      auto walked_it = std::search(walked_edges.begin(), walked_edges.end(), matched_edges.begin() + 1, matched_edges.end() - 1);
      if(walked_it == walked_edges.end()) {
        std::cout << "route shape: " << print(shape) << std::endl;
        std::cout << "faked gps: " << print(simulation) << std::endl;
        throw std::logic_error("The match did not match the walk.\nExpected: " + print(walked_edges) + "\nGot:      " + print(matched_edges));
      }
      std::cout << "Iteration " << tested << " complete" << std::endl;
      ++tested;
    }
  }


}

int main(int argc, char* argv[]) {
  test::suite suite("map matcher");
  midgard::logging::Configure({{"type", ""}}); //silence logs
  if(argc > 1)
    seed = std::stoi(argv[1]);
  if(argc > 2)
    bound = std::stoi(argv[2]);

  suite.test(TEST_CASE(test_matcher));

  return suite.tear_down();
}
