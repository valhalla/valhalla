#include "test.h"

#include "mjolnir/graphbuilder.h"

#include <sstream>
#include <string>
#include <thread>
#include <algorithm>
#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

using namespace std;
using namespace valhalla::mjolnir;

namespace {

class test_graph_builder : public GraphBuilder {
 public:
  using GraphBuilder::GraphBuilder;
  using GraphBuilder::threads_;
};

shared_ptr<test_graph_builder> make_builder(boost::optional<unsigned int> threads) {
  std::stringstream json;
  if(threads)
    json << "{\"concurrency\": " << threads << ", \"hierarchy\": {\"tile_dir\": \"/data/valhalla\",\"levels\": [{\"name\": \"local\", \"level\": 2, \"size\": 0.25}]}, \
              \"tagtransform\": {\"node_script\": \"conf/vertices.lua\", \"node_function\": \"nodes_proc\", \"way_script\": \"conf/edges.lua\", \"way_function\": \"ways_proc\"}}";
  else
    json << "{\"hierarchy\": {\"tile_dir\": \"/data/valhalla\",\"levels\": [{\"name\": \"local\", \"level\": 2, \"size\": 0.25}]}, \
              \"tagtransform\": {\"node_script\": \"conf/vertices.lua\", \"node_function\": \"nodes_proc\", \"way_script\": \"conf/edges.lua\", \"way_function\": \"ways_proc\"}}";
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(json, pt);
  return shared_ptr<test_graph_builder>(new test_graph_builder(pt));
}

}

void TestThread() {

  if(make_builder(static_cast<unsigned int>(0))->threads_ != 1)
    throw std::runtime_error("Expected 1 thread but got: " + to_string(make_builder(static_cast<unsigned int>(0))->threads_));
  for(unsigned int i = 1; i < 5; ++i) {
    if(make_builder(i)->threads_ != i)
      throw std::runtime_error("Expected 1 thread but got: " + to_string(make_builder(i)->threads_));
  }
  auto system_threads = std::max(static_cast<unsigned int>(1), std::thread::hardware_concurrency());
  if(make_builder(boost::none)->threads_ != system_threads)
    throw std::runtime_error("Expected " + to_string(system_threads) +  " thread but got: " + to_string(make_builder(boost::none)->threads_));
}

int main() {
  test::suite suite("graphbuilder");

  // Test setting and getting on random sizes of bit tables
  suite.test(TEST_CASE(TestThread));
  //TODO: sweet jesus add more tests of this class!

  return suite.tear_down();
}
