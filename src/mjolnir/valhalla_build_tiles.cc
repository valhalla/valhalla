#include <string>
#include <vector>
#include <iostream>

#include "midgard/sequence.h"
#include "mjolnir/osmdata.h"
#include "mjolnir/node_expander.h"

using namespace valhalla::mjolnir;
using namespace valhalla::midgard;


int main(int argc, char** argv) {
  std::cout << sizeof(OSMWayNode) << std::endl;
  return 0;
//  sequence<OSMWayNode> seq("seq_origin.bin", true);
//  for (size_t i = 0; i < 44739242 * 5; ++i) {
//    OSMWayNode n;
//    for (size_t j = 0; j < sizeof(n); ++j) {
//      *(reinterpret_cast<char*>(&n) + j) = rand();
//    }
//    seq.push_back(n);
//  }
  sequence<OSMWayNode> seq("seq.bin");
  seq.sort([](const OSMWayNode& lhs, const OSMWayNode& rhs) {
    return lhs.node.osmid_ < rhs.node.osmid_;
  });
}
