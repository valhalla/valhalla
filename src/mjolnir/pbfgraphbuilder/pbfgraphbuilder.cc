#include "pbfgraphbuilder.h"
#include "graphbuilder.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"
using namespace CanalTP;  // For OSM pbf reader
using namespace valhalla::mjolnir;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " file_to_read.osm.pbf" << std::endl;
    return 1;
  }

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the GraphConstructor class
  GraphBuilder graphbuilder;
  read_osm_pbf(argv[1], graphbuilder);
  graphbuilder.PrintCounts();

  // Compute node use counts
  graphbuilder.SetNodeUses();

  // Construct edges
  graphbuilder.ConstructEdges();

  // TODO
  // Iterate through edges - tile the end nodes to create connected graph

  return 0;
}

