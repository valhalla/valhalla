#include <string>

#include "pbfgraphbuilder.h"
#include "graphbuilder.h"

// Use open source PBF reader from:
//     https://github.com/CanalTP/libosmpbfreader
#include "osmpbfreader.h"
using namespace CanalTP;  // For OSM pbf reader
using namespace valhalla::mjolnir;

#include <ostream>
#include "geo/point2.h"
#include "geo/aabb2.h"
#include "geo/polyline2.h"

using namespace valhalla::geo;

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

  // Remove unused node (maybe this can recover memory?)
  graphbuilder.RemoveUnusedNodes();

  // Tile the nodes
  unsigned int level = 2;    // Local hierarchy - TODO - configurable
  float tilesize = 0.25f;    // TODO - configurable
  graphbuilder.TileNodes(tilesize, level);

  // TODO - make bounding box and tilesize configurable (via properties file?)
  // Iterate through edges - tile the end nodes to create connected graph
  std::string outputdir = "/home/dave/data/tiles/";
  graphbuilder.BuildLocalTiles(outputdir, level);

  return 0;
}

