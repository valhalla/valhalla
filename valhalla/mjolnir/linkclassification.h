#ifndef VALHALLA_MJOLNIR_LINK_CLASSIFICATION_H_
#define VALHALLA_MJOLNIR_LINK_CLASSIFICATION_H_

#include <cstdint>
#include <string>
#include <vector>
#include <map>

#include <valhalla/mjolnir/node_expander.h>
#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/dataquality.h>

namespace valhalla {
namespace mjolnir {

// Get the best classification for any driveable non-link edges from a node.
uint32_t GetBestNonLinkClass(const std::map<Edge, size_t>& edges);

// Reclassify links (ramps and turn channels). OSM usually classifies links as
// the best classification, while to more effectively create shortcuts it is
// better to "downgrade" link edges to the lower classification.
void ReclassifyLinks(const std::string& ways_file,
                     const std::string& nodes_file,
                     const std::string& edges_file,
                     const std::string& way_nodes_file,
                     DataQuality& stats);
}
}
#endif  // VALHALLA_MJOLNIR_LINK_CLASSIFICATION_H_
