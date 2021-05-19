#ifndef VALHALLA_MJOLNIR_LINK_CLASSIFICATION_H_
#define VALHALLA_MJOLNIR_LINK_CLASSIFICATION_H_

#include <map>
#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

// Reclassify links (ramps and turn channels). OSM usually classifies links as
// the best classification, while to more effectively create shortcuts it is
// better to "downgrade" link edges to the lower classification.
void ReclassifyLinks(const std::string& ways_file,
                     const std::string& nodes_file,
                     const std::string& edges_file,
                     const std::string& way_nodes_file,
                     const OSMData& osmdata,
                     bool infer_turn_channels);
} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_LINK_CLASSIFICATION_H_
