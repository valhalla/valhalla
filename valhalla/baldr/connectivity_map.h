#ifndef VALHALLA_BALDR_CONNECTIVITY_MAP_H_
#define VALHALLA_BALDR_CONNECTIVITY_MAP_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace valhalla {
namespace baldr {
// TODO: maintain consistent coloring of regions despite the connectivity changing
class connectivity_map_t {
public:
  /**
   * Constructs the connectivity map
   * @param pt   the ptree sub child labeled mjolnir in the valhalla json config
   * @param graphreader optional pointer to the graph reader to use. If null, then the reader will be constructed using pt.
   */
  connectivity_map_t(const boost::property_tree::ptree& pt, const std::shared_ptr<GraphReader>& graph_reader = {});

  /**
   * Returns the color for the given graphid
   *
   * @param id      the graphid
   * @return color  the color
   */
  size_t get_color(const GraphId& id) const;

  /**
   * Returns the colors for the given level,point,radius
   *
   * @param hierarchy_level  the hierarchy level whos connectivity you are querying
   * @param location           the center of the circle
   * @param radius           the radius of the circle
   * @return colors          the colors of the tiles that intersect this circle at this level
   */
  std::unordered_set<size_t>
  get_colors(uint32_t hierarchy_level, const baldr::PathLocation& location, float radius) const;

  /**
   * Returns the geojson representing the connectivity map
   *
   * @param hierarchy_level the hierarchy level whos connectivity you want to see
   * @return string         the geojson
   */
  std::string to_geojson(const uint32_t hierarchy_level) const;

  /**
   * Returns the vector of colors (one per tile) representing the connectivity map
   *
   * @param hierarchy_level the hierarchy level whos connectivity you want to see
   * @return vector         the vector of colors per tile
   */
  std::vector<size_t> to_image(const uint32_t hierarchy_level) const;

  /**
   * Does data exist for a level.
   * @param level Tile hierarchy level.
   * @return Returns true if the level has data, false if it does not (no tiles present)
   */
  bool has_data(const uint32_t level) const {
    auto c = colors.find(level);
    return (c == colors.end()) ? false : c->second.size() > 0;
  }

private:
  uint32_t transit_level;
  // this is a map(tile_level, map(tile_id, tile_color))
  std::unordered_map<uint32_t, std::unordered_map<uint32_t, size_t>> colors;
};
} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_CONNECTIVITY_MAP_H_
