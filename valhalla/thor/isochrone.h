#ifndef VALHALLA_THOR_ISOCHRONE_H_
#define VALHALLA_THOR_ISOCHRONE_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/midgard/gridded_data.h>
#include <valhalla/proto/tripcommon.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/dijkstras.h>
#include <valhalla/thor/edgestatus.h>

namespace valhalla {
namespace thor {

/**
 * Algorithm to generate an isochrone as a lat,lon grid with time taken to
 * each each grid point. This gridded data can then be contoured to create
 * isolines or contours.
 */
class Isochrone : public Dijkstras {
public:
  /**
   * Constructor.
   * @param max_reserved_labels_count maximum capacity of edgelabels container
   *                                  that allowed to keep reserved
   */
  explicit Isochrone(uint32_t max_reserved_labels_count = std::numeric_limits<uint32_t>::max());

  /**
   * Destructor
   */
  virtual ~Isochrone() {
  }

  /**
   * Compute an isochrone grid. This creates and populates a lat,lon grid with
   * time taken to reach each grid point. This gridded data is then contoured
   * so it can be output as polygons. Multiple locations are allowed as the
   * origins - within some reasonable distance from each other.
   * @param  origin_locs  List of origin locations.
   * @param  max_minutes  Maximum time (minutes) for largest contour
   * @param  graphreader  Graphreader
   * @param  mode_costing List of costing objects
   * @param  mode         Travel mode
   */
  using Dijkstras::Compute;
  std::shared_ptr<const midgard::GriddedData<2>> Compute(Api& api,
                                                         baldr::GraphReader& graphreader,
                                                         const sif::mode_costing_t& mode_costing,
                                                         const sif::TravelMode mode);

  // Compute iso-tile that we can use to generate isochrones. This is used for
  // the reverse direction - construct times for gridded data indicating how
  // long it takes to reach the destination location.
  using Dijkstras::ComputeReverse;
  std::shared_ptr<const midgard::GriddedData<2>>
  ComputeReverse(Api& api,
                 baldr::GraphReader& graphreader,
                 const sif::mode_costing_t& mode_costing,
                 const sif::TravelMode mode);

  /**
   * Compute an isochrone grid for multi-modal routes. This creates and
   * populates a lat,lon grid with time taken to reach each grid point.
   * This gridded data is then contoured so it can be output as polygons.
   * Multiple locations are allowed as the origins - within some reasonable
   * distance from each other.
   * @param  origin_locations  List of origin locations.
   * @param  max_minutes  Maximum time (minutes) for largest contour
   * @param  graphreader  Graphreader
   * @param  mode_costing List of costing objects
   * @param  mode         Travel mode
   */
  using Dijkstras::ComputeMultiModal;
  std::shared_ptr<const midgard::GriddedData<2>>
  ComputeMultiModal(Api& api,
                    baldr::GraphReader& graphreader,
                    const sif::mode_costing_t& mode_costing,
                    const sif::TravelMode mode);

protected:
  // when we expand up to a node we color the cells of the grid that the edge that ends at the
  // node touches
  virtual void ExpandingNode(baldr::GraphReader& graphreader,
                             graph_tile_ptr tile,
                             const baldr::NodeInfo* node,
                             const sif::EdgeLabel& current,
                             const sif::EdgeLabel* previous) override;

  // when the main loop is looking to continue expanding we tell it to terminate here
  virtual ExpansionRecommendation ShouldExpand(baldr::GraphReader& graphreader,
                                               const sif::EdgeLabel& pred,
                                               const InfoRoutingType route_type) override;

  // tell the expansion how many labels to expect and how many buckets to use
  virtual void GetExpansionHints(uint32_t& bucket_count,
                                 uint32_t& edge_label_reservation) const override;

  float shape_interval_; // Interval along shape to mark time
  float max_seconds_;
  float max_meters_;
  std::shared_ptr<midgard::GriddedData<2>> isotile_;

  /**
   * Constructs the isotile - 2-D gridded data containing the time
   * to get to each lat,lng tile.
   * @param  multimodal  True if the route type is multimodal.
   * @param  api         Request information
   * @param  locations   List of origin locations.
   * @param  mode        Travel mode
   */
  void ConstructIsoTile(const bool multimodal, const valhalla::Api& api, const sif::TravelMode mode);

  /**
   * Updates the isotile using the edge information from the predecessor edge
   * label. This is the edge being settled (lowest cost found to the edge).
   * @param  pred         Predecessor edge label (edge being settled).
   * @param  graphreader  Graph reader
   * @param  ll           Lat,lon at the end of the edge.
   * @param  secs0        Seconds at start of the edge.
   */
  void UpdateIsoTile(const sif::EdgeLabel& pred,
                     baldr::GraphReader& graphreader,
                     const midgard::PointLL& ll,
                     const float secs0,
                     const float dist0);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ISOCHRONE_H_
