#ifndef VALHALLA_THOR_ISOCHRONE_H_
#define VALHALLA_THOR_ISOCHRONE_H_

#include <cstdint>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

#include <valhalla/midgard/gridded_data.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>

namespace valhalla {
namespace thor {

/**
 * Algorithm to generate an isochrone as a lat,lon grid with time taken to
 * each each grid point. This gridded data can then be contoured to create
 * isolines or contours.
 */
class Isochrone {
 public:
  /**
   * Constructor.
   */
  Isochrone();

  /**
   * Destructor
   */
  virtual ~Isochrone();

  /**
   * Clear the temporary memory (adjacency list, edgestatus, edgelabels)
   */
  void Clear();

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
  std::shared_ptr<const GriddedData<midgard::PointLL> > Compute(
          std::vector<baldr::PathLocation>& origin_locs,
          const unsigned int max_minutes,
          baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode);

  // Compute iso-tile that we can use to generate isochrones. This is used for
  // the reverse direction - construct times for gridded data indicating how
  // long it takes to reach the destination location.
  std::shared_ptr<const GriddedData<midgard::PointLL> > ComputeReverse(
               std::vector<baldr::PathLocation>& dest_locations,
               const unsigned int max_minutes,
               baldr::GraphReader& graphreader,
               const std::shared_ptr<sif::DynamicCost>* mode_costing,
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
  std::shared_ptr<const GriddedData<midgard::PointLL> > ComputeMultiModal(
               std::vector<baldr::PathLocation>& origin_locations,
               const unsigned int max_minutes,
               baldr::GraphReader& graphreader,
               const std::shared_ptr<sif::DynamicCost>* mode_costing,
               const sif::TravelMode mode);

 protected:
  float shape_interval_;        // Interval along shape to mark time
  sif::TravelMode mode_;        // Current travel mode
  uint32_t access_mode_;        // Access mode used by the costing method

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> costing_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::EdgeLabel> edgelabels_;
  std::vector<sif::BDEdgeLabel> bdedgelabels_;
  std::vector<sif::MMEdgeLabel> mmedgelabels_;

  // Adjacency list - approximate double bucket sort
  std::shared_ptr<baldr::DoubleBucketQueue> adjacencylist_;

  // Edge status. Mark edges that are in adjacency list or settled.
  std::shared_ptr<EdgeStatus> edgestatus_;

  // Isochrone gridded time data
  std::shared_ptr<GriddedData<midgard::PointLL> > isotile_;

  /**
   * Initialize prior to computing the isochrones. Creates adjacency list,
   * edgestatus support, and reserves edgelabels.
   * @param bucketsize  Adjacency list bucket size.
   */
  void Initialize(const uint32_t bucketsize);

  /**
   * Initialize prior to computing reverse isochrones. Creates adjacency list,
   * edgestatus support, and reserves edgelabels.
   * @param bucketsize  Adjacency list bucket size.
   */
  void InitializeReverse(const uint32_t bucketsize);

  /**
   * Initialize prior to computing mulit-modal isochrones. Creates adjacency
   * list, edgestatus support, and reserves edgelabels.
   * @param bucketsize  Adjacency list bucket size.
   */
  void InitializeMultiModal(const uint32_t bucketsize);

  /**
   * Constructs the isotile - 2-D gridded data containing the time
   * to get to each lat,lng tile.
   * @param  mulitmodal  True if the route type is multimodal.
   * @param  max_minutes Maximum time (minutes) for computing isochrones.
   * @param  origin_locations  List of origin locations.
   */
  void ConstructIsoTile(const bool multimodal, const unsigned int max_minutes,
                        std::vector<baldr::PathLocation>& origin_locations);

  /**
   * Expand from the node along the forward search path.
   */
  void ExpandForward(baldr::GraphReader& graphreader,
           const baldr::GraphId& node, const sif::EdgeLabel& pred,
           const uint32_t pred_idx, const bool from_transition);

  /**
   * Expand from the node along the reverse search path.
   */
  void ExpandReverse(baldr::GraphReader& graphreader,
           const baldr::GraphId& node, const sif::BDEdgeLabel& pred,
           const uint32_t pred_idx, const baldr::DirectedEdge* opp_pred_edge,
           const bool from_transition);

  /**
   * Updates the isotile using the edge information from the predecessor edge
   * label. This is the edge being settled (lowest cost found to the edge).
   * @param  pred         Predecessor edge label (edge being settled).
   * @param  graphreader  Graph reader
   * @param  ll           Lat,lon at the end of the edge.
   *
   */
  void UpdateIsoTile(const sif::EdgeLabel& pred,
                     baldr::GraphReader& graphreader,
                     const midgard::PointLL& ll);

  /**
   * Add edge(s) at each origin location to the adjacency list.
   * @param  graphreader       Graph tile reader.
   * @param  origin_locations  Location information for origins.
   * @param  costing           Dynamic costing.
   */
  void SetOriginLocations(baldr::GraphReader& graphreader,
                  std::vector<baldr::PathLocation>& origin_locations,
                  const std::shared_ptr<sif::DynamicCost>& costing);

  /**
    * Add edge(s) at each destination location to the adjacency list.
    * @param  graphreader       Graph tile reader.
    * @param  dest_locations    Location information for destinations.
    * @param  costing           Dynamic costing.
    */
  void SetDestinationLocations(baldr::GraphReader& graphreader,
                       std::vector<baldr::PathLocation>& dest_locations,
                       const std::shared_ptr<sif::DynamicCost>& costing);

};


}
}

#endif  // VALHALLA_THOR_ISOCHRONE_H_
