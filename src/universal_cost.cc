#include <valhalla/midgard/logging.h>
#include <valhalla/sif/dynamiccost.h>

using namespace valhalla;

#include "mmp/universal_cost.h"


namespace mmp {


class UniversalCost : public sif::DynamicCost {
 public:
  UniversalCost(const boost::property_tree::ptree& pt)
      : DynamicCost(pt, kUniversalTravelMode) {}

  bool Allowed(const baldr::DirectedEdge* edge,
               const sif::EdgeLabel& pred,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid) const
  {
    // Disable transit lines
    if (edge->IsTransitLine()) {
      return false;
    }
    return true;
  }

  bool AllowedReverse(const baldr::DirectedEdge* edge,
                      const sif::EdgeLabel& pred,
                      const baldr::DirectedEdge* opp_edge,
                      const baldr::GraphTile*& tile,
                      const baldr::GraphId& edgeid) const
  { return true; }

  bool Allowed(const baldr::NodeInfo* node) const
  { return true; }

  sif::Cost EdgeCost(const baldr::DirectedEdge* edge,
                     const uint32_t density) const
  {
    float length = edge->length();
    return { length, length };
  }

  // Disable astar
  float AStarCostFactor() const
  { return 0.f; }

  virtual const sif::EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge){
      // Disable transit lines
      return edge->IsTransitLine();
    };
  }
};


sif::cost_ptr_t CreateUniversalCost(const boost::property_tree::ptree& config)
{ return std::make_shared<UniversalCost>(config); }


}
