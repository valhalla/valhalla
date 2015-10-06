// -*- mode: c++ -*-

#include <valhalla/sif/dynamiccost.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::sif;
using namespace valhalla::baldr;


class PedestrianCost : public DynamicCost {
 public:
  PedestrianCost(const boost::property_tree::ptree& pt)
      : DynamicCost(pt, TravelMode::kPedestrian) {
  }

  bool Allowed(const DirectedEdge* edge,
               const EdgeLabel& pred) const {
    return true;
  }

  bool AllowedReverse(const DirectedEdge* edge,
                      const DirectedEdge* opp_edge,
                      const DirectedEdge* opp_pred_edge) const
  {
    return true;
  }

  bool Allowed(const NodeInfo* node) const
  {
    return true;
  }

  Cost EdgeCost(const DirectedEdge* edge,
                const uint32_t density) const
  {
    float length = edge->length();
    return { length, length };
  }

  // Disable astar
  float AStarCostFactor() const {
    return 0.f;
  }

  virtual const EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const DirectedEdge* edge){
      return false;
    };
  }
};


cost_ptr_t CreatePedestrianCost(const boost::property_tree::ptree& config) {
  return std::make_shared<PedestrianCost>(config);
}
