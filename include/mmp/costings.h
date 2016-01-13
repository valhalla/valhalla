// -*- mode: c++ -*-

#ifndef MM_COSTINGS_H_
#define MM_COSTINGS_H_

#include <valhalla/sif/dynamiccost.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla;


namespace mmp
{

constexpr sif::TravelMode kUniversalTravelMode = static_cast<sif::TravelMode>(4);

class UniversalCost : public sif::DynamicCost {
 public:
  UniversalCost(const boost::property_tree::ptree& pt)
      : DynamicCost(pt, kUniversalTravelMode) {}

  bool Allowed(const baldr::DirectedEdge* edge,
               const sif::EdgeLabel& pred,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid) const
  { return true; }

  bool AllowedReverse(const baldr::DirectedEdge* edge,
                      const sif::EdgeLabel& pred,
                      const baldr::DirectedEdge* opp_edge,
                      const baldr::DirectedEdge* opp_pred_edge,
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
      return false;
    };
  }
};


sif::cost_ptr_t CreateUniversalCost(const boost::property_tree::ptree& config)
{ return std::make_shared<UniversalCost>(config); }

}


#endif // MM_COSTINGS_H_
