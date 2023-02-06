#include "sif/customcost.h"
#include "proto_conversions.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

namespace {
BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  return cfg;
}
const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();
} // namespace

class CustomCost : public DynamicCost {
public:
  // mandatory
  CustomCost(const Costing& costing) : DynamicCost(costing, TravelMode::kDrive, kAllAccess) {
  }

  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool,
                       const EdgeLabel&,
                       const graph_tile_ptr&,
                       const baldr::GraphId&,
                       const uint64_t,
                       const uint32_t,
                       uint8_t&) const override {
    return !edge->is_shortcut();
  }

  virtual bool AllowedReverse(const baldr::DirectedEdge*,
                              const EdgeLabel&,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr&,
                              const baldr::GraphId&,
                              const uint64_t,
                              const uint32_t,
                              uint8_t&) const override {
    return !opp_edge->is_shortcut();
  }

  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("NoCost::EdgeCost does not support transit edges");
  }

  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr&,
                        const baldr::TimeInfo&,
                        uint8_t&) const override {
    return {static_cast<float>(edge->length()), static_cast<float>(edge->length())};
  }

  virtual float AStarCostFactor() const override {
    return 1.f;
  }

  // extras
  //   virtual ~CustomCost() {
  //   }

  //   bool Allowed(const baldr::NodeInfo*) const override {
  //     return true;
  //   }

  //   virtual bool IsAccessible(const baldr::DirectedEdge*) const override {
  //     return true;
  //   }

  //   bool IsClosed(const baldr::DirectedEdge*, const graph_tile_ptr&) const override {
  //     return false;
  //   }

  //   virtual Cost TransitionCost(const baldr::DirectedEdge*,
  //                               const baldr::NodeInfo*,
  //                               const EdgeLabel&) const override {
  //     return {};
  //   }

  //   virtual Cost TransitionCostReverse(const uint32_t,
  //                                      const baldr::NodeInfo*,
  //                                      const baldr::DirectedEdge*,
  //                                      const baldr::DirectedEdge*,
  //                                      const bool,
  //                                      const InternalTurn) const override {
  //     return {};
  //   }

  //   bool Allowed(const baldr::DirectedEdge* edge, const graph_tile_ptr&, uint16_t) const override {
  //     return !(edge->is_shortcut() || edge->IsTransitLine());
  //   }
};

void ParseCustomCostOptions(const rapidjson::Document& doc,
                            const std::string& costing_options_key,
                            Costing* c) {
  c->set_type(Costing::custom);
  c->set_name(Costing_Enum_Name(c->type()));
  //   auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig);
}

cost_ptr_t CreateCustomCost(const Costing& costing_options) {
  return std::make_shared<CustomCost>(costing_options);
}

} // namespace sif
} // namespace valhalla