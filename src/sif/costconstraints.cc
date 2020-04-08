#include <sif/costconstraints.h>
namespace valhalla {
namespace sif {

    double CostConstraints::EmptyReturnEdge(uint32_t idx, const baldr::DirectedEdge* edge, const baldr::GraphTile* tile, const uint32_t start, const float end, const bool is_forward) {return 0;}
    double CostConstraints::EmptyReturnNode(uint32_t idx, const baldr::GraphId& id, const uint32_t start, const float end, const bool is_forward) {return 0;}
    std::function<double(uint32_t, const baldr::DirectedEdge* , const baldr::GraphTile* , const uint32_t, const float, const bool)> CostConstraints::StaticComputeConstraintForEdgeF = CostConstraints::EmptyReturnEdge;    
    std::function<double(uint32_t, const baldr::GraphId& id, const uint32_t start, const float end, const bool)> CostConstraints::StaticComputeConstraintForNodeF = CostConstraints::EmptyReturnNode;

}
}