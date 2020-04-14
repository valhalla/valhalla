#ifndef VALHALLA_SIF_COST_CONSTRAINTS_H_
#define VALHALLA_SIF_COST_CONSTRAINTS_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/baldr/graphconstants.h>

#include <memory>
#include <third_party/rapidjson/include/rapidjson/document.h>
#include <unordered_map>

namespace valhalla {
namespace sif {

struct CostConstraints
{
    std::function<double(uint32_t, const baldr::DirectedEdge* , const baldr::GraphTile* , const uint32_t, const float, const bool)> ComputeConstraintForEdgeF = 0;    
    std::function<double(uint32_t, const baldr::GraphId id, const uint32_t start, const float end, const bool)> ComputeConstraintForNodeF = 0;

 
    static std::function<double(uint32_t, const baldr::DirectedEdge* , const baldr::GraphTile* , const uint32_t, const float, const bool)> StaticComputeConstraintForEdgeF;    
    static std::function<double(uint32_t, const baldr::GraphId& id, const uint32_t start, const float end, const bool)> StaticComputeConstraintForNodeF;
    

    /**
     * Parses constraints from json
     */    
    static void ParseConstraints(const rapidjson::Value& value, CostingOptions* pbf_costing_options)
    {
        //auto speed_types = rapidjson::get_child_optional(value, "/speed_types");
        auto constraint_costing_json = rapidjson::get_optional<rapidjson::Value::ConstArray>(value, "/constraints");
        if (constraint_costing_json) 
        {
            for (const auto& constraint : *constraint_costing_json) 
            {
                auto c = pbf_costing_options->add_costing_constraints();                
                auto max_inclusive = rapidjson::get_optional<bool>(constraint, "/max_inclusive");
                auto min_inclusive = rapidjson::get_optional<bool>(constraint, "/min_inclusive");
                if(max_inclusive)
                    c->set_max_inclusive(max_inclusive.get());
                if(min_inclusive)
                    c->set_min_inclusive(min_inclusive.get());
            
                auto maxval = rapidjson::get_optional<double>(constraint, "/max_value");
                auto minval = rapidjson::get_optional<double>(constraint, "/min_value");
                if(maxval)
                    c->set_max_value(maxval.get());
                if(minval)
                    c->set_min_value(minval.get());
            }
        }       
        
    }
    /**
     * Loads Constraints from costing options into structure...
     * @param pbf_costing_options Costing pptions to be loaded...
     */ 
    void LoadConstraints(const CostingOptions& pbf_costing_options)
    {
        for(auto option= pbf_costing_options.costing_constraints().begin();option != pbf_costing_options.costing_constraints().end(); ++option)
            constraints_.push_back(CostingConstraint(*option));        
    }
    /**
     * Compute all secondary constraints for the specified directed edge. 
     * @param   cost        Cost where constraint calculations will be stored.
     * @param   edge        Pointer to a directed edge.
     * @param   tile        Pointer to the tile which contains the directed edge for speed lookup
     * @param   start       Time arriving at edge (secs)
     * @param   end         Time leaving edge (secs)
     * @param   is_forward  Boolean indicating whether this is a forward or reverse search
     */ 
    virtual void ComputeConstraintsForEdge(Cost& cost, const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile, const uint32_t start,
                        const float end, const bool is_forward) const
    {   
        auto ComputeConstraintForEdgeF = GetComputeConstraintForEdgeF();
        if(constraints_.size() > 0)        
            for(uint32_t i=0;i<constraints_.size();++i)            
                cost.data.push_back(ComputeConstraintForEdgeF(i,edge, tile, start, end, is_forward));                        
    }

    /**
     * Compute all secondary constraints for the specified node. 
     * @param   cost        Cost where constraint calculations will be stored
     * @param   id          Node Id
     * @param   start       Time arriving at node (sec)
     * @param   end         Time leaving node (float)
     * @param   is_forward  Boolean indicating whether this is a forward or reverse search
     */
    virtual void ComputeConstraintsForNode(Cost& cost, const baldr::GraphId& id, const uint32_t start, const float end, 
                              const bool is_forward) const
    {   
        auto ComputeConstraintForNodeF = GetComputeConstraintForNodeF();
        if(constraints_.size() > 0)        
            for(uint32_t i=0;i<constraints_.size();++i)            
                cost.data.push_back(ComputeConstraintForNodeF(i, id, start, end, is_forward));                
    }
    virtual inline bool ConstraintsSatisfied(const Cost& cost) const   
    { 
        if(cost.data.size() == constraints_.size())
        {
            for(int i = 0; i < constraints_.size(); ++i)
            {
                auto d = cost.data[i];
                auto c = constraints_[i];
                bool lower_satisfied = c.min_inclusive() ? d >= c.min_value() : d > c.min_value();
                bool upper_satisfied = c.max_inclusive() ? d <= c.max_value() : d < c.max_value();        
                if(!lower_satisfied || !upper_satisfied)
                    return false;
            }
        }  
        return true; 
   }
   inline bool HasConstraints() const
   {
       return constraints_.size() > 0;
   }
    protected:
     std::vector<CostingConstraint> constraints_;
     std::function<double(uint32_t, const baldr::DirectedEdge* , const baldr::GraphTile* , const uint32_t, const float, const bool)> GetComputeConstraintForEdgeF() const
     {
         if(ComputeConstraintForEdgeF != 0)
            return ComputeConstraintForEdgeF;
        return StaticComputeConstraintForEdgeF;
     }
      std::function<double(uint32_t, const baldr::GraphId& id, const uint32_t start, const float end, const bool)> GetComputeConstraintForNodeF() const
      {
          if(ComputeConstraintForNodeF != 0)
            return ComputeConstraintForNodeF;
          return StaticComputeConstraintForNodeF;
      }
     static double EmptyReturnEdge(uint32_t idx, const baldr::DirectedEdge* edge, const baldr::GraphTile* tile, const uint32_t start, const float end, const bool is_forward) ;
     static double EmptyReturnNode(uint32_t idx, const baldr::GraphId& id, const uint32_t start, const float end, const bool is_forward);
};
} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_COST_CONSTRAINTS_H_