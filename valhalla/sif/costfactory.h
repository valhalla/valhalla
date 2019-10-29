#ifndef VALHALLA_SIF_COSTFACTORY_H_
#define VALHALLA_SIF_COSTFACTORY_H_

#include <map>
#include <memory>

#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/motorcyclecost.h>
#include <valhalla/sif/motorscootercost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/sif/transitcost.h>
#include <valhalla/sif/truckcost.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace sif {

/**
 * Generic factory class for creating objects based on type name.
 */
template <class cost_t> class CostFactory {
public:
  typedef std::shared_ptr<cost_t> cost_ptr_t;
  typedef cost_ptr_t (*factory_function_t)(const Costing costing, const Options& options);

  /**
   * Constructor
   */
  CostFactory() {
  }

  /**
   * Register the callback to create this type of cost
   *
   * @param costing    the cost type that the function creates
   * @param function   the function pointer to call to actually create the cost object
   */
  void Register(const Costing costing, factory_function_t function) {
    factory_funcs_.emplace(costing, function);
  }

  /**
   * Make a cost from its specified type
   * @param costing  the type of cost to create
   * @param options  pbf with request options
   */
  cost_ptr_t Create(const Costing costing, const Options& options) const {
    auto itr = factory_funcs_.find(costing);
    if (itr == factory_funcs_.end()) {
      auto costing_str = Costing_Enum_Name(costing);
      throw std::runtime_error("No costing method found for '" + costing_str + "'");
    }
    // create the cost using the function pointer
    return itr->second(costing, options);
  }

  /**
   * Convenience method to register all of the standard costing models.
   */
  void RegisterStandardCostingModels() {
    Register(Costing::auto_, CreateAutoCost);
    Register(Costing::auto_data_fix, CreateAutoDataFixCost);
    Register(Costing::auto_shorter, CreateAutoShorterCost);
    Register(Costing::bicycle, CreateBicycleCost);
    Register(Costing::bus, CreateBusCost);
    Register(Costing::hov, CreateHOVCost);
    Register(Costing::taxi, CreateTaxiCost);
    Register(Costing::motor_scooter, CreateMotorScooterCost);
    Register(Costing::motorcycle, CreateMotorcycleCost);
    Register(Costing::pedestrian, CreatePedestrianCost);
    Register(Costing::truck, CreateTruckCost);
    Register(Costing::transit, CreateTransitCost);
  }

private:
  std::map<const Costing, factory_function_t> factory_funcs_;
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_COSTFACTORY_H_
