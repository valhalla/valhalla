#ifndef VALHALLA_SIF_COSTFACTORY_H_
#define VALHALLA_SIF_COSTFACTORY_H_

#include <map>
#include <memory>

#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/motorcyclecost.h>
#include <valhalla/sif/motorscootercost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/sif/transitcost.h>
#include <valhalla/sif/truckcost.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace sif {

/**
 * Generic factory class for creating objects based on type name.
 */
template <class cost_t> class CostFactory {
public:
  typedef std::shared_ptr<cost_t> cost_ptr_t;
  typedef cost_ptr_t (*factory_function_t)(const odin::Costing costing,
                                           const odin::DirectionsOptions& options);

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
  void Register(const odin::Costing costing, factory_function_t function) {
    factory_funcs_.emplace(costing, function);
  }

  // TODO rm
  //  cost_ptr_t Create(const std::string& name, const boost::property_tree::ptree& config) const {
  //    auto itr = factory_funcs_.find(name);
  //    if (itr == factory_funcs_.end()) {
  //      throw std::runtime_error("No costing method found for '" + name + "'");
  //    }
  //    // create the cost using the function pointer
  //    return itr->second(config);
  //  }

  /**
   * Make a cost from its specified type
   * @param cost    the type of cost to create
   * @param options  pbf with request options
   */
  cost_ptr_t Create(const odin::Costing costing, const odin::DirectionsOptions& options) const {
    auto itr = factory_funcs_.find(costing);
    if (itr == factory_funcs_.end()) {
      auto costing_str = odin::Costing_Name(costing);
      if (costing_str.back() == '_') {
        costing_str.pop_back();
      }
      throw std::runtime_error("No costing method found for '" + costing_str + "'");
    }
    // create the cost using the function pointer
    return itr->second(costing, options);
  }

  // TODO rm
  //  cost_ptr_t Create(const std::string& name, const rapidjson::Value& config) const {
  //    if (config.IsNull()) {
  //      return Create(name, boost::property_tree::ptree{});
  //    }
  //    rapidjson::StringBuffer buffer;
  //    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  //    config.Accept(writer);
  //    boost::property_tree::ptree pt;
  //    std::istringstream is(buffer.GetString());
  //    boost::property_tree::read_json(is, pt);
  //    return Create(name, pt);
  //  }

  /**
   * Convenience method to register all of the standard costing models.
   */
  void RegisterStandardCostingModels() {
    Register(odin::Costing::auto_, CreateAutoCost);
    Register(odin::Costing::auto_data_fix, CreateAutoDataFixCost);
    Register(odin::Costing::auto_shorter, CreateAutoShorterCost);
    Register(odin::Costing::bicycle, CreateBicycleCost);
    Register(odin::Costing::bus, CreateBusCost);
    Register(odin::Costing::hov, CreateHOVCost);
    Register(odin::Costing::motor_scooter, CreateMotorScooterCost);
    Register(odin::Costing::motorcycle, CreateMotorcycleCost);
    Register(odin::Costing::pedestrian, CreatePedestrianCost);
    Register(odin::Costing::truck, CreateTruckCost);
    Register(odin::Costing::transit, CreateTransitCost);
  }

private:
  std::map<const odin::Costing, factory_function_t> factory_funcs_;
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_COSTFACTORY_H_
