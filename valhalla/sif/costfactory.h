#ifndef VALHALLA_SIF_COSTFACTORY_H_
#define VALHALLA_SIF_COSTFACTORY_H_

#include <functional>
#include <map>
#include <memory>

#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto_conversions.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/motorcyclecost.h>
#include <valhalla/sif/motorscootercost.h>
#include <valhalla/sif/nocost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/sif/transitcost.h>
#include <valhalla/sif/truckcost.h>

namespace valhalla {
namespace sif {

/**
 * Generic factory class for creating objects based on type name.
 */
class CostFactory {
public:
  using factory_function_t = std::function<cost_ptr_t(const CostingOptions& options)>;

  /**
   * Constructor
   */
  CostFactory() {
    Register(Costing::auto_, CreateAutoCost);
    // auto_data_fix was deprecated
    // auto_shorter was deprecated
    Register(Costing::bicycle, CreateBicycleCost);
    Register(Costing::bus, CreateBusCost);
    Register(Costing::taxi, CreateTaxiCost);
    Register(Costing::motor_scooter, CreateMotorScooterCost);
    Register(Costing::motorcycle, CreateMotorcycleCost);
    Register(Costing::pedestrian, CreatePedestrianCost);
    Register(Costing::truck, CreateTruckCost);
    Register(Costing::transit, CreateTransitCost);
    Register(Costing::none_, CreateNoCost);
    Register(Costing::bikeshare, CreateBikeShareCost);
  }

  /**
   * Register the callback to create this type of cost
   *
   * @param costing    the cost type that the function creates
   * @param function   the function pointer to call to actually create the cost object
   */
  void Register(const Costing costing, factory_function_t function) {
    factory_funcs_.erase(costing);
    factory_funcs_.emplace(costing, function);
  }

  /**
   * Make a cost from its specified type
   * @param options  pbf with costing type and costing options
   */
  cost_ptr_t Create(const Options& options) const {
    // you cant get a costing without a costing type
    if (!options.has_costing())
      throw std::runtime_error("No costing provided to cost factory");

    // create the cost using the creation function
    auto costing_index = static_cast<int>(options.costing());
    if (costing_index < options.costing_options_size()) {
      return Create(options.costing_options(costing_index));
    } // if we didnt have costing options we need to use some default ones
    else {
      throw std::runtime_error("No costing options provided to cost factory");
    }
  }

  /**
   * Make a default cost from its specified type
   * @param costing  which costing to create
   */
  cost_ptr_t Create(Costing costing) const {
    CostingOptions default_options;
    default_options.set_costing(costing);
    return Create(default_options);
  }

  /**
   * Make a cost from its specified type
   * @param costing  the type of cost to create
   * @param options  pbf with request options
   */
  cost_ptr_t Create(const CostingOptions& options) const {
    // you cant get a costing without a costing type
    if (!options.has_costing())
      throw std::runtime_error("No costing provided to cost factory");

    auto itr = factory_funcs_.find(options.costing());
    if (itr == factory_funcs_.end()) {
      auto costing_str = Costing_Enum_Name(options.costing());
      throw std::runtime_error("No costing method found for '" + costing_str + "'");
    }
    // create the cost using the function pointer
    return itr->second(options);
  }

  mode_costing_t CreateModeCosting(const Options& options, TravelMode& mode) {
    mode_costing_t mode_costing;
    // Set travel mode and construct costing
    if (options.costing() == Costing::multimodal || options.costing() == Costing::transit ||
        options.costing() == Costing::bikeshare) {
      // For multi-modal we construct costing for all modes and set the
      // initial mode to pedestrian. (TODO - allow other initial modes)
      mode_costing[0] = Create(options.costing_options(static_cast<int>(Costing::auto_)));
      mode_costing[1] = Create(options.costing_options(static_cast<int>(Costing::pedestrian)));
      mode_costing[2] = Create(options.costing_options(static_cast<int>(Costing::bicycle)));
      mode_costing[3] = Create(options.costing_options(static_cast<int>(Costing::transit)));
      mode = valhalla::sif::TravelMode::kPedestrian;
    } else {
      valhalla::sif::cost_ptr_t cost = Create(options);
      mode = cost->travel_mode();
      mode_costing[static_cast<uint32_t>(mode)] = cost;
    }
    return mode_costing;
  }

private:
  std::map<const Costing, factory_function_t> factory_funcs_;
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_COSTFACTORY_H_
