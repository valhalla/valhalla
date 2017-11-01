#ifndef VALHALLA_SIF_COST_CONSTANTS_H_
#define VALHALLA_SIF_COST_CONSTANTS_H_

namespace valhalla {
namespace sif {

// Travel modes
enum class TravelMode : uint8_t {
   kDrive = 0,
   kPedestrian = 1,
   kBicycle = 2,
   kPublicTransit = 3,
   kMaxTravelMode = 4
};

// Vehicle travel type
enum class VehicleType : uint8_t {
  kCar = 0,
  kMotorcycle = 1,
  kBus = 2,
  kTractorTrailer = 3,
  kMotorScooter = 4
};

// Pedestrian travel type
enum class PedestrianType : uint8_t {
  kFoot = 0,
  kWheelchair = 1,
  kSegway = 2
};

// Bicycle travel type
enum class BicycleType : uint8_t {
  kRoad     = 0,
  kCross    = 1,    // Cyclocross bike - road bike setup with wider tires
  kHybrid   = 2,    // Hybrid or city bike
  kMountain = 3
};

// Transit travel type
// TODO: these are in graphconstants
// decide what to do
//enum class TransitType : uint8_t {
//  kTram = 0,
//  kMetro = 1,
//  kRail = 2,
//  kBus = 3,
//  kFerry = 4,
//  kCableCar = 5,
//  kGondola = 6,
//  kFunicular = 7
//};

/**
 * Simple structure for returning costs. Includes cost and true elapsed time
 * in seconds.
 */
struct Cost {
  float cost;
  float secs;

  /**
   * Default constructor
   */
  Cost()
      : cost(0.0f),
        secs(0.0f) {
  }

  /**
   * Constructor given cost and seconds.
   * @param  c  Cost (units defined by the costing model)
   * @param  s  Time in seconds.
   */
  Cost(const float c, const float s)
       : cost(c),
         secs(s) {
  }

  /**
   * Add 2 costs.
   * @param  other  Cost to add to this cost.
   * @return  Returns the sum of the costs.
   */
  Cost operator + (const Cost& other) const {
    return Cost(cost + other.cost, secs + other.secs);
  }

  /**
   * Subtract cost from another.
   * @param  other  Cost to subtract from this cost.
   * @return  Returns the cost after subtraction.
   */
  Cost operator - (const Cost& other) const {
    return Cost(cost - other.cost, secs - other.secs);
  }

  /**
   * Add to this cost.
   * @param   other  Cost to add to the current cost.
   * @return  Returns address of this cost.
   */
  Cost& operator += (const Cost& other) {
    cost += other.cost;
    secs += other.secs;
    return *this;
  }

  /**
   * Subtract from this cost.
   * @param   other  Cost to subtract from the current cost.
   * @return  Returns address of this cost.
   */
  Cost& operator -= (const Cost& other) {
    cost -= other.cost;
    secs -= other.secs;
    return *this;
  }

  /**
   * Scale this cost by a factor (for partial costs along edges).
   * @param  f  Scale / multiplication factor.
   * @return  Returns address of this cost.
   */
  Cost& operator *= (const float f) {
    cost *= f;
    secs *= f;
    return *this;
  }

  /**
   * Scale the cost by a factor (for partial costs along edges).
   * @param   f  Scale / multiplication factor.
   * @return  Returns a new cost that is the product of this cost and
   *          the scaling factor.
   */
  Cost operator * (const float f) const {
    return Cost(cost * f, secs * f);
  }

  /**
   * Less than operator - compares cost.
   * @param  other  Cost to compare against.
   * @return  Returns true if this cost is less than the other cost.
   */
  bool operator < (const Cost& other) const {
    return cost < other.cost;
  }

  /**
   * Greater than operator - compares cost.
   * @param  other  Cost to compare against.
   * @return  Returns true if this cost is greater than the other cost.
   */
  bool operator > (const Cost& other) const {
    return cost > other.cost;
  }
};

}
}

#endif  // VALHALLA_SIF_COST_CONSTANTS_H_
