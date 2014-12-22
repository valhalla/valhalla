#ifndef VALHALLA_BALDR_GRAPHCONSTANTS_H_
#define VALHALLA_BALDR_GRAPHCONSTANTS_H_

namespace valhalla {
namespace baldr {

enum class RoadClass : uint8_t {
  kMotorway,
  kTrunk,
  kPrimary,
  kTertiaryUnclassified,
  kResidential,
  kService,
  kTrack,
  kOther
};

enum class Use : uint8_t {
  kNone,
  kCycleway,
  kParkingAisle,
  kDriveway,
  kAlley,
  kEmergencyAccess,
  kDriveThru,
  kSteps,
  kOther
};


}
}

#endif  // VALHALLA_BALDR_GRAPHCONSTANTS_H_
