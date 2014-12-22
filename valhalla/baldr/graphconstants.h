#ifndef VALHALLA_BALDR_GRAPHCONSTANTS_H_
#define VALHALLA_BALDR_GRAPHCONSTANTS_H_

namespace valhalla {
namespace baldr {

enum class RoadClass : unsigned int {
  kMotorway,
  kTrunk,
  kPrimary,
  kTertiaryUnclassified,
  kResidential,
  kService,
  kTrack,
  kOther
};

enum class Use : unsigned int {
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
