#pragma once
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/proto/incidents.pb.h>
#include <valhalla/proto/matrix.pb.h>
#include <valhalla/sif/costconstants.h>

namespace valhalla {
// Associate RoadClass values to TripLeg proto
constexpr valhalla::RoadClass kTripLegRoadClass[] = {valhalla::RoadClass::kMotorway,
                                                     valhalla::RoadClass::kTrunk,
                                                     valhalla::RoadClass::kPrimary,
                                                     valhalla::RoadClass::kSecondary,
                                                     valhalla::RoadClass::kTertiary,
                                                     valhalla::RoadClass::kUnclassified,
                                                     valhalla::RoadClass::kResidential,
                                                     valhalla::RoadClass::kServiceOther};
inline valhalla::RoadClass GetRoadClass(const baldr::RoadClass road_class) {
  return kTripLegRoadClass[static_cast<int>(road_class)];
}

// Associate Surface values to TripLeg proto
constexpr TripLeg_Surface kTripLegSurface[] =
    {TripLeg_Surface_kPavedSmooth, TripLeg_Surface_kPaved,     TripLeg_Surface_kPavedRough,
     TripLeg_Surface_kCompacted,   TripLeg_Surface_kDirt,      TripLeg_Surface_kGravel,
     TripLeg_Surface_kPath,        TripLeg_Surface_kImpassable};
inline TripLeg_Surface GetTripLegSurface(const baldr::Surface surface) {
  return kTripLegSurface[static_cast<int>(surface)];
}

// Associate vehicle types to TripLeg proto
// TODO - why doesn't these use an enum input?
constexpr VehicleType kTripLegVehicleType[] = {
    VehicleType::kCar,   VehicleType::kMotorcycle,   VehicleType::kAutoBus,
    VehicleType::kTruck, VehicleType::kMotorScooter,
};
inline VehicleType GetTripLegVehicleType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(sif::VehicleType::kMotorScooter)) ? kTripLegVehicleType[type]
                                                                         : kTripLegVehicleType[0];
}

// Associate pedestrian types to TripLeg proto
constexpr PedestrianType kTripLegPedestrianType[] = {PedestrianType::kFoot,
                                                     PedestrianType::kWheelchair,
                                                     PedestrianType::kBlind};
inline PedestrianType GetTripLegPedestrianType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(sif::PedestrianType::kBlind)) ? kTripLegPedestrianType[type]
                                                                     : kTripLegPedestrianType[0];
}

// Associate bicycle types to TripLeg proto
constexpr BicycleType kTripLegBicycleType[] = {
    BicycleType::kRoad,
    BicycleType::kCross,
    BicycleType::kHybrid,
    BicycleType::kMountain,
};
inline BicycleType GetTripLegBicycleType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(sif::BicycleType::kMountain)) ? kTripLegBicycleType[type]
                                                                     : kTripLegBicycleType[0];
}

// Associate transit types to TripLeg proto
constexpr TransitType kTripLegTransitType[] = {
    TransitType::kTram,  TransitType::kMetro,    TransitType::kRail,    TransitType::kBus,
    TransitType::kFerry, TransitType::kCableCar, TransitType::kGondola, TransitType::kFunicular,
};
inline TransitType GetTripLegTransitType(const baldr::TransitType transit_type) {
  return kTripLegTransitType[static_cast<uint32_t>(transit_type)];
}

// Associate traversability values to TripLeg proto
constexpr TripLeg_Traversability kTripLegTraversability[] = {TripLeg_Traversability_kNone,
                                                             TripLeg_Traversability_kForward,
                                                             TripLeg_Traversability_kBackward,
                                                             TripLeg_Traversability_kBoth};
inline TripLeg_Traversability GetTripLegTraversability(const baldr::Traversability traversability) {
  return kTripLegTraversability[static_cast<uint32_t>(traversability)];
}

// Associate side of street to TripLeg proto
constexpr valhalla::Location::SideOfStreet kTripLegSideOfStreet[] = {valhalla::Location::kNone,
                                                                     valhalla::Location::kLeft,
                                                                     valhalla::Location::kRight};
inline valhalla::Location::SideOfStreet
GetTripLegSideOfStreet(const valhalla::Location::SideOfStreet sos) {
  return kTripLegSideOfStreet[static_cast<uint32_t>(sos)];
}

inline TripLeg_Node_Type GetTripLegNodeType(const baldr::NodeType node_type) {
  switch (node_type) {
    case baldr::NodeType::kStreetIntersection:
      return TripLeg_Node_Type_kStreetIntersection;
    case baldr::NodeType::kGate:
      return TripLeg_Node_Type_kGate;
    case baldr::NodeType::kBollard:
      return TripLeg_Node_Type_kBollard;
    case baldr::NodeType::kTollBooth:
      return TripLeg_Node_Type_kTollBooth;
    case baldr::NodeType::kTransitEgress:
      return TripLeg_Node_Type_kTransitEgress;
    case baldr::NodeType::kTransitStation:
      return TripLeg_Node_Type_kTransitStation;
    case baldr::NodeType::kMultiUseTransitPlatform:
      return TripLeg_Node_Type_kTransitPlatform;
    case baldr::NodeType::kBikeShare:
      return TripLeg_Node_Type_kBikeShare;
    case baldr::NodeType::kParking:
      return TripLeg_Node_Type_kParking;
    case baldr::NodeType::kMotorWayJunction:
      return TripLeg_Node_Type_kMotorwayJunction;
    case baldr::NodeType::kBorderControl:
      return TripLeg_Node_Type_kBorderControl;
    case baldr::NodeType::kTollGantry:
      return TripLeg_Node_Type_kTollGantry;
    case baldr::NodeType::kSumpBuster:
      return TripLeg_Node_Type_kSumpBuster;
    case baldr::NodeType::kBuildingEntrance:
      return TripLeg_Node_Type_kBuildingEntrance;
    case baldr::NodeType::kElevator:
      return TripLeg_Node_Type_kElevator;
  }
  auto num = static_cast<uint8_t>(node_type);
  throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) +
                           " Unhandled NodeType: " + std::to_string(num));
}

inline Pronunciation_Alphabet
GetTripPronunciationAlphabet(const valhalla::baldr::PronunciationAlphabet pronunciation_alphabet) {
  switch (pronunciation_alphabet) {
    case baldr::PronunciationAlphabet::kNone:
      return Pronunciation_Alphabet_kNone;
    case baldr::PronunciationAlphabet::kIpa:
      return Pronunciation_Alphabet_kIpa;
    case baldr::PronunciationAlphabet::kKatakana:
      return Pronunciation_Alphabet_kKatakana;
    case baldr::PronunciationAlphabet::kJeita:
      return Pronunciation_Alphabet_kJeita;
    case baldr::PronunciationAlphabet::kNtSampa:
      return Pronunciation_Alphabet_kNtSampa;
  }
  return Pronunciation_Alphabet_kNone;
}

inline LanguageTag GetTripLanguageTag(valhalla::baldr::Language l) {
  static const std::unordered_map<valhalla::baldr::Language, LanguageTag> language_tag_map =
      {{valhalla::baldr::Language::kAb, LanguageTag::kAb},
       {valhalla::baldr::Language::kAm, LanguageTag::kAm},
       {valhalla::baldr::Language::kAr, LanguageTag::kAr},
       {valhalla::baldr::Language::kAz, LanguageTag::kAz},
       {valhalla::baldr::Language::kBe, LanguageTag::kBe},
       {valhalla::baldr::Language::kBg, LanguageTag::kBg},
       {valhalla::baldr::Language::kBn, LanguageTag::kBn},
       {valhalla::baldr::Language::kBs, LanguageTag::kBs},
       {valhalla::baldr::Language::kCa, LanguageTag::kCa},
       {valhalla::baldr::Language::kCkb, LanguageTag::kCkb},
       {valhalla::baldr::Language::kCs, LanguageTag::kCs},
       {valhalla::baldr::Language::kDa, LanguageTag::kDa},
       {valhalla::baldr::Language::kDe, LanguageTag::kDe},
       {valhalla::baldr::Language::kDv, LanguageTag::kDv},
       {valhalla::baldr::Language::kDz, LanguageTag::kDz},
       {valhalla::baldr::Language::kEl, LanguageTag::kEl},
       {valhalla::baldr::Language::kEn, LanguageTag::kEn},
       {valhalla::baldr::Language::kEs, LanguageTag::kEs},
       {valhalla::baldr::Language::kEt, LanguageTag::kEt},
       {valhalla::baldr::Language::kFa, LanguageTag::kFa},
       {valhalla::baldr::Language::kFi, LanguageTag::kFi},
       {valhalla::baldr::Language::kFr, LanguageTag::kFr},
       {valhalla::baldr::Language::kFy, LanguageTag::kFy},
       {valhalla::baldr::Language::kGl, LanguageTag::kGl},
       {valhalla::baldr::Language::kHe, LanguageTag::kHe},
       {valhalla::baldr::Language::kHr, LanguageTag::kHr},
       {valhalla::baldr::Language::kHu, LanguageTag::kHu},
       {valhalla::baldr::Language::kHy, LanguageTag::kHy},
       {valhalla::baldr::Language::kId, LanguageTag::kId},
       {valhalla::baldr::Language::kIs, LanguageTag::kIs},
       {valhalla::baldr::Language::kIt, LanguageTag::kIt},
       {valhalla::baldr::Language::kJa, LanguageTag::kJa},
       {valhalla::baldr::Language::kKa, LanguageTag::kKa},
       {valhalla::baldr::Language::kKl, LanguageTag::kKl},
       {valhalla::baldr::Language::kKm, LanguageTag::kKm},
       {valhalla::baldr::Language::kKo, LanguageTag::kKo},
       {valhalla::baldr::Language::kLo, LanguageTag::kLo},
       {valhalla::baldr::Language::kLt, LanguageTag::kLt},
       {valhalla::baldr::Language::kLv, LanguageTag::kLv},
       {valhalla::baldr::Language::kMg, LanguageTag::kMg},
       {valhalla::baldr::Language::kMk, LanguageTag::kMk},
       {valhalla::baldr::Language::kMn, LanguageTag::kMn},
       {valhalla::baldr::Language::kMo, LanguageTag::kMo},
       {valhalla::baldr::Language::kMt, LanguageTag::kMt},
       {valhalla::baldr::Language::kMy, LanguageTag::kMy},
       {valhalla::baldr::Language::kNe, LanguageTag::kNe},
       {valhalla::baldr::Language::kNl, LanguageTag::kNl},
       {valhalla::baldr::Language::kNo, LanguageTag::kNo},
       {valhalla::baldr::Language::kOc, LanguageTag::kOc},
       {valhalla::baldr::Language::kPap, LanguageTag::kPap},
       {valhalla::baldr::Language::kPl, LanguageTag::kPl},
       {valhalla::baldr::Language::kPs, LanguageTag::kPs},
       {valhalla::baldr::Language::kPt, LanguageTag::kPt},
       {valhalla::baldr::Language::kRm, LanguageTag::kRm},
       {valhalla::baldr::Language::kRo, LanguageTag::kRo},
       {valhalla::baldr::Language::kRu, LanguageTag::kRu},
       {valhalla::baldr::Language::kSk, LanguageTag::kSk},
       {valhalla::baldr::Language::kSl, LanguageTag::kSl},
       {valhalla::baldr::Language::kSq, LanguageTag::kSq},
       {valhalla::baldr::Language::kSr, LanguageTag::kSr},
       {valhalla::baldr::Language::kSrLatn, LanguageTag::kSrLatn},
       {valhalla::baldr::Language::kSv, LanguageTag::kSv},
       {valhalla::baldr::Language::kTg, LanguageTag::kTg},
       {valhalla::baldr::Language::kTh, LanguageTag::kTh},
       {valhalla::baldr::Language::kTk, LanguageTag::kTk},
       {valhalla::baldr::Language::kTr, LanguageTag::kTr},
       {valhalla::baldr::Language::kUk, LanguageTag::kUk},
       {valhalla::baldr::Language::kUr, LanguageTag::kUr},
       {valhalla::baldr::Language::kUz, LanguageTag::kUz},
       {valhalla::baldr::Language::kVi, LanguageTag::kVi},
       {valhalla::baldr::Language::kZh, LanguageTag::kZh},
       {valhalla::baldr::Language::kCy, LanguageTag::kCy},
       {valhalla::baldr::Language::kNone, LanguageTag::kUnspecified}};

  auto i = language_tag_map.find(l);
  if (i == language_tag_map.cend()) {
    return LanguageTag::kUnspecified;
  }
  return i->second;
}

inline LanguageTag GetTripLanguageTag(const std::string& str_language) {
  static const std::unordered_map<std::string, LanguageTag> str_language_tag_map =
      {{"ab", LanguageTag::kAb},          {"am", LanguageTag::kAm},   {"ar", LanguageTag::kAr},
       {"az", LanguageTag::kAz},          {"be", LanguageTag::kBe},   {"bg", LanguageTag::kBg},
       {"bn", LanguageTag::kBn},          {"bs", LanguageTag::kBs},   {"ca", LanguageTag::kCa},
       {"ckb", LanguageTag::kCkb},        {"cs", LanguageTag::kCs},   {"da", LanguageTag::kDa},
       {"de", LanguageTag::kDe},          {"dv", LanguageTag::kDv},   {"dz", LanguageTag::kDz},
       {"el", LanguageTag::kEl},          {"en", LanguageTag::kEn},   {"es", LanguageTag::kEs},
       {"et", LanguageTag::kEt},          {"fa", LanguageTag::kFa},   {"fi", LanguageTag::kFi},
       {"fr", LanguageTag::kFr},          {"fy", LanguageTag::kFy},   {"gl", LanguageTag::kGl},
       {"he", LanguageTag::kHe},          {"hr", LanguageTag::kHr},   {"hu", LanguageTag::kHu},
       {"hy", LanguageTag::kHy},          {"id", LanguageTag::kId},   {"is", LanguageTag::kIs},
       {"it", LanguageTag::kIt},          {"ja", LanguageTag::kJa},   {"ka", LanguageTag::kKa},
       {"kl", LanguageTag::kKl},          {"km", LanguageTag::kKm},   {"ko", LanguageTag::kKo},
       {"lo", LanguageTag::kLo},          {"lt", LanguageTag::kLt},   {"lv", LanguageTag::kLv},
       {"mg", LanguageTag::kMg},          {"mk", LanguageTag::kMk},   {"mn", LanguageTag::kMn},
       {"mo", LanguageTag::kMo},          {"mt", LanguageTag::kMt},   {"my", LanguageTag::kMy},
       {"ne", LanguageTag::kNe},          {"nl", LanguageTag::kNl},   {"no", LanguageTag::kNo},
       {"oc", LanguageTag::kOc},          {"pap", LanguageTag::kPap}, {"pl", LanguageTag::kPl},
       {"ps", LanguageTag::kPs},          {"pt", LanguageTag::kPt},   {"rm", LanguageTag::kRm},
       {"ro", LanguageTag::kRo},          {"ru", LanguageTag::kRu},   {"sk", LanguageTag::kSk},
       {"sl", LanguageTag::kSl},          {"sq", LanguageTag::kSq},   {"sr", LanguageTag::kSr},
       {"sr-latn", LanguageTag::kSrLatn}, {"sv", LanguageTag::kSv},   {"tg", LanguageTag::kTg},
       {"th", LanguageTag::kTh},          {"tk", LanguageTag::kTk},   {"tr", LanguageTag::kTr},
       {"uk", LanguageTag::kUk},          {"ur", LanguageTag::kUr},   {"uz", LanguageTag::kUz},
       {"vi", LanguageTag::kVi},          {"zh", LanguageTag::kZh},   {"cy", LanguageTag::kCy}};

  auto i = str_language_tag_map.find(str_language);
  if (i == str_language_tag_map.cend()) {
    return LanguageTag::kUnspecified;
  }
  return i->second;
}

inline std::string to_string(LanguageTag lang_tag) {
  switch (lang_tag) {
    case LanguageTag::kAb:
      return "ab";
    case LanguageTag::kAm:
      return "am";
    case LanguageTag::kAr:
      return "ar";
    case LanguageTag::kAz:
      return "az";
    case LanguageTag::kBe:
      return "be";
    case LanguageTag::kBg:
      return "bg";
    case LanguageTag::kBn:
      return "bn";
    case LanguageTag::kBs:
      return "bs";
    case LanguageTag::kCa:
      return "ca";
    case LanguageTag::kCkb:
      return "ckb";
    case LanguageTag::kCs:
      return "cs";
    case LanguageTag::kDa:
      return "da";
    case LanguageTag::kDe:
      return "de";
    case LanguageTag::kDv:
      return "dv";
    case LanguageTag::kDz:
      return "dz";
    case LanguageTag::kEl:
      return "el";
    case LanguageTag::kEn:
      return "en";
    case LanguageTag::kEs:
      return "es";
    case LanguageTag::kEt:
      return "et";
    case LanguageTag::kFa:
      return "fa";
    case LanguageTag::kFi:
      return "fi";
    case LanguageTag::kFr:
      return "fr";
    case LanguageTag::kFy:
      return "fy";
    case LanguageTag::kGl:
      return "gl";
    case LanguageTag::kHe:
      return "he";
    case LanguageTag::kHr:
      return "hr";
    case LanguageTag::kHu:
      return "hu";
    case LanguageTag::kHy:
      return "hy";
    case LanguageTag::kId:
      return "id";
    case LanguageTag::kIs:
      return "is";
    case LanguageTag::kIt:
      return "it";
    case LanguageTag::kJa:
      return "ja";
    case LanguageTag::kKa:
      return "ka";
    case LanguageTag::kKl:
      return "kl";
    case LanguageTag::kKm:
      return "km";
    case LanguageTag::kKo:
      return "ko";
    case LanguageTag::kLo:
      return "lo";
    case LanguageTag::kLt:
      return "lt";
    case LanguageTag::kLv:
      return "lv";
    case LanguageTag::kMg:
      return "mg";
    case LanguageTag::kMk:
      return "mk";
    case LanguageTag::kMn:
      return "mn";
    case LanguageTag::kMo:
      return "mo";
    case LanguageTag::kMt:
      return "mt";
    case LanguageTag::kMy:
      return "my";
    case LanguageTag::kNe:
      return "ne";
    case LanguageTag::kNl:
      return "nl";
    case LanguageTag::kNo:
      return "no";
    case LanguageTag::kOc:
      return "oc";
    case LanguageTag::kPap:
      return "pap";
    case LanguageTag::kPl:
      return "pl";
    case LanguageTag::kPs:
      return "ps";
    case LanguageTag::kPt:
      return "pt";
    case LanguageTag::kRm:
      return "rm";
    case LanguageTag::kRo:
      return "ro";
    case LanguageTag::kRu:
      return "ru";
    case LanguageTag::kSk:
      return "sk";
    case LanguageTag::kSl:
      return "sl";
    case LanguageTag::kSq:
      return "sq";
    case LanguageTag::kSr:
      return "sr";
    case LanguageTag::kSrLatn:
      return "sr-Latn";
    case LanguageTag::kSv:
      return "sv";
    case LanguageTag::kTg:
      return "tg";
    case LanguageTag::kTh:
      return "th";
    case LanguageTag::kTk:
      return "tk";
    case LanguageTag::kTr:
      return "tr";
    case LanguageTag::kUk:
      return "uk";
    case LanguageTag::kUr:
      return "ur";
    case LanguageTag::kUz:
      return "uz";
    case LanguageTag::kVi:
      return "vi";
    case LanguageTag::kZh:
      return "zh";
    case LanguageTag::kCy:
      return "cy";
    case LanguageTag::kUnspecified:
      return "unspecified";
    default:
      // should never come here
      return "unknown";
  }
}

// Associate cycle lane values to TripLeg proto
constexpr TripLeg_CycleLane kTripLegCycleLane[] = {TripLeg_CycleLane_kNoCycleLane,
                                                   TripLeg_CycleLane_kShared,
                                                   TripLeg_CycleLane_kDedicated,
                                                   TripLeg_CycleLane_kSeparated};
inline TripLeg_CycleLane GetTripLegCycleLane(const baldr::CycleLane cyclelane) {
  return kTripLegCycleLane[static_cast<uint32_t>(cyclelane)];
}

// Associate Sac scale values to TripLeg proto
constexpr TripLeg_SacScale kTripLegSacScale[] = {TripLeg_SacScale_kNoSacScale,
                                                 TripLeg_SacScale_kHiking,
                                                 TripLeg_SacScale_kMountainHiking,
                                                 TripLeg_SacScale_kDemandingMountainHiking,
                                                 TripLeg_SacScale_kAlpineHiking,
                                                 TripLeg_SacScale_kDemandingAlpineHiking,
                                                 TripLeg_SacScale_kDifficultAlpineHiking};
inline TripLeg_SacScale GetTripLegSacScale(const baldr::SacScale sac) {
  return kTripLegSacScale[static_cast<uint32_t>(sac)];
}

// Associate Use to TripLeg proto
inline TripLeg_Use GetTripLegUse(const baldr::Use use) {
  switch (use) {
    case baldr::Use::kRoad:
      return TripLeg_Use_kRoadUse;
    case baldr::Use::kRamp:
      return TripLeg_Use_kRampUse;
    case baldr::Use::kTurnChannel:
      return TripLeg_Use_kTurnChannelUse;
    case baldr::Use::kTrack:
      return TripLeg_Use_kTrackUse;
    case baldr::Use::kDriveway:
      return TripLeg_Use_kDrivewayUse;
    case baldr::Use::kAlley:
      return TripLeg_Use_kAlleyUse;
    case baldr::Use::kParkingAisle:
      return TripLeg_Use_kParkingAisleUse;
    case baldr::Use::kEmergencyAccess:
      return TripLeg_Use_kEmergencyAccessUse;
    case baldr::Use::kDriveThru:
      return TripLeg_Use_kDriveThruUse;
    case baldr::Use::kCuldesac:
      return TripLeg_Use_kCuldesacUse;
    case baldr::Use::kLivingStreet:
      return TripLeg_Use_kLivingStreetUse;
    case baldr::Use::kServiceRoad:
      return TripLeg_Use_kServiceRoadUse;
    case baldr::Use::kCycleway:
      return TripLeg_Use_kCyclewayUse;
    case baldr::Use::kMountainBike:
      return TripLeg_Use_kMountainBikeUse;
    case baldr::Use::kSidewalk:
      // return TripLeg_Use_kSidewalkUse;
      return TripLeg_Use_kFootwayUse; // TODO: update when odin has been updated
    case baldr::Use::kFootway:
      return TripLeg_Use_kFootwayUse;
    case baldr::Use::kElevator:
      return TripLeg_Use_kElevatorUse;
    case baldr::Use::kSteps:
      return TripLeg_Use_kStepsUse;
    case baldr::Use::kEscalator:
      return TripLeg_Use_kEscalatorUse;
    case baldr::Use::kPath:
      return TripLeg_Use_kPathUse;
    case baldr::Use::kPedestrian:
      return TripLeg_Use_kPedestrianUse;
    case baldr::Use::kBridleway:
      return TripLeg_Use_kBridlewayUse;
    case baldr::Use::kPedestrianCrossing:
      return TripLeg_Use_kPedestrianCrossingUse;
    case baldr::Use::kRestArea:
      return TripLeg_Use_kRestAreaUse;
    case baldr::Use::kServiceArea:
      return TripLeg_Use_kServiceAreaUse;
    case baldr::Use::kOther:
      return TripLeg_Use_kOtherUse;
    case baldr::Use::kFerry:
      return TripLeg_Use_kFerryUse;
    case baldr::Use::kRailFerry:
      return TripLeg_Use_kRailFerryUse;
    case baldr::Use::kConstruction:
      return TripLeg_Use_kConstructionUse;
    case baldr::Use::kRail:
      return TripLeg_Use_kRailUse;
    case baldr::Use::kBus:
      return TripLeg_Use_kBusUse;
    case baldr::Use::kEgressConnection:
      return TripLeg_Use_kEgressConnectionUse;
    case baldr::Use::kPlatformConnection:
      return TripLeg_Use_kPlatformConnectionUse;
    case baldr::Use::kTransitConnection:
      return TripLeg_Use_kTransitConnectionUse;
      // Should not see other values
    default:
      // TODO should we throw a runtime error?
      return TripLeg_Use_kRoadUse;
  }
}

// matrix algo to string
const std::string& MatrixAlgoToString(const valhalla::Matrix::Algorithm algo);
// Get the string representing the incident-type
std::string incidentTypeToString(const valhalla::IncidentsTile::Metadata::Type& incident_type);
// Get the string representing the incident-Impact
const char* incidentImpactToString(const valhalla::IncidentsTile::Metadata::Impact& impact);
// Get the string representing the guidance view type
const std::string& GuidanceViewTypeToString(const valhalla::DirectionsLeg_GuidanceView_Type type);

// to use protobuflite we cant use descriptors which means we cant translate enums to strings
// and so we reimplement the ones we use here. newer versions of protobuf provide these even
// for the lite bindings so we avoid collisions by picking somewhat goofy names. an alternative
// which would allow us to delete this completely would be to target a newer protobuf version
bool Options_Action_Enum_Parse(const std::string& action, Options::Action* a);
const std::string& Options_Action_Enum_Name(const Options::Action action);
bool Costing_Enum_Parse(const std::string& costing, Costing::Type* c);
const std::string& Costing_Enum_Name(const Costing::Type costing);
bool ShapeMatch_Enum_Parse(const std::string& match, ShapeMatch* s);
const std::string& ShapeMatch_Enum_Name(const ShapeMatch match);
bool Options_Format_Enum_Parse(const std::string& format, Options::Format* f);
const std::string& Options_Format_Enum_Name(const Options::Format match);
const std::string& Options_Units_Enum_Name(const Options::Units unit);
bool FilterAction_Enum_Parse(const std::string& action, FilterAction* a);
const std::string& FilterAction_Enum_Name(const FilterAction action);
bool DirectionsType_Enum_Parse(const std::string& dtype, DirectionsType* t);
bool PreferredSide_Enum_Parse(const std::string& pside, valhalla::Location::PreferredSide* p);
bool RoadClass_Enum_Parse(const std::string& rc_name, valhalla::RoadClass* rc);
bool Location_Type_Enum_Parse(const std::string& type, Location::Type* t);
const std::string& Location_Type_Enum_Name(const Location::Type t);
const std::string& Location_SideOfStreet_Enum_Name(const Location::SideOfStreet s);
bool Options_ExpansionProperties_Enum_Parse(const std::string& prop, Options::ExpansionProperties* a);
bool Options_ExpansionAction_Enum_Parse(const std::string& action, Options::Action* a);
const std::string& Expansion_EdgeStatus_Enum_Name(const Expansion_EdgeStatus status);

std::pair<std::string, std::string>
travel_mode_type(const valhalla::DirectionsLeg_Maneuver& maneuver);

inline midgard::PointLL to_ll(const LatLng& ll) {
  return midgard::PointLL{ll.lng(), ll.lat()};
}

inline midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}

inline void from_ll(valhalla::Location* l, const midgard::PointLL& p) {
  l->mutable_ll()->set_lat(p.lat());
  l->mutable_ll()->set_lng(p.lng());
}

} // namespace valhalla
