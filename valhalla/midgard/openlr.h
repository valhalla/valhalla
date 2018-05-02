#ifndef VALHALLA_MIDGARD_OPENLR_H_
#define VALHALLA_MIDGARD_OPENLR_H_

#include "pointll.h"
#include "base64.h"

namespace valhalla {
namespace midgard {

namespace OpenLR {

// Specify 1-byte padding for this struct
#pragma pack(push, 1)
struct TwoPointLinearReference  {
    using byte = std::uint8_t;

    // Byte 0
    byte unused_1 : 1;
    bool ArF1 : 1;
    bool no_point : 1;
    bool ArF0 : 1;
    bool has_attributes : 1;
    byte version  : 3;

    // Note: wire-format is big-endian
    // Bytes 1-6
    std::int32_t fixed_longitude : 24;
    std::int32_t fixed_latitude : 24;

    // Byte 7
    bool unused_2 : 2;
    byte first_frc : 3;
    byte first_fow : 3;

    // Byte 8
    byte lowest_frc_next_point : 3;
    byte first_bearing : 5;

    // Byte 9
    byte dist_to_next;

    // Bytes 10-13
    std::int16_t relative_longitude;
    std::int16_t relative_latitude;

    // Byte 14
    bool unused_3 : 2;
    byte last_frc : 3;
    byte last_fow : 3;

    // Byte 15
    bool unused_4 : 1;
    bool has_positive_offset : 1;
    bool has_negative_offset : 1;
    byte last_bearing : 5;

    // TODO: initialize everything
    TwoPointLinearReference() {
        version = 0b11;
        first_frc = 0;
        last_frc = 0;
        first_fow = 0;
        last_fow = 0;
        has_positive_offset = false;
        has_negative_offset = false;
    }

    // C++ implementation of https://en.wikipedia.org/wiki/Sign_function
    template <typename T> int sgn(const T val) const {
        return (T(0) < val) - (val < T(0));
    }

    std::int32_t deg2fixed(double deg) const {
        return sgn(deg) * 0.5 + (deg * std::pow(2, 24) / 360);
    }

    double fixed2deg(std::int32_t fixed) const {
        return ((fixed - sgn(fixed) * 0.5) * 360) / std::pow(2,24);
    }

    void setFirstCoordinate(const midgard::PointLL &p) {
        fixed_latitude = deg2fixed(p.lat());
        fixed_longitude = deg2fixed(p.lng());
    }

    midgard::PointLL getFirstCoordinate() const {
        return midgard::PointLL(
            fixed2deg(fixed_longitude),
            fixed2deg(fixed_latitude)
        );
    }

    // Precondition: setFirstCoordinate already called
    void setLastCoordinate(const midgard::PointLL &p) {
        relative_longitude = deg2fixed(p.lng()) - fixed_longitude;
        relative_latitude = deg2fixed(p.lat()) - fixed_latitude;
    }

    midgard::PointLL getLastCoordinate() const {
        const auto last_longitude = fixed_longitude + relative_longitude;
        const auto last_latitude = fixed_latitude + relative_latitude;
        return midgard::PointLL(
            fixed2deg(last_longitude),
            fixed2deg(last_latitude)
        );
    }

    unsigned getFirstFOW() const { return first_fow; }
    void setFirstFOW(unsigned fow_) { first_fow = fow_; }
    unsigned getFirstFRC() const { return first_frc; }
    void setFirstFRC(unsigned frc_) { first_frc = frc_; }

    unsigned getLastFOW() const { return last_fow; }
    void setLastFOW(unsigned fow_) { last_fow = fow_; }
    unsigned getLastFRC() const { return last_frc; }
    void setLastFRC(unsigned frc_) { last_frc = frc_; }

    void setFirstBearing(double bearing_) { first_bearing = static_cast<std::uint8_t>(bearing_ / 360. * 255.); }
    double getFirstBearing() const { return first_bearing * 360./255.; }

    void setLastBearing(double bearing_) { last_bearing = static_cast<std::uint8_t>(bearing_ / 360. * 255.); }
    double getLastBearing() const { return last_bearing * 360./255.; }

    void setLength(double length) { dist_to_next = length / 58.6; }
    double getLength() const { return dist_to_next * 58.6; }

    static TwoPointLinearReference fromBase64(const std::string base64) {
        std::string binary;
        Base64::Decode(base64, &binary);

        assert(sizeof(TwoPointLinearReference) == binary.size());
        // TODO: fixme - quick hack to convert bytes to object
        TwoPointLinearReference *tmp = reinterpret_cast<TwoPointLinearReference *>(const_cast<char *>(binary.data()));
        return *tmp; // Be sure to return a copy
    }

};
#pragma pack(pop)

} // OpenLR
} // midgard
} // valhalla

#endif // VALHALLA_MIDGARD_OPENLR_H_