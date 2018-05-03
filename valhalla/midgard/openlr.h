#ifndef VALHALLA_MIDGARD_OPENLR_H_
#define VALHALLA_MIDGARD_OPENLR_H_

#include "pointll.h"
#include "base64.h"

namespace valhalla {
namespace midgard {

namespace OpenLR {

namespace {
    constexpr double BEARING_BUCKET_SIZE = 360. / 255.;

    // Used to fill in bits for 24 and 16 bit signed integers
    constexpr uint64_t COMPLEMENT_MASK [] = {0xffffffff, 0xfffffffe, 0xfffffffc,
                        0xfffffff8, 0xfffffff0, 0xffffffe0, 0xffffffc0, 0xffffff80,
                        0xffffff00, 0xfffffe00, 0xfffffc00, 0xfffff800, 0xfffff000,
                        0xffffe000, 0xffffc000, 0xffff8000, 0xffff0000, 0xfffe0000,
                        0xfffc0000, 0xfff80000, 0xfff00000, 0xffe00000, 0xffc00000,
                        0xff800000, 0xff000000, 0xfe000000, 0xfc000000, 0xf8000000,
                        0xf0000000, 0xe0000000, 0xc0000000, 0x80000000, 0x00000000 };
}

// Specify 1-byte padding for this struct
#pragma pack(push, 1)
struct TwoPointLinearReference  {
    using byte = uint8_t;

    // Byte 0
    byte unused_1 : 1;
    bool ArF1 : 1;
    bool no_point : 1;
    bool ArF0 : 1;
    bool has_attributes : 1;
    byte version  : 3;

    // Note: wire-format is big-endian (MSB first)
    // Bytes 1-6
    byte fixed_longitude[3];
    byte fixed_latitude[3];

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
    byte relative_longitude[2];
    byte relative_latitude[2];

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

    // Inverse of fixed2deg - convert a decimal degrees value into
    // a signed integer, according to the OpenLR conversion formula
    int32_t deg2fixed(double deg) const {
        return sgn(deg) * 0.5 + (deg * std::pow(2, 24) / 360);
    }

    // Convert a big-endian sequence of bites into a signed integer
    int32_t bytes2fixed(const byte bytes[], const int count) const {
        int32_t value = 0;
        for (int i=0; i<count; i++) {
            value += bytes[count-i-1] << (i*8);
        }

        // If MSB is set, number is negative, so we need to fill
        // in all the upper bits in the target int32_t to convert
        // to a two's complement value
        if ((bytes[0] & 0x80) == 0x80) {
            // Convert value to two's complement negative
            value |= COMPLEMENT_MASK[count * 8];
        }
        return value;
    }

    // Convert a signed integer into a decimal degree value, according
    // to the OpenLR conversion formula.
    double fixed2deg(const int value) const {
        return ((value - sgn(value) * 0.5) * 360) / std::pow(2,24);
    }

/*
    TODO:
    void setFirstCoordinate(const midgard::PointLL &p) {
        fixed_latitude = deg2fixed(p.lat());
        fixed_longitude = deg2fixed(p.lng());
    }
    */

    midgard::PointLL getFirstCoordinate() const {
        return midgard::PointLL(
            fixed2deg(bytes2fixed(fixed_longitude,3)),
            fixed2deg(bytes2fixed(fixed_latitude,3))
        );
    }

/*
    TODO:
    // Precondition: setFirstCoordinate already called
    void setLastCoordinate(const midgard::PointLL &p) {
        relative_longitude = deg2fixed(p.lng()) - fixed_longitude;
        relative_latitude = deg2fixed(p.lat()) - fixed_latitude;
    }
    */

    midgard::PointLL getLastCoordinate() const {
        return midgard::PointLL(
            fixed2deg(bytes2fixed(fixed_longitude,3)) + bytes2fixed(relative_longitude,2) / 100000.,
            fixed2deg(bytes2fixed(fixed_latitude,3)) + bytes2fixed(relative_latitude,2) / 100000.
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

    void setFirstBearing(double bearing_) { first_bearing = static_cast<uint8_t>(bearing_ / 360. * 255.); }
    double getFirstBearing() const { return first_bearing * 360./255.; }

    void setLastBearing(double bearing_) { last_bearing = static_cast<uint8_t>(bearing_ / 360. * 255.); }
    double getLastBearing() const { return last_bearing * 360./255.; }

    void setLength(double length) { dist_to_next = length / 58.6; }
    double getLength() const { return dist_to_next * 58.6; }

    static TwoPointLinearReference fromBase64(const std::string &base64) {
        std::string binary;
        Base64::Decode(base64, &binary);

        assert(sizeof(TwoPointLinearReference) <= binary.size());

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