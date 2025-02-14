using System;
using System.Collections.Generic;

namespace valhalla.baldr
{
    public enum AccessType
    {
        kHazmat,
        kMaxHeight,
        kMaxWidth,
        kMaxLength,
        kMaxWeight,
        kMaxAxleLoad,
        kTimedAllowed,
        kTimedDenied,
        kDestinationAllowed,
        kMaxAxles
    }

    public class AccessRestriction
    {
        private uint edgeindex_;
        private uint type_;
        private uint modes_;
        private uint spare_;
        private ulong value_;

        private static readonly Dictionary<AccessType, string> type_to_string = new Dictionary<AccessType, string>
        {
            { AccessType.kHazmat, "hazmat" },
            { AccessType.kMaxHeight, "max_height" },
            { AccessType.kMaxWidth, "max_width" },
            { AccessType.kMaxLength, "max_length" },
            { AccessType.kMaxWeight, "max_weight" },
            { AccessType.kMaxAxleLoad, "max_axle_load" },
            { AccessType.kTimedAllowed, "timed_allowed" },
            { AccessType.kTimedDenied, "timed_denied" },
            { AccessType.kDestinationAllowed, "destination_allowed" },
            { AccessType.kMaxAxles, "max_axles" }
        };

        public AccessRestriction(uint edgeindex, AccessType type, uint modes, ulong value)
        {
            edgeindex_ = edgeindex;
            type_ = (uint)type;
            modes_ = modes;
            spare_ = 0;
            value_ = value;
        }

        public uint edgeindex()
        {
            return edgeindex_;
        }

        public void set_edgeindex(uint edgeindex)
        {
            edgeindex_ = edgeindex;
        }

        public AccessType type()
        {
            return (AccessType)type_;
        }

        public uint modes()
        {
            return modes_;
        }

        public ulong value()
        {
            return value_;
        }

        public void set_value(ulong v)
        {
            value_ = v;
        }

        public Dictionary<string, object> json()
        {
            string restriction_type = "unsupported";
            if (type_to_string.TryGetValue(type(), out string found))
            {
                restriction_type = found;
            }

            var map = new Dictionary<string, object>
            {
                { "type", restriction_type },
                { "edge_index", (ulong)edgeindex() },
                { "bus", (modes_ & kBusAccess) != 0 },
                { "car", (modes_ & kAutoAccess) != 0 },
                { "emergency", (modes_ & kEmergencyAccess) != 0 },
                { "HOV", (modes_ & kHOVAccess) != 0 },
                { "pedestrian", (modes_ & kPedestrianAccess) != 0 },
                { "taxi", (modes_ & kTaxiAccess) != 0 },
                { "truck", (modes_ & kTruckAccess) != 0 },
                { "wheelchair", (modes_ & kWheelchairAccess) != 0 },
                { "moped", (modes_ & kMopedAccess) != 0 },
                { "motorcycle", (modes_ & kMotorcycleAccess) != 0 }
            };

            switch (type())
            {
                case AccessType.kTimedAllowed:
                case AccessType.kTimedDenied:
                case AccessType.kDestinationAllowed:
                    map["value"] = new Dictionary<string, object> { { "time_domain", value() } };
                    break;
                case AccessType.kMaxAxles:
                    map["value"] = value();
                    break;
                default:
                    map["value"] = Math.Round((double)value() * 0.01, 2);
                    break;
            }

            return map;
        }

        private const uint kBusAccess = 1 << 0;
        private const uint kAutoAccess = 1 << 1;
        private const uint kEmergencyAccess = 1 << 2;
        private const uint kHOVAccess = 1 << 3;
        private const uint kPedestrianAccess = 1 << 4;
        private const uint kTaxiAccess = 1 << 5;
        private const uint kTruckAccess = 1 << 6;
        private const uint kWheelchairAccess = 1 << 7;
        private const uint kMopedAccess = 1 << 8;
        private const uint kMotorcycleAccess = 1 << 9;
    }
}
