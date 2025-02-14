using System;
using System.Collections.Generic;
using System.Text.Json;

namespace valhalla.baldr
{
    public class LaneConnectivity
    {
        public uint to_ { get; set; }
        public ulong from_ { get; set; }
        public string to_lanes_ { get; set; }
        public string from_lanes_ { get; set; }

        public LaneConnectivity(uint idx, ulong from, string to_lanes, string from_lanes)
        {
            if (from >= (1UL << 42))
            {
                throw new ArgumentOutOfRangeException("from way_id is too large");
            }

            to_ = idx;
            from_ = from;
            to_lanes_ = to_lanes;
            from_lanes_ = from_lanes;
        }

        public void set_to(uint idx)
        {
            to_ = idx;
        }

        public uint to()
        {
            return to_;
        }

        public ulong from()
        {
            return from_;
        }

        public string from_lanes()
        {
            return from_lanes_;
        }

        public string to_lanes()
        {
            return to_lanes_;
        }

        public bool operator <(LaneConnectivity other)
        {
            return to() < other.to();
        }

        public JsonElement json()
        {
            var map = new Dictionary<string, object>
            {
                { "to", to_ },
                { "from", from_ },
                { "to_lanes", to_lanes_ },
                { "from_lanes", from_lanes_ }
            };

            return JsonDocument.Parse(JsonSerializer.Serialize(map)).RootElement;
        }
    }

    public class LaneConnectivityLanes
    {
        private ulong value_;

        public LaneConnectivityLanes(string lanes)
        {
            value_ = 0;
            var tokens = lanes.Split('|');
            byte n = 1;
            foreach (var t in tokens)
            {
                set_lane(n++, byte.Parse(t));
            }
        }

        public string to_string()
        {
            var result = string.Empty;
            for (int i = 1; i <= 8; ++i)
            {
                var lane = get_lane((byte)i);
                if (lane != 0)
                {
                    result += (result == string.Empty ? "" : "|") + lane.ToString();
                }
            }
            return result;
        }

        public void set_lane(byte n, byte lane)
        {
            if (n == 0 || n > 8 || lane > 8)
            {
                throw new ArgumentOutOfRangeException("lane or index out of bounds");
            }
            value_ |= (ulong)lane << ((n - 1) * 8);
        }

        public byte get_lane(byte n)
        {
            if (n == 0 || n > 8)
            {
                throw new ArgumentOutOfRangeException("index out of bounds");
            }
            return (byte)((value_ >> ((n - 1) * 8)) & 0xFF);
        }
    }
}
