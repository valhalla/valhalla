using System;
using System.Collections.Generic;
using System.Text.Json;

namespace valhalla.baldr
{
    public class EdgeInfo
    {
        private EdgeInfoInner ei_;
        private NameInfo[] name_info_list_;
        private string encoded_shape_;
        private int extended_wayid2_;
        private int extended_wayid3_;
        private sbyte[] encoded_elevation_;
        private string names_list_;
        private int names_list_length_;
        private List<midgard.PointLL> shape_ = new List<midgard.PointLL>();
        private bool tag_cache_ready_ = false;
        private Dictionary<TaggedValue, string> tag_cache_ = new Dictionary<TaggedValue, string>();

        public EdgeInfo(char[] ptr, string names_list, int names_list_length)
        {
            names_list_ = names_list;
            names_list_length_ = names_list_length;

            ei_ = (EdgeInfoInner)BitConverter.ToInt32(ptr, 0);
            ptr = ptr[sizeof(EdgeInfoInner)..];

            name_info_list_ = new NameInfo[name_count()];
            for (int i = 0; i < name_count(); i++)
            {
                name_info_list_[i] = (NameInfo)BitConverter.ToInt32(ptr, 0);
                ptr = ptr[sizeof(NameInfo)..];
            }

            encoded_shape_ = new string(ptr, 0, encoded_shape_size());
            ptr = ptr[encoded_shape_size()..];

            extended_wayid2_ = extended_wayid3_ = 0;
            if (ei_.extended_wayid_size_ > 0)
            {
                extended_wayid2_ = (int)ptr[0];
                ptr = ptr[sizeof(byte)..];
            }
            if (ei_.extended_wayid_size_ > 1)
            {
                extended_wayid3_ = (int)ptr[0];
                ptr = ptr[sizeof(byte)..];
            }

            encoded_elevation_ = new sbyte[ptr.Length];
            for (int i = 0; i < ptr.Length; i++)
            {
                encoded_elevation_[i] = (sbyte)ptr[i];
            }
        }

        public NameInfo GetNameInfo(int index)
        {
            if (index < ei_.name_count_)
            {
                return name_info_list_[index];
            }
            else
            {
                throw new Exception("StreetNameOffset index was out of bounds");
            }
        }

        public List<string> GetNames()
        {
            List<string> names = new List<string>();
            for (int i = 0; i < name_count(); i++)
            {
                if (!name_info_list_[i].tagged_)
                {
                    if (name_info_list_[i].name_offset_ < names_list_length_)
                    {
                        names.Add(names_list_.Substring(name_info_list_[i].name_offset_));
                    }
                    else
                    {
                        throw new Exception("GetNames: offset exceeds size of text list");
                    }
                }
            }
            return names;
        }

        public List<Tuple<string, bool>> GetNames(bool include_tagged_values)
        {
            List<Tuple<string, bool>> name_type_pairs = new List<Tuple<string, bool>>();
            for (int i = 0; i < name_count(); i++)
            {
                if (name_info_list_[i].tagged_ && !include_tagged_values)
                {
                    continue;
                }
                if (name_info_list_[i].tagged_)
                {
                    if (name_info_list_[i].name_offset_ < names_list_length_)
                    {
                        string name = names_list_.Substring(name_info_list_[i].name_offset_);
                        if (IsNonLiguisticTagValue(name[0]))
                        {
                            name_type_pairs.Add(new Tuple<string, bool>(name.Substring(1), false));
                        }
                    }
                    else
                    {
                        throw new Exception("GetNames: offset exceeds size of text list");
                    }
                }
                else if (name_info_list_[i].name_offset_ < names_list_length_)
                {
                    name_type_pairs.Add(new Tuple<string, bool>(names_list_.Substring(name_info_list_[i].name_offset_), name_info_list_[i].is_route_num_));
                }
                else
                {
                    throw new Exception("GetNames: offset exceeds size of text list");
                }
            }
            return name_type_pairs;
        }

        public List<string> GetLinguisticTaggedValues()
        {
            List<string> names = new List<string>();
            for (int i = 0; i < name_count(); i++)
            {
                if (name_info_list_[i].tagged_)
                {
                    if (name_info_list_[i].name_offset_ < names_list_length_)
                    {
                        string name = names_list_.Substring(name_info_list_[i].name_offset_);
                        if (name[0] == (char)TaggedValue.kLinguistic)
                        {
                            name = name.Substring(1);
                            while (name[0] != '\0')
                            {
                                var header = midgard.unaligned_read<linguistic_text_header_t>(name);
                                names.Add(name.Substring(0, kLinguisticHeaderSize) + name.Substring(kLinguisticHeaderSize, header.length_));
                                name = name.Substring(header.length_ + kLinguisticHeaderSize);
                            }
                        }
                    }
                    else
                    {
                        throw new Exception("GetTaggedNames: offset exceeds size of text list");
                    }
                }
            }
            return names;
        }

        public List<Tuple<string, bool, byte>> GetNamesAndTypes(bool include_tagged_values)
        {
            List<Tuple<string, bool, byte>> name_type_pairs = new List<Tuple<string, bool, byte>>();
            for (int i = 0; i < name_count(); i++)
            {
                if (name_info_list_[i].tagged_ && !include_tagged_values)
                {
                    continue;
                }
                if (name_info_list_[i].tagged_)
                {
                    if (name_info_list_[i].name_offset_ < names_list_length_)
                    {
                        string name = names_list_.Substring(name_info_list_[i].name_offset_);
                        byte tag = (byte)name[0];
                        if (IsNonLiguisticTagValue(name[0]))
                        {
                            name_type_pairs.Add(new Tuple<string, bool, byte>(name.Substring(1), false, tag));
                        }
                    }
                    else
                    {
                        throw new Exception("GetNamesAndTypes: offset exceeds size of text list");
                    }
                }
                else if (name_info_list_[i].name_offset_ < names_list_length_)
                {
                    name_type_pairs.Add(new Tuple<string, bool, byte>(names_list_.Substring(name_info_list_[i].name_offset_), name_info_list_[i].is_route_num_, 0));
                }
                else
                {
                    throw new Exception("GetNamesAndTypes: offset exceeds size of text list");
                }
            }
            return name_type_pairs;
        }

        public List<string> GetTaggedValues()
        {
            List<string> tagged_values = new List<string>();
            for (int i = 0; i < name_count(); i++)
            {
                if (name_info_list_[i].tagged_)
                {
                    if (name_info_list_[i].name_offset_ < names_list_length_)
                    {
                        string value = names_list_.Substring(name_info_list_[i].name_offset_);
                        if (value[0] != (char)TaggedValue.kLinguistic)
                        {
                            tagged_values.AddRange(parse_tagged_value(value));
                        }
                    }
                    else
                    {
                        throw new Exception("GetTaggedNames: offset exceeds size of text list");
                    }
                }
            }
            return tagged_values;
        }

        public Dictionary<TaggedValue, string> GetTags()
        {
            if (!tag_cache_ready_)
            {
                for (int i = 0; i < name_count(); i++)
                {
                    if (name_info_list_[i].tagged_)
                    {
                        if (name_info_list_[i].name_offset_ < names_list_length_)
                        {
                            string value = names_list_.Substring(name_info_list_[i].name_offset_);
                            if (value[0] != (char)TaggedValue.kLinguistic)
                            {
                                var contents = parse_tagged_value(value);
                                foreach (var c in contents)
                                {
                                    tag_cache_.Add((TaggedValue)value[0], c.Substring(1));
                                }
                            }
                        }
                        else
                        {
                            throw new Exception("GetTags: offset exceeds size of text list");
                        }
                    }
                }
                if (tag_cache_.Count > 0)
                {
                    tag_cache_ready_ = true;
                }
            }
            return tag_cache_;
        }

        public Dictionary<byte, Tuple<byte, byte, string>> GetLinguisticMap()
        {
            Dictionary<byte, Tuple<byte, byte, string>> index_linguistic_map = new Dictionary<byte, Tuple<byte, byte, string>>();
            for (int i = 0; i < name_count(); i++)
            {
                if (name_info_list_[i].tagged_)
                {
                    if (name_info_list_[i].name_offset_ < names_list_length_)
                    {
                        string name = names_list_.Substring(name_info_list_[i].name_offset_);
                        if (name[0] == (char)TaggedValue.kLinguistic)
                        {
                            name = name.Substring(1);
                            while (name[0] != '\0')
                            {
                                var header = midgard.unaligned_read<linguistic_text_header_t>(name);
                                Tuple<byte, byte, string> liguistic_attributes = new Tuple<byte, byte, string>(header.phonetic_alphabet_, header.language_, name.Substring(kLinguisticHeaderSize, header.length_));
                                byte name_index = header.name_index_;
                                var iter = index_linguistic_map.TryAdd(name_index, liguistic_attributes);
                                if (!iter && liguistic_attributes.Item1 > index_linguistic_map[name_index].Item1 && liguistic_attributes.Item1 != (byte)PronunciationAlphabet.kNone && liguistic_attributes.Item2 == index_linguistic_map[name_index].Item2)
                                {
                                    index_linguistic_map[name_index] = liguistic_attributes;
                                }
                                name = name.Substring(header.length_ + kLinguisticHeaderSize);
                            }
                        }
                    }
                    else
                    {
                        throw new Exception("GetLinguisticMap: offset exceeds size of text list");
                    }
                }
            }
            return index_linguistic_map;
        }

        public int GetTypes()
        {
            int types = 0;
            for (int i = 0; i < name_count(); i++)
            {
                NameInfo info = GetNameInfo(i);
                types |= (info.is_route_num_ ? 1 : 0) << i;
            }
            return types;
        }

        public List<midgard.PointLL> shape()
        {
            if (encoded_shape_ != null && shape_.Count == 0)
            {
                shape_ = midgard.decode7<List<midgard.PointLL>>(encoded_shape_, ei_.encoded_shape_size_);
            }
            return shape_;
        }

        public string encoded_shape()
        {
            return encoded_shape_ == null ? midgard.encode7(shape_) : encoded_shape_;
        }

        public List<sbyte> encoded_elevation(int length, out double interval)
        {
            if (!has_elevation())
            {
                interval = length;
                return new List<sbyte>();
            }
            interval = midgard.sampling_interval(length);
            int n = midgard.encoded_elevation_count(length);
            return new List<sbyte>(encoded_elevation_, 0, n);
        }

        public List<ConditionalSpeedLimit> conditional_speed_limits()
        {
            List<ConditionalSpeedLimit> limits = new List<ConditionalSpeedLimit>();
            foreach (var tag in GetTags())
            {
                if (tag.Key == TaggedValue.kConditionalSpeedLimits)
                {
                    limits.Add((ConditionalSpeedLimit)BitConverter.ToInt32(tag.Value.ToCharArray(), 0));
                }
            }
            return limits;
        }

        public sbyte layer()
        {
            foreach (var tag in GetTags())
            {
                if (tag.Key == TaggedValue.kLayer)
                {
                    if (tag.Value.Length != 1)
                    {
                        throw new Exception("layer must contain 1-byte value");
                    }
                    return (sbyte)tag.Value[0];
                }
            }
            return 0;
        }

        public Tuple<List<Tuple<float, float>>, int> levels()
        {
            foreach (var tag in GetTags())
            {
                if (tag.Key == TaggedValue.kLevels)
                {
                    return decode_levels(tag.Value);
                }
            }
            return new Tuple<List<Tuple<float, float>>, int>(new List<Tuple<float, float>>(), 0);
        }

        public bool includes_level(float lvl)
        {
            foreach (var tag in GetTags())
            {
                if (tag.Key == TaggedValue.kLevels)
                {
                    var decoded = decode_levels(tag.Value).Item1;
                    foreach (var range in decoded)
                    {
                        if (range.Item1 <= lvl && lvl <= range.Item2)
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        public List<string> level_ref()
        {
            List<string> values = new List<string>();
            foreach (var tag in GetTags())
            {
                if (tag.Key == TaggedValue.kLevelRef)
                {
                    values.Add(tag.Value);
                }
            }
            return values;
        }

        public JsonElement json()
        {
            var edge_info = new Dictionary<string, object>
            {
                { "way_id", (ulong)wayid() },
                { "bike_network", bike_network_json(bike_network()) },
                { "names", names_json(GetNames()) },
                { "shape", midgard.encode(shape()) }
            };

            var elev = mean_elevation();
            if (elev == kNoElevationData)
            {
                edge_info.Add("mean_elevation", null);
            }
            else
            {
                edge_info.Add("mean_elevation", (long)elev);
            }

            if (speed_limit() == kUnlimitedSpeedLimit)
            {
                edge_info.Add("speed_limit", "unlimited");
            }
            else
            {
                edge_info.Add("speed_limit", (ulong)speed_limit());
            }

            Dictionary<string, object> conditional_speed_limits = null;
            foreach (var tag in GetTags())
            {
                switch (tag.Key)
                {
                    case TaggedValue.kLayer:
                    case TaggedValue.kLinguistic:
                    case TaggedValue.kBssInfo:
                    case TaggedValue.kLevel:
                    case TaggedValue.kLevelRef:
                    case TaggedValue.kLandmark:
                    case TaggedValue.kTunnel:
                    case TaggedValue.kBridge:
                        break;
                    case TaggedValue.kLevels:
                        var levels = new List<object>();
                        var decoded = decode_levels(tag.Value).Item1;
                        var precision = decode_levels(tag.Value).Item2;
                        foreach (var range in decoded)
                        {
                            if (range.Item1 == range.Item2)
                            {
                                levels.Add((double)range.Item1 / precision);
                            }
                            else
                            {
                                levels.Add(new List<double> { (double)range.Item1 / precision, (double)range.Item2 / precision });
                            }
                        }
                        edge_info.Add("levels", levels);
                        break;
                    case TaggedValue.kConditionalSpeedLimits:
                        if (conditional_speed_limits == null)
                        {
                            conditional_speed_limits = new Dictionary<string, object>();
                        }
                        var l = (ConditionalSpeedLimit)BitConverter.ToInt32(tag.Value.ToCharArray(), 0);
                        conditional_speed_limits.Add(l.td_.ToString(), (ulong)l.speed_);
                        break;
                }
            }
            if (conditional_speed_limits != null)
            {
                edge_info.Add("conditional_speed_limits", conditional_speed_limits);
            }

            return JsonDocument.Parse(JsonSerializer.Serialize(edge_info)).RootElement;
        }

        private int name_count()
        {
            return ei_.name_count_;
        }

        private int encoded_shape_size()
        {
            return ei_.encoded_shape_size_;
        }

        private bool has_elevation()
        {
            return ei_.has_elevation_;
        }

        private int wayid()
        {
            return ei_.wayid_;
        }

        private int bike_network()
        {
            return ei_.bike_network_;
        }

        private int mean_elevation()
        {
            return ei_.mean_elevation_;
        }

        private int speed_limit()
        {
            return ei_.speed_limit_;
        }

        private Tuple<List<Tuple<float, float>>, int> decode_levels(string encoded)
        {
            int precision = 0;
            List<Tuple<float, float>> decoded = new List<Tuple<float, float>>();
            var ptr = encoded.ToCharArray();
            int size = parse_varint(ref ptr);
            var end = ptr[size..];
            int prec_power = parse_varint(ref ptr);
            if (prec_power > 0)
            {
                precision = (int)Math.Pow(10, prec_power);
            }
            bool prev = false;
            while (ptr != end)
            {
                int val = parse_varint(ref ptr);
                if (val == kLevelRangeSeparator)
                {
                    prev = false;
                    continue;
                }
                float f = precision == 0 ? val : (float)val / precision;
                if (!prev)
                {
                    decoded.Add(new Tuple<float, float>(f, f));
                    prev = true;
                }
                else
                {
                    decoded[decoded.Count - 1] = new Tuple<float, float>(decoded[decoded.Count - 1].Item1, f);
                }
            }
            return new Tuple<List<Tuple<float, float>>, int>(decoded, precision);
        }

        private int parse_varint(ref char[] encoded)
        {
            int byte_ = 0, shift = 0, result = 0;
            while ((byte_ & 0x80) != 0 || shift == 0)
            {
                byte_ = encoded[0];
                result |= (byte_ & 0x7f) << shift;
                shift += 7;
                encoded = encoded[1..];
            }
            return (result & 1) != 0 ? ~result >> 1 : result >> 1;
        }

        private List<string> parse_tagged_value(string ptr)
        {
            switch ((TaggedValue)ptr[0])
            {
                case TaggedValue.kLayer:
                case TaggedValue.kBssInfo:
                case TaggedValue.kLevel:
                case TaggedValue.kLevelRef:
                case TaggedValue.kTunnel:
                case TaggedValue.kBridge:
                    return new List<string> { ptr };
                case TaggedValue.kLandmark:
                    string landmark_name = ptr.Substring(10);
                    int landmark_size = landmark_name.Length + 10;
                    return new List<string> { ptr.Substring(0, landmark_size) };
                case TaggedValue.kLevels:
                    var start = ptr.Substring(1);
                    int size = parse_varint(ref start.ToCharArray());
                    return new List<string> { ptr.Substring(0, (start.Length + size) - ptr.Length) };
                case TaggedValue.kConditionalSpeedLimits:
                    return new List<string> { ptr.Substring(0, 1 + sizeof(ConditionalSpeedLimit)) };
                case TaggedValue.kLinguistic:
                default:
                    return new List<string>();
            }
        }

        private bool IsNonLiguisticTagValue(char ch)
        {
            return (TaggedValue)ch != TaggedValue.kLinguistic;
        }

        private Dictionary<string, bool> bike_network_json(int mask)
        {
            return new Dictionary<string, bool>
            {
                { "national", (mask & kNcn) != 0 },
                { "regional", (mask & kRcn) != 0 },
                { "local", (mask & kLcn) != 0 },
                { "mountain", (mask & kMcn) != 0 }
            };
        }

        private List<string> names_json(List<string> names)
        {
            return names;
        }
    }

    public struct EdgeInfoInner
    {
        public int wayid_;
        public int bike_network_;
        public int mean_elevation_;
        public int speed_limit_;
        public int name_count_;
        public int encoded_shape_size_;
        public int has_elevation_;
        public int extended_wayid_size_;
    }

    public struct NameInfo
    {
        public int name_offset_;
        public bool tagged_;
        public bool is_route_num_;
    }

    public struct linguistic_text_header_t
    {
        public byte phonetic_alphabet_;
        public byte language_;
        public byte name_index_;
        public int length_;
    }

    public struct ConditionalSpeedLimit
    {
        public int speed_;
        public TimeDomain td_;
    }

    public struct TimeDomain
    {
        public string ToString()
        {
            return "";
        }
    }

    public enum TaggedValue
    {
        kLayer,
        kBssInfo,
        kLevel,
        kLevelRef,
        kTunnel,
        kBridge,
        kLandmark,
        kLevels,
        kConditionalSpeedLimits,
        kLinguistic
    }

    public static class midgard
    {
        public static T unaligned_read<T>(string str)
        {
            return default(T);
        }

        public static List<T> decode7<T>(string str, int size)
        {
            return new List<T>();
        }

        public static string encode7(List<midgard.PointLL> shape)
        {
            return "";
        }

        public static double sampling_interval(int length)
        {
            return 0;
        }

        public static int encoded_elevation_count(int length)
        {
            return 0;
        }

        public static string encode(List<midgard.PointLL> shape)
        {
            return "";
        }
    }

    public class midgard
    {
        public struct PointLL
        {
        }
    }
}
