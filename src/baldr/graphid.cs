using System;
using System.Text.Json;

namespace valhalla.baldr
{
    public struct GraphId
    {
        public uint value;

        public GraphId(uint value)
        {
            this.value = value;
        }

        public bool Is_Valid()
        {
            return value != 0;
        }

        public uint level()
        {
            return (value >> 24) & 0xFF;
        }

        public uint tileid()
        {
            return (value >> 12) & 0xFFF;
        }

        public uint id()
        {
            return value & 0xFFF;
        }

        public JsonElement json()
        {
            if (Is_Valid())
            {
                var idMap = new
                {
                    level = level(),
                    tile_id = tileid(),
                    id = id(),
                    value = value
                };
                return JsonDocument.Parse(JsonSerializer.Serialize(idMap)).RootElement;
            }
            return JsonDocument.Parse("null").RootElement;
        }

        public override string ToString()
        {
            return value.ToString();
        }
    }
}
