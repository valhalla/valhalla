using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace valhalla.baldr
{
    public class GraphReader
    {
        private readonly string tileDir;
        private readonly Dictionary<GraphId, GraphTile> tileCache;

        public GraphReader(string tileDir)
        {
            this.tileDir = tileDir;
            this.tileCache = new Dictionary<GraphId, GraphTile>();
        }

        public GraphTile GetGraphTile(GraphId graphId)
        {
            if (!graphId.Is_Valid())
            {
                return null;
            }

            if (tileCache.TryGetValue(graphId, out var cachedTile))
            {
                return cachedTile;
            }

            var tilePath = Path.Combine(tileDir, $"{graphId.level()}/{graphId.tileid()}.tile");
            if (!File.Exists(tilePath))
            {
                return null;
            }

            var tileData = File.ReadAllBytes(tilePath);
            var tile = new GraphTile(tileData);
            tileCache[graphId] = tile;

            return tile;
        }

        public HashSet<GraphId> GetTileSet()
        {
            var tiles = new HashSet<GraphId>();

            foreach (var levelDir in Directory.GetDirectories(tileDir))
            {
                var level = int.Parse(Path.GetFileName(levelDir));
                foreach (var tileFile in Directory.GetFiles(levelDir, "*.tile"))
                {
                    var tileId = int.Parse(Path.GetFileNameWithoutExtension(tileFile));
                    tiles.Add(new GraphId((uint)((level << 24) | tileId)));
                }
            }

            return tiles;
        }
    }

    public class GraphTile
    {
        private readonly byte[] data;

        public GraphTile(byte[] data)
        {
            this.data = data;
        }

        public byte[] GetData()
        {
            return data;
        }
    }
}
