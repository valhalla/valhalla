# Tile Specifications

## Hierarchies/Levels

Tiles are arranged into a hierarchy with three levels.

| Hierarchy level | Tile size, degrees | Content |
|---|---|---|
| 0 | 4° | Highway roads: motorway, trunk and primary. |
| 1 | 1° | Arterial roads: secondary and tertiary. |
| 2 | 0.25° | Local roads: unclassified, residential, service or other. |

At each level, the world ([WGS 84](https://en.wikipedia.org/wiki/World_Geodetic_System#WGS_84)) is split into rectangular tiles with a specific size using the bounding box `(-180, -90, 180, 90)`. The rows and columns start from the _bottom left_ and increase to the top right - tiles are row ordered increasing from west to east.

### The World at Level 0

The following image shows the world at level 0.

![Level 0](images/world_level0.png)
Image generated using <https://geojson.io>

Using bounding boxes for Germany, Pennsylvania, and NYC we can show how the regions would be split up into the 3 levels. Level 0 is colored in light blue. Level 1 is light green and level 2 is light red.

### Germany

![Germany](images/germany.png)
Image generated using <https://geojson.io>

### Pennsylvania

![Pennsylvania](images/pennsylvania.png)
Image generated using <https://geojson.io>

### NYC

![NYC](images/nyc.png)
Image generated using <https://geojson.io>

## Sample Tile Code

Below are some sample functions to help you obtain latitude and longitude coordinates, levels, tile ids, and lists of tiles that intersect a bounding box.

```python
valhalla_tiles = [
    {"level": 2, "size": 0.25},
    {"level": 1, "size": 1.0},
    {"level": 0, "size": 4.0},
]

LEVEL_BITS = 3
TILE_INDEX_BITS = 22
ID_INDEX_BITS = 21

LEVEL_MASK = (2**LEVEL_BITS) - 1
TILE_INDEX_MASK = (2**TILE_INDEX_BITS) - 1
ID_INDEX_MASK = (2**ID_INDEX_BITS) - 1

INVALID_ID = (
    (ID_INDEX_MASK << (TILE_INDEX_BITS + LEVEL_BITS))
    | (TILE_INDEX_MASK << LEVEL_BITS)
    | LEVEL_MASK
)


def get_tile_level(id):
    return id & LEVEL_MASK


def get_tile_index(id):
    return (id >> LEVEL_BITS) & TILE_INDEX_MASK


def get_index(id):
    return (id >> (LEVEL_BITS + TILE_INDEX_BITS)) & ID_INDEX_MASK


def tiles_for_bounding_box(left, bottom, right, top):
    # if this is crossing the anti meridian split it up and combine
    if left > right:
        east = tiles_for_bounding_box(left, bottom, 180.0, top)
        west = tiles_for_bounding_box(-180.0, bottom, right, top)
        return east + west
    # move these so we can compute percentages
    left += 180
    right += 180
    bottom += 90
    top += 90
    tiles = []
    # for each size of tile
    for tile_set in valhalla_tiles:
        # for each column
        for x in range(int(left / tile_set["size"]), int(right / tile_set["size"]) + 1):
            # for each row
            for y in range(
                int(bottom / tile_set["size"]), int(top / tile_set["size"]) + 1
            ):
                # give back the level and the tile index
                tiles.append(
                    (tile_set["level"], int(y * (360.0 / tile_set["size"]) + x))
                )
    return tiles


def get_tile_id(tile_level, lat, lon):
    level = list(filter(lambda x: x["level"] == tile_level, valhalla_tiles))[0]
    width = int(360 / level["size"])
    return int((lat + 90) / level["size"]) * width + int((lon + 180) / level["size"])


def get_ll(id):
    tile_level = get_tile_level(id)
    tile_index = get_tile_index(id)
    level = list(filter(lambda x: x["level"] == tile_level, valhalla_tiles))[0]
    width = int(360 / level["size"])
    height = int(180 / level["size"])
    return (
        int(tile_index / width) * level["size"] - 90,
        (tile_index % width) * level["size"] - 180,
    )
```

### Get the Level or Hierarchy

`get_tile_level(73160266)` returns a level of 2.  73160266 is a Valhalla Graphid.

`get_tile_level(142438865769)` returns a level of 1.  142438865769 is an Open Traffic Segment id.

### Get the Latitude and Longitude from an ID

`get_ll(73160266)` returns the bottom left corner of the level 2 tile.  (41.25, -73.75)

`get_ll(142438865769)` returns the bottom left corner of the level 1 tile.  (14.0, 121.0)

### Get the Tile ID from a Latitude, Longitude, and Level

`get_tile_id(0, 14.601879, 120.972545)`  2415 Tile would be on disk with the directory structure of `/0/002/415.gph`

`get_tile_id(1, 14.601879, 120.972545)`  37740 Tile would be on disk with the directory structure of `/1/037/740.gph`

`get_tile_id(2, 41.413203, -73.623787)`  756425 Tile would be on disk with the directory structure of `/2/000/756/425.gph`

### Get Tile ID from an ID

`get_tile_index(73160266)`  Returns the tile id 756425.

`get_tile_index(142438865769)`  Returns the tile id 37741.

### Get Tiles That Intersect a Bounding Box

`tiles_for_bounding_box(-74.251961,40.512764,-73.755405,40.903125)`  Returns a list of tiles for the NYC bounding box at each level.
`[(2, 752102), (2, 753542), (2, 752103), (2, 753543), (2, 752104), (2, 753544), (1, 46905), (1, 46906), (0, 2906)]`

### Hierarchy Level or Tile Index from `GraphId`

```python linenums="1"
HIERARCHY_LEVEL_BITS = 3
HIERARCHY_LEVEL_MASK = 2**HIERARCHY_LEVEL_BITS - 1

TILE_INDEX_BITS = 22
TILE_INDEX_MASK = 2**TILE_INDEX_BITS - 1

OBJECT_INDEX_BITS = 21
OBJECT_INDEX_MASK = 2**OBJECT_INDEX_BITS - 1


def get_hierarchy_level(graph_id: int) -> int:
    """Hierarchy level from 64-bit representation of `GraphId`."""

    return graph_id & HIERARCHY_LEVEL_MASK


def get_tile_index(graph_id: int) -> int:
    """Tile index from 64-bit representation of `GraphId`."""

    offset = HIERARCHY_LEVEL_BITS

    return (graph_id >> offset) & TILE_INDEX_MASK


def get_object_index(graph_id: int) -> int:
    """Object (node or edge) index from 64-bit representation of `GraphId`."""

    offset = HIERARCHY_LEVEL_BITS + TILE_INDEX_BITS

    return (graph_id >> offset) & OBJECT_INDEX_MASK
```

```python
>>> get_hierarchy_level(73160266)
2
>>> get_hierarchy_level(142438865769)
1
>>> get_tile_index(73160266)
756425
>>> get_tile_index(142438865769)
37741
```
