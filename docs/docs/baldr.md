# Baldr

Baldr serves as a set of routing-specific data structures for use within other pieces of the Valhalla library. In keeping with the Norse mythological theme, the name [Baldr](https://en.wikipedia.org/wiki/Baldr) was chosen as a backronym standing for: Base ALgorithms and Data Resource. Since Baldr deals mostly with accessing routing data and algorithms related to routing sub-problems.

Baldr is essentially a set of various data structures and algorithms which deal with things like: route data tiles, tile caching, hierarchical tile layout and tile data members such as nodes, edges and exits.

## Components

### `GraphId`

`GraphId` ([source](https://github.com/valhalla/valhalla/blob/master/valhalla/baldr/graphid.h)) is a **unique identifier** of a _node_ or an _edge_ within the [tiled, hierarchical graph](tiles.md).

It includes:

- Hierarchy level
- Tile index
- Node or edge index

!!! warning
       `GraphId` does **not** contain the information about the _kind_ of object it identifies. We cannot tell whether a `GraphId` represents a node or an edge just by looking at it.

       For example, an ID with object index 0 may represent _either_ a node _or_ an edge. In other words, we could (and usually do) have two objects -  a node and an edge - with the same identifier.

#### Implementation

Internally, Valhalla uses a **64-bit unsigned integer** value to represent `GraphId`. This representation supports efficient comparison and hashing operations and allows for reasonable ranges of field values.

Here's the bit layout of the value. The fields are presented with the most significant bit (MSb) first - see [Bit numbering](https://en.wikipedia.org/wiki/Bit_numbering) for more info.

```text
       MSb                                     LSb
       ▼                                       ▼
bit   64         46        25         3        0
pos    ┌──────────┬─────────┬─────────┬────────┐
       │ RESERVED │ id      │ tileid  │ level  │
       └──────────┴─────────┴─────────┴────────┘
size     18         21        22        3
```

#### Fields

Order is based on field position in bit layout, from field starting at the MSb to one ending at LSb.

| Field | Bits | Description |
|---|---|---|
| `RESERVED` | 18 | Spare, unused bits |
| `id` | 21 | Node or edge index |
| `tileid` | 22 | Tile index |
| `level` | 3 | Hierarchy level |

#### Examples

##### Valid `GraphId`

| Field | Bits | Decimal | Binary |
|---|---|---|---|
| RESERVED | 18 |  |  |
| `id` | 21 | 1234567 | `0b100101101011010000111` |
| `tileid` | 22 | 5869 | `0b0000000001011011101101` |
| `level` | 3 | 1 | `0b001` |

- Final value, decimal: `41425194497897`
- Final value, binary:

    ```python
    0b100101101011010000111_0000000001011011101101_001
    ```

##### Invalid `GraphId`

Invalid `GraphId` value has all non-reserved bits set to `1`:

```python
0x3fffffffffff
```

#### Code

Below are some sample Python functions:

```python linenums="1"
HIERARCHY_LEVEL_BITS = 3
HIERARCHY_LEVEL_MASK = 2**HIERARCHY_LEVEL_BITS - 1
MAX_HIERARCHY_LEVEL = HIERARCHY_LEVEL_MASK

TILE_INDEX_BITS = 22
TILE_INDEX_MASK = 2**TILE_INDEX_BITS - 1
MAX_TILE_INDEX = TILE_INDEX_MASK

OBJECT_INDEX_BITS = 21
OBJECT_INDEX_MASK = 2**OBJECT_INDEX_BITS - 1
MAX_OBJECT_INDEX = OBJECT_INDEX_MASK

def to_graph_id(
    hierarchy_level: int,
    tile_index: int,
    object_index: int,
) -> int:
    """Create 64-bit representation of `GraphId`."""

    assert 0 <= hierarchy_level <= MAX_HIERARCHY_LEVEL
    assert 0 <= tile_index <= MAX_TILE_INDEX
    assert 0 <= object_index <= MAX_OBJECT_INDEX

    x = (
        # 3 bits
        hierarchy_level
        # 22 bits, offset by previous field's width
        | (tile_index << HIERARCHY_LEVEL_BITS)
        # 21 bits, offset by the combined width of previous fields
        | (object_index << (HIERARCHY_LEVEL_BITS + TILE_INDEX_BITS))
    )

    return x


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
```

```python
>>> get_tile_index(73160266)
756425
>>> get_tile_index(142438865769)
37741
```

### GraphTileReader

TODO:
