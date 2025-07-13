from typing import overload, Final, Tuple, List, Optional

class GraphId:
    """Identifier of a node or an edge within the tiled, hierarchical graph.
    Includes the tile Id, hierarchy level, and a unique identifier within
    the tile/level.
    """

    # The integer representation of the bit-fielded GraphId.
    value: Final

    def __init__(self) -> None:
        """Constructs an invalid GraphId"""

    @overload
    def __init__(self, tile_id: int, level: int, id: int) -> None:
        """Constructs a GraphId from its portions."""

    @overload
    def __init__(self, value: int) -> None:
        """Constructs a GraphId from its integer value, e.g. 118931 == 3/14866/0."""

    @overload
    def __init__(self, value: str) -> None:
        """Constructs a GraphId from its string representation, e.g. "2/71944/0"."""

    def __add__(self, value: int) -> None:
        """Increments the id portion by value"""

    def __iadd__(self, value: int) -> None:
        """Increments the id portion by value"""

    def __eq__(self, other: GraphId) -> None:
        """Equality operator"""

    def __ne__(self, other: GraphId) -> None:
        """Inequality operator"""

    def __bool__(self) -> None:
        """True if Is_Valid()."""

    def tileid() -> int:
        """Gets the tile Id."""

    def level() -> int:
        """Gets the hierarchy level."""

    def id() -> int:
        """Gets the identifier within the hierarchy level."""

    def Is_Valid() -> bool:
        """Returns true if the id is valid."""

    def Tile_Base() -> GraphId:
        """Returns a GraphId omitting the id of the of the object within the level.
        Construct a new GraphId with the Id portion omitted.
        """

    def tile_value() -> int:
        """Returns a value indicating the tile (level and tile id) of the graph Id."""

def get_tile_base_lon_lat(graph_id: GraphId) -> Tuple[float, float]:
    """Get the geographic coordinate of the south-western corner of this graph_id's tile.

    :param graph_id: The tile's or object's GraphId
    :returns: The lon/lat coordinate of the SW corner of the tile
    """

def get_tile_id_from_lon_lat(level: int, coord: Tuple[float, float]) -> GraphId:
    """Get the tile at this hierarchy level and geographic coordinate.

    :param level: The hierarchy level of the searched tiles.
    :param coord: The geographic coordinate to intersect with tiles.
    :returns: GraphId of found tile.
    :raises ValueError: When the level or coord are invalid.
    """

def get_tile_ids_from_bbox(
    minx: float, miny: float, maxx: float, maxy: float, levels: Optional[List[int]] = [0, 1, 2]
) -> List[GraphId]:
    """Returns all tiles GraphIds for the specified levels (default: all),
    which intersect the bbox.

    :param minx: The bbox's minimum longitude.
    :param miny: The bbox's minimum latitude.
    :param maxx: The bbox's maximum longitude.
    :param maxy: The bbox's maximum latitude.
    :param levels: The hierarchy levels for which to find tiles.
    :returns: The list of tile GraphIds which intersect the bounding box
    :raises ValueError: When the level(s) or coord are invalid.
    """
