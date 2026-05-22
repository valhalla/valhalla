from collections.abc import Sequence
from typing import overload


class GraphId:
    """
    Identifier of a node or an edge within the tiled, hierarchical graph.
    Includes the tile Id, hierarchy level, and a unique identifier within
    the tile/level.
    """

    @overload
    def __init__(self) -> None:
        """Constructs an invalid GraphId"""

    @overload
    def __init__(self, tileid: int, level: int, id: int) -> None:
        """Constructs a GraphId from its portions."""

    @overload
    def __init__(self, arg: int, /) -> None:
        """Constructs a GraphId from its integer value, e.g. 118931 == 3/14866/0."""

    @overload
    def __init__(self, arg: str, /) -> None:
        """Constructs a GraphId from its string representation, e.g. "2/71944/0"."""

    @property
    def value(self) -> int:
        """The integer representation of the bit-fielded GraphId."""

    def tileid(self) -> int:
        """Gets the tile Id."""

    def level(self) -> int:
        """Gets the hierarchy level."""

    def id(self) -> int:
        """Gets the identifier within the hierarchy level."""

    def is_valid(self) -> bool:
        """Returns true if the id is valid."""

    def tile_base(self) -> GraphId:
        """
        Returns a GraphId omitting the id of the of the object within the level.
        Construct a new GraphId with the Id portion omitted.
        """

    def tile_value(self) -> int:
        """
        Returns a value indicating the tile (level and tile id) of the graph Id.
        """

    def __add__(self, arg: int, /) -> GraphId:
        """Increments the id portion by value"""

    def __iadd__(self, arg: int, /) -> GraphId:
        """Increments the id portion by value"""

    def __eq__(self, arg: GraphId, /) -> bool:
        """Equality operator"""

    def __ne__(self, arg: GraphId, /) -> bool:
        """Inequality operator"""

    def __bool__(self) -> bool:
        """True if is_valid()."""

    def __str__(self) -> str: ...

    def __fspath__(self) -> str: ...

    def __repr__(self) -> str: ...

    def __getstate__(self) -> tuple[int]: ...

    def __setstate__(self, arg: tuple[int], /) -> None: ...

def get_tile_base_lon_lat(graph_id: GraphId) -> tuple:
    """
    Get the geographic coordinate of the south-western corner of this graph_id's tile.

    :param graph_id: The tile's or object's GraphId
    :returns: The lon/lat coordinate of the SW corner of the tile
    """

def get_tile_id_from_lon_lat(level: int, coord: tuple) -> GraphId:
    """
    Get the tile at this hierarchy level and geographic coordinate.

    :param level: The hierarchy level of the searched tiles.
    :param coord: The geographic coordinate to intersect with tiles.
    :returns: GraphId of found tile.
    :raises ValueError: When the level or coord are invalid.
    """

def get_tile_ids_from_bbox(minx: float, miny: float, maxx: float, maxy: float, levels: Sequence[int] = []) -> list[GraphId]:
    """
    Returns all tiles GraphIds for the specified levels (default: all),
    which intersect the bbox.

    :param minx: The bbox's minimum longitude.
    :param miny: The bbox's minimum latitude.
    :param maxx: The bbox's maximum longitude.
    :param maxy: The bbox's maximum latitude.
    :param levels: The hierarchy levels for which to find tiles.
    :returns: The list of tile GraphIds which intersect the bounding box
    :raises ValueError: When the level(s) or coord are invalid.
    """

def get_tile_ids_from_ring(ring_coords: Sequence[tuple], levels: Sequence[int] = []) -> list[GraphId]:
    """
    Returns all tile GraphIds for the specified levels (default: all),
    which intersect or are contained within the polygon ring. The ring is
    assumed and coerced to be an outer ring. It's automatically closed if
    the last coordinate does not match the first.

    :param coords: List of (lon, lat) tuples forming a closed ring (polygon boundary).
                   Must have at least 3 coordinates.
    :param levels: The hierarchy levels for which to find tiles.
    :returns: The list of tile GraphIds which intersect or are inside the ring.
    :raises ValueError: When the level(s), coords, or ring size are invalid.
    """

class _GraphUtils:
    """
    C++ binding for GraphUtils (internal use - prefer GraphUtils wrapper).

    Manages a GraphReader for efficient access to tiles and edges.
    Initialize once and reuse for multiple edge queries.
    """

    def __init__(self, config: str) -> None:
        """
        Initialize _GraphUtils with Valhalla configuration.

        :param config: Valhalla configuration as JSON string or path to config file
        :raises RuntimeError: When config is invalid
        """

    def get_edge_shape(self, edge_id: GraphId) -> list[tuple[float, float]]:
        """
        Get the shape (polyline) for an edge as a list of (lon, lat) tuples.

        :param edge_id: GraphId of the edge
        :returns: List of (lon, lat) tuples representing the edge geometry
        :raises RuntimeError: When the tile or edge is not found
        """
