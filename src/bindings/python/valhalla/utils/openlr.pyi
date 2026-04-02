from enum import IntEnum
from typing import Optional, Sequence

def decimal2integer(v: float) -> int:
    """Transform decimal coordinates into OpenLR integer representation.

    :param v: Coordinate in decimal degrees.
    :type v: float
    :return: Integer encoded coordinate.
    :rtype: int
    """

def integer2decimal(v: int) -> float:
    """Transform OpenLR integer coordinates into decimal degrees.

    :param v: Integer encoded coordinate.
    :type v: int
    :return: Coordinate in decimal degrees.
    :rtype: float
    """

def bearing2integer(v: float) -> int:
    """Convert a bearing in degrees to OpenLR integer representation.

    :param v: Bearing in degrees.
    :type v: float
    :return: Encoded bearing bucket.
    :rtype: int
    """

def integer2bearing(v: int) -> float:
    """Convert an OpenLR integer bearing into degrees.

    :param v: Encoded bearing bucket.
    :type v: int
    :return: Bearing in degrees.
    :rtype: float
    """

def distance2integer(v: float) -> int:
    """Convert a distance in meters into OpenLR integer representation.

    :param v: Distance in meters.
    :type v: float
    :return: Encoded distance bucket.
    :rtype: int
    """

def integer2distance(v: int) -> float:
    """Convert an OpenLR integer distance into meters.

    :param v: Encoded distance bucket.
    :type v: int
    :return: Distance in meters.
    :rtype: float
    """

class FormOfWay(IntEnum):
    """OpenLR Form Of Way enumeration."""

    UNDEFINED = 0
    MOTORWAY = 1
    MULTIPLE_CARRIAGEWAY = 2
    SINGLE_CARRIAGEWAY = 3
    ROUNDABOUT = 4
    TRAFFICSQUARE = 5
    SLIPROAD = 6
    OTHER = 7

class Orientation(IntEnum):
    """OpenLR orientation enumeration for PointAlongLine references."""

    NoOrientation = 0
    FirstLrpTowardsSecond = 1
    SecondLrpTowardsFirst = 2
    BothDirections = 3

class SideOfTheRoad(IntEnum):
    """OpenLR side-of-the-road enumeration for PointAlongLine references."""

    DirectlyOnRoadOrNotApplicable = 0
    RightSideOfRoad = 1
    LeftSideOfRoad = 2
    BothSidesOfRoad = 3

class LocationReferencePoint:
    """OpenLR Location Reference Point (LRP).

    Represents a single location reference point used in an OpenLR
    line or point-along-line location.

    :param longitude: Longitude in decimal degrees.
    :type longitude: float

    :param latitude: Latitude in decimal degrees.
    :type latitude: float

    :param bearing: Bearing in degrees.
    :type bearing: float

    :param frc: Functional Road Class.
    :type frc: int

    :param fow: Form Of Way.
    :type fow: FormOfWay

    :param prev: Previous location reference point for relative coordinate encoding.
    :type prev: LocationReferencePoint | None

    :param distance: Distance to the next LRP in meters (quantizes to multiples of 58.6).
    :type distance: float

    :param lfrcnp: Lowest Functional Road Class to next point.
    :type lfrcnp: int
    """

    longitude: float
    latitude: float
    bearing: float
    distance: float
    frc: int
    lfrcnp: int
    fow: FormOfWay

    def __init__(
        self,
        longitude: float,
        latitude: float,
        bearing: float,
        frc: int,
        fow: FormOfWay,
        prev: Optional["LocationReferencePoint"] = ...,
        distance: float = ...,
        lfrcnp: int = ...,
    ) -> None: ...

class OpenLr:
    """OpenLR line or point-along-line location descriptor.

    Supports serialization to/from binary and Base64 OpenLR references.

    :param lrps: Sequence of location reference points.
    :type lrps: Sequence[LocationReferencePoint]

    :param positive_offset_bucket: Positive offset bucket value.
    :type positive_offset_bucket: int

    :param negative_offset_bucket: Negative offset bucket value.
    :type negative_offset_bucket: int

    :param point_along_line: Whether this is a PointAlongLine reference.
    :type point_along_line: bool

    :param orientation: Point orientation metadata.
    :type orientation: Orientation

    :param side_of_the_road: Side-of-road metadata.
    :type side_of_the_road: SideOfTheRoad
    """

    lrps: list[LocationReferencePoint]
    poff: int
    noff: int
    is_point_along_line: bool
    orientation: Orientation
    side_of_the_road: SideOfTheRoad

    def __init__(
        self,
        lrps: Sequence[LocationReferencePoint],
        positive_offset_bucket: int = ...,
        negative_offset_bucket: int = ...,
        point_along_line: bool = ...,
        orientation: Orientation = ...,
        side_of_the_road: SideOfTheRoad = ...,
    ) -> None: ...
    @staticmethod
    def from_binary(binary: bytes) -> "OpenLr":
        """Construct an OpenLR object from binary encoded data.

        :param binary: Raw OpenLR binary payload.
        :type binary: bytes
        :return: Parsed OpenLR object.
        :rtype: OpenLr
        """

    @staticmethod
    def from_base64(encoded: str) -> "OpenLr":
        """Construct an OpenLR object from a Base64 encoded string.

        :param encoded: Base64 encoded OpenLR reference.
        :type encoded: str
        :return: Parsed OpenLR object.
        :rtype: OpenLr
        """

    @property
    def first_coordinate(self) -> tuple[float, float]:
        """First coordinate of the location.

        :return: Tuple of ``(longitude, latitude)``.
        :rtype: tuple[float, float]
        """

    @property
    def last_coordinate(self) -> tuple[float, float]:
        """Last coordinate of the location.

        :return: Tuple of ``(longitude, latitude)``.
        :rtype: tuple[float, float]
        """

    @property
    def length(self) -> float:
        """Total encoded path length in meters.

        :return: Length in meters.
        :rtype: float
        """

    def to_binary(self) -> bytes:
        """Serialize the OpenLR reference to binary format.

        :return: Binary encoded OpenLR payload.
        :rtype: bytes
        """

    def to_base64(self) -> str:
        """Serialize the OpenLR reference to Base64 format.

        :return: Base64 encoded OpenLR reference.
        :rtype: str
        """
