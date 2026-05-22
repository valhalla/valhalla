VALHALLA_PRINT_VERSION: str = "3.7.0"

class ValhallaError(RuntimeError):
    """
    Exception raised when a Valhalla operation fails.

    :param int code: Valhalla-internal error code.
    :param str message: Human-readable error message.
    :param int http_code: Corresponding HTTP status code.
    :param str http_message: Corresponding HTTP status message.
    """

class _Actor:
    """
    Valhalla's Actor class is used to call its actions, like route, isochrone, matrix etc.

    Configuration passed in either by an existing configuration JSON file path or in `dict` form,
    e.g. by calling valhalla.config.get_config(). In the latter case a temp file will be
    created.

    For details on parameters for each function consult Valhalla's documentation:
    https://github.com/valhalla/valhalla/blob/master/docs/docs/api
    """

    def __init__(self, config: str) -> None: ...
    def route(self, arg: str, /) -> str:
        """Calculates a route."""

    def locate(self, arg: str, /) -> str:
        """Provides information about nodes and edges."""

    def optimized_route(self, arg: str, /) -> str:
        """Optimizes the order of a set of waypoints by time."""

    def matrix(self, arg: str, /) -> str:
        """
        Computes the time and distance between a set of locations and returns them as a matrix table.
        """

    def isochrone(self, arg: str, /) -> str:
        """Calculates isochrones and isodistances."""

    def trace_route(self, arg: str, /) -> str:
        """Map-matching for a set of input locations, e.g. from a GPS."""

    def trace_attributes(self, arg: str, /) -> str:
        """
        Returns detailed attribution along each portion of a route calculated from a set of input locations, e.g. from a GPS trace.
        """

    def height(self, arg: str, /) -> str:
        """Computes the height for a set of input geometries."""

    def transit_available(self, arg: str, /) -> str:
        """
        Lookup if transit stops are available in a defined radius around a set of input locations.
        """

    def expansion(self, arg: str, /) -> str:
        """
        Returns all road segments which were touched by the routing algorithm during the graph traversal.
        """

    def centroid(self, arg: str, /) -> str:
        """
        Returns routes from all the input locations to the minimum cost meeting point of those paths.
        """

    def status(self, arg: str, /) -> str:
        """Returns nothing or optionally details about Valhalla's configuration."""

    def tile(self, arg: str, /) -> bytes:
        """
        Returns a vector tile (MVT binary data) with a bounding box feature for the given z/x/y tile coordinates.
        """
