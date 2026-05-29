import json
from pathlib import Path
from typing import Union

from .config import parse_and_validate_config

try:
    from ._valhalla import _Actor
except ModuleNotFoundError:
    from _valhalla import _Actor

__all__ = ["Actor"]


# TODO: wasteful for dict input/output; more reasonable would be to extend
#   the Actor's action C++ interfaces with a JSON arg
def dict_or_str(func):
    def wrapped(*args):
        # /status doesn't take any parameters
        if not len(args) > 1:
            return func(*args)

        if isinstance(args[1], dict):
            return json.loads(func(args[0], json.dumps(args[1])))
        elif not isinstance(args[1], str):
            raise ValueError("Request must be either of type str or dict")
        return func(*args)

    return wrapped


# The class docstring lives here as a string literal (not on `_Actor` in
# _valhalla.cc) because auto-intellisense tools are picky
class Actor(_Actor):
    """
    Valhalla's Actor class is used to call its actions, like route, isochrone, matrix etc.

    Configuration passed in either by an existing configuration JSON file path or in `dict` form,
    e.g. by calling valhalla.config.get_config(). In the latter case a temp file will be
    created.

    For details on parameters for each function consult Valhalla's documentation:
    https://github.com/valhalla/valhalla/blob/master/docs/docs/api
    """

    def __init__(self, config: Union[Path, str, dict]):
        # Use shared validation function
        config_path, temp_file_path = parse_and_validate_config(config)
        self._config_path = config_path
        self._temp_file_path = temp_file_path  # Keep reference to prevent cleanup

        # Call C++ constructor
        super(Actor, self).__init__(self._config_path)

    @dict_or_str
    def route(self, req: Union[str, dict]) -> Union[str, dict]:
        """Calculates a route."""
        return super().route(req)

    @dict_or_str
    def locate(self, req: Union[str, dict]) -> Union[str, dict]:
        """Provides information about nodes and edges."""
        return super().locate(req)

    @dict_or_str
    def optimized_route(self, req: Union[str, dict]) -> Union[str, dict]:
        """Optimizes the order of a set of waypoints."""
        return super().optimized_route(req)

    @dict_or_str
    def isochrone(self, req: Union[str, dict]) -> Union[str, dict]:
        """Calculates isochrones and isodistances."""
        return super().isochrone(req)

    @dict_or_str
    def matrix(self, req: Union[str, dict]) -> Union[str, dict]:
        """Computes the time and distance between a set of locations and returns them as a matrix table."""
        return super().matrix(req)

    @dict_or_str
    def trace_route(self, req: Union[str, dict]) -> Union[str, dict]:
        """Map-matching for a set of input locations, e.g. from a GPS."""
        return super().trace_route(req)

    @dict_or_str
    def trace_attributes(self, req: Union[str, dict]) -> Union[str, dict]:
        """Returns detailed attribution along each portion of a route calculated from a set of input locations, e.g. from a GPS trace."""
        return super().trace_attributes(req)

    @dict_or_str
    def height(self, req: Union[str, dict]):
        """Computes the height for a set of input geometries."""
        return super().height(req)

    @dict_or_str
    def transit_available(self, req: Union[str, dict]) -> Union[str, dict]:
        """Lookup if transit stops are available in a defined radius around a set of input locations."""
        return super().transit_available(req)

    @dict_or_str
    def expansion(self, req: Union[str, dict]) -> Union[str, dict]:
        """Returns all road segments which were touched by the routing algorithm during the graph traversal."""
        return super().expansion(req)

    @dict_or_str
    def centroid(self, req: Union[str, dict]) -> Union[str, dict]:
        """Returns routes from all the input locations to the minimum cost meeting point of those paths."""
        return super().centroid(req)

    @dict_or_str
    def status(self, req: Union[str, dict] = "") -> Union[str, dict]:
        """Returns nothing or optionally details about Valhalla's configuration."""
        return super().status(req)

    def tile(self, req: Union[str, dict]) -> bytes:
        """Returns a vector tile (MVT binary data) with a bounding box feature for the given z/x/y tile coordinates."""
        if isinstance(req, dict):
            return super().tile(json.dumps(req))
        elif not isinstance(req, str):
            raise ValueError("Request must be either of type str or dict")
        return super().tile(req)
