import json
from typing import Union, Callable

try:
    from .python_valhalla import _Actor
except ModuleNotFoundError:
    from python_valhalla import _Actor


def _wrapper(func: Callable, req: Union[str, dict]) -> Union[str, dict]:
    # return the type being passed, str -> str, dict -> dict
    if isinstance(req, dict):
        return json.loads(func(json.dumps(req)))
    elif not isinstance(req, str):
        raise ValueError(f"Request must be either of type str or dict")

    return func(req)


class Actor(_Actor):

    def Route(self, req: Union[str, dict]) -> Union[str, dict]:
        """Calculates a route."""
        return _wrapper(super().Route, req)

    def Locate(self, req: Union[str, dict]) -> Union[str, dict]:
        """Provides information about nodes and edges."""
        return _wrapper(super().Locate, req)

    def OptimizedRoute(self, req: Union[str, dict]) -> Union[str, dict]:
        """Optimizes the order of a set of waypoints by time."""
        return _wrapper(super().OptimizedRoute, req)

    def Matrix(self, req: Union[str, dict]) -> Union[str, dict]:
        """Computes the time and distance between a set of locations and returns them as a matrix table."""
        return _wrapper(super().Matrix, req)

    def Isochrone(self, req: Union[str, dict]) -> Union[str, dict]:
        """Calculates isochrones and isodistances."""
        return _wrapper(super().Isochrone, req)

    def TraceRoute(self, req: Union[str, dict]) -> Union[str, dict]:
        """Map-matching for a set of input locations, e.g. from a GPS."""
        return _wrapper(super().TraceRoute, req)

    def TraceAttributes(self, req: Union[str, dict]) -> Union[str, dict]:
        """Returns detailed attribution along each portion of a route calculated from a set of input locations, e.g. from a GPS trace."""
        return _wrapper(super().TraceAttributes, req)

    def Height(self, req: Union[str, dict]) -> Union[str, dict]:
        """Provides elevation data for a set of input geometries."""
        return _wrapper(super().Height, req)

    def TransitAvailable(self, req: Union[str, dict]) -> Union[str, dict]:
        """Lookup if transit stops are available in a defined radius around a set of input locations."""
        return _wrapper(super().TransitAvailable, req)

    def Expansion(self, req: Union[str, dict]) -> Union[str, dict]:
        """Returns all road segments which were touched by the routing algorithm during the graph traversal."""
        return _wrapper(super().Expansion, req)
