import json
from typing import Union

try:
    from .python_valhalla import _Actor
except ModuleNotFoundError:
    from python_valhalla import _Actor


def decorator(func):
    def wrapped(*args):
        if isinstance(args[1], dict):
            return json.loads(func(args[0], json.dumps(args[1])))
        elif not isinstance(args[1], str):
            raise ValueError("Request must be either of type str or dict")
        return func(*args)
    return wrapped


class Actor(_Actor):
    
    @decorator
    def route(self, req: Union[str, dict]):
        return super().route(req)

    @decorator
    def locate(self, req: Union[str, dict]):
        return super().locate(req)

    @decorator
    def isochrone(self, req: Union[str, dict]):
        return super().isochrone(req)

    @decorator
    def matrix(self, req: Union[str, dict]):
        return super().matrix(req)

    @decorator
    def trace_route(self, req: Union[str, dict]):
        return super().traceRoute(req)

    @decorator
    def trace_attributes(self, req: Union[str, dict]):
        return super().traceAttributes(req)

    @decorator
    def height(self, req: Union[str, dict]):
        return super().height(req)

    @decorator
    def expansion(self, req: Union[str, dict]):
        return super().expansion(req)

    @decorator
    def centroid(self, req: Union[str, dict]):
        return super().centroid(req)

    @decorator
    def status(self, req: Union[str, dict]):
        return super().status(req)
