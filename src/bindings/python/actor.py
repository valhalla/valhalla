import json
from typing import Union

try:
    from .python_valhalla import _Actor
except ModuleNotFoundError:
    from python_valhalla import _Actor


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


class Actor(_Actor):
    @dict_or_str
    def route(self, req: Union[str, dict]):
        return super().route(req)

    @dict_or_str
    def locate(self, req: Union[str, dict]):
        return super().locate(req)

    @dict_or_str
    def isochrone(self, req: Union[str, dict]):
        return super().isochrone(req)

    @dict_or_str
    def matrix(self, req: Union[str, dict]):
        return super().matrix(req)

    @dict_or_str
    def trace_route(self, req: Union[str, dict]):
        return super().trace_route(req)

    @dict_or_str
    def trace_attributes(self, req: Union[str, dict]):
        return super().trace_attributes(req)

    @dict_or_str
    def height(self, req: Union[str, dict]):
        return super().height(req)

    @dict_or_str
    def transit_available(self, req: Union[str, dict]):
        return super().transit_available(req)

    @dict_or_str
    def expansion(self, req: Union[str, dict]):
        return super().expansion(req)

    @dict_or_str
    def centroid(self, req: Union[str, dict]):
        return super().centroid(req)

    @dict_or_str
    def status(self, req: Union[str, dict] = ""):
        return super().status(req)
