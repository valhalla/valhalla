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


class Actor(_Actor):
    def __init__(self, config: Union[Path, str, dict]):
        """
        Valhalla's Actor class is used to call its actions, like route, isochrone, matrix etc.

        Configuration passed in either by an existing configuration JSON file path or in `dict` form,
        e.g. by calling valhalla.config.get_config(). In the latter case a temp file will be
        created.

        For details on parameters for each function consult Valhalla's documentation:
        https://github.com/valhalla/valhalla/blob/master/docs/docs/api
        """
        # Use shared validation function
        config_path, temp_file_path = parse_and_validate_config(config)
        self._config_path = config_path
        self._temp_file_path = temp_file_path  # Keep reference to prevent cleanup

        # Call C++ constructor
        super(Actor, self).__init__(self._config_path)

    @dict_or_str
    def route(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().route(req)

    @dict_or_str
    def locate(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().locate(req)

    @dict_or_str
    def isochrone(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().isochrone(req)

    @dict_or_str
    def matrix(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().matrix(req)

    @dict_or_str
    def trace_route(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().trace_route(req)

    @dict_or_str
    def trace_attributes(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().trace_attributes(req)

    @dict_or_str
    def height(self, req: Union[str, dict]):
        return super().height(req)

    @dict_or_str
    def transit_available(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().transit_available(req)

    @dict_or_str
    def expansion(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().expansion(req)

    @dict_or_str
    def centroid(self, req: Union[str, dict]) -> Union[str, dict]:
        return super().centroid(req)

    @dict_or_str
    def status(self, req: Union[str, dict] = "") -> Union[str, dict]:
        return super().status(req)

    def tile(self, req: Union[str, dict]) -> bytes:
        if isinstance(req, dict):
            return super().tile(json.dumps(req))
        elif not isinstance(req, str):
            raise ValueError("Request must be either of type str or dict")
        return super().tile(req)
