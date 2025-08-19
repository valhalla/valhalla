import json
import tempfile
from pathlib import Path
from typing import Union

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
        # make sure there's a valhalla.json file
        if isinstance(config, dict):
            with tempfile.NamedTemporaryFile(
                "w", suffix=".json", prefix="valhalla_config_", delete=False
            ) as f:
                json.dump(config, f)
                self._config_path = f.name
        elif isinstance(config, str):
            if not Path(config).is_file():
                raise FileNotFoundError(f"Valhalla JSON config file doesn't exist: {config}")
            self._config_path = config
        elif isinstance(config, Path):
            self._config_path = str(config.resolve())
        else:
            raise AttributeError(f"Valhalla JSON config can't be of type {type(config)}")

        # test if there's an extract or tile_dir
        with open(self._config_path) as f:
            config = json.load(f)
        tile_extract_fp = config.get("mjolnir", {}).get("tile_extract")
        tile_dir = config.get("mjolnir", {}).get("tile_dir")

        # raise if neither exists
        if not tile_extract_fp and not tile_dir:
            raise AttributeError(
                "Valhalla config JSON is not valid: mjolnir.tile_extract and mjolnir.tile_dir are missing."
            )
        if not Path(config["mjolnir"]["tile_extract"]).is_file():
            if not Path(config["mjolnir"]["tile_dir"]).is_dir():
                raise FileNotFoundError(
                    f"Neither mjolnir.tile_extract ({Path(tile_extract_fp).resolve()}) nor mjolnir.tile_dir ({Path(tile_dir).resolve()}) exists. Can't load graph."
                )

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
