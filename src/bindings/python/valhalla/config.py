from pathlib import Path
from typing import Union, Dict

from .valhalla_build_config import config as default_config, Optional, help_text


def _sanitize_config(dict_: dict = None) -> dict:
    """remove the "Optional" values from the config."""
    int_dict_ = dict_.copy()
    for k, v in int_dict_.items():
        if isinstance(v, Optional):
            del dict_[k]
        elif isinstance(v, dict):
            _sanitize_config(v)

    return dict_


def get_help() -> Dict[str, Union[Dict[str, str], str]]:
    """
    Returns the help dictionary with the same keys as the config JSON.
    """
    return help_text


def get_config(
    tile_extract: Union[str, Path] = "valhalla_tiles.tar",
    tile_dir: Union[str, Path] = "valhalla_tiles",
    verbose: bool = False,
) -> dict:
    """
    Returns a default Valhalla configuration.

    :param tile_extract: The file path (with .tar extension) of the tile extract (mjolnir.tile_extract), if present. Preferred over tile_dir.
    :param tile_dir: The directory path where the graph tiles are stored (mjolnir.tile_dir), if present.
    :param verbose: Whether you want to see Valhalla's logs on stdout (mjolnir.logging). Default False.
    """

    config = _sanitize_config(default_config.copy())

    config["mjolnir"]["tile_dir"] = str(Path(tile_dir).resolve())
    config["mjolnir"]["tile_extract"] = str(Path(tile_extract).resolve())
    config["mjolnir"]["logging"]["type"] = "std_out" if verbose else ""

    return config
