from pathlib import Path
from typing import Union

from .valhalla_build_config import config as default_config, Optional


def _sanitize_config(dict_: dict = None) -> dict:
    """remove the "Optional" values from the config."""
    int_dict_ = dict_.copy()
    for k, v in int_dict_.items():
        if isinstance(v, Optional):
            del dict_[k]
        elif isinstance(v, dict):
            _sanitize_config(v)

    return dict_


def get_config(
    tile_dir: Union[str, Path], tile_extract: Union[str, Path] = "valhalla_tiles.tar"
) -> dict:
    """
    Returns a default Valhalla configuration expecting an existing tile directory.

    :param tile_dir: The directory path where the graph tiles should be stored.
    :param tile_extract: The file path (with .tar extension) of the tile extract, if present.
    """

    config = _sanitize_config(default_config.copy())

    tile_dir = Path(tile_dir)
    tile_extract = Path(tile_extract)

    # insert the tile paths
    if not tile_dir.is_dir():
        raise FileNotFoundError(f"'tile_dir': {tile_dir.resolve()} is not an existing Valhalla graph.")

    config["mjolnir"]["tile_dir"] = str(tile_dir.resolve())
    config["mjolnir"]["tile_extract"] = str(tile_extract.resolve())

    return config
