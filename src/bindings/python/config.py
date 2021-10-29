from pathlib import Path
from typing import Union

from .valhalla_build_config import config as default_config, sanitize_config


def get_config(tile_dir: Union[str, Path], tile_extract: Union[str, Path] = "valhalla_tiles.tar") -> dict:
    """
    Returns a default Valhalla configuration expecting an existing tile directory.
    
    :param tile_dir: The directory path where the graph tiles should be stored.
    :param tile_extract: The file path (with .tar extension) of the tile extract, if present.
    """

    config = sanitize_config(default_config.copy())

    tile_dir = Path(tile_dir)
    tile_extract = Path(tile_extract)
    
    # insert the tile paths
    if not tile_dir.is_dir():
        raise FileNotFoundError(f"'tile_dir': {tile_dir.resolve()} is not an existing Valhalla graph.")

    config["mjolnir"]["tile_dir"] = str(tile_dir.resolve())
    config["mjolnir"]["tile_extract"] = str(tile_extract.resolve())

    return config
