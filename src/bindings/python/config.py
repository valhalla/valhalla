import json
import os

from .valhalla_build_config import config as _config, help_text as _help_text, optional as _optional


def get_default() -> dict:
    """Returns the default Valhalla configuration."""
    c = _config.copy()
    for k, v in c['mjolnir'].items():
        if isinstance(v, _optional):
            c['mjolnir'][k] = ""
    return c

def get_help() -> dict:
    """Returns the help texts to the Valhalla configuration."""
    return _help_text

def _create_config(path: str, c: dict, tile_dir: str, tile_extract: str):
    conf = c.copy()
    # use the existing config if one exists and no changes are requested
    if os.path.exists(path) and not conf:
        with open(path) as f:
            conf = json.load(f)
    elif not conf:
        raise ValueError("No local config file found, you need to specify a configuration")
    
    if tile_dir:
        conf["mjolnir"]["tile_dir"] = tile_dir
    if tile_extract:
        conf["mjolnir"]["tile_extract"] = tile_extract

    with open(path, 'w') as f:
        json.dump(conf, f, indent=2)
