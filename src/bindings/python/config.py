import json
import os

from .valhalla_build_config import config as _config, help_text as _help_text, optional as _optional

_global_config = dict()


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


def _create_config(path: str, c: dict, tile_dir: str, tile_extract: str, verbose: bool) -> None:
    # set a global config so that other modules can work with it
    global _global_config
    conf = c.copy()

    if os.path.exists(path) and not conf:
        # use the existing file if one exists and no config was passed
        with open(path) as f:
            conf = json.load(f)
    elif not conf:
        # if the file doesn't exist and no config was passed, raise
        raise ValueError("No local config file found, you need to specify a configuration to create one.")
    
    # Write the convenience stuff
    if tile_dir:
        conf["mjolnir"]["tile_dir"] = tile_dir
    if tile_extract:
        conf["mjolnir"]["tile_extract"] = tile_extract
    if verbose is True:
        conf["loki"]["logging"]["type"] = "std_out"
    elif verbose is False:
        conf["loki"]["logging"]["type"] = ""

    # Finally write the config to filesystem
    with open(path, 'w') as f:
        json.dump(conf, f, indent=2)

    _global_config = conf
