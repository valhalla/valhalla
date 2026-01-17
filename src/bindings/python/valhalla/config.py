import json
import tempfile
from pathlib import Path
from typing import Dict, Tuple, Union
from typing import Optional as TypingOptional

from .valhalla_build_config import Optional, help_text
from .valhalla_build_config import config as default_config

__all__ = ["get_help", "get_config", "parse_and_validate_config"]


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

    config["mjolnir"]["tile_dir"] = (
        ""
        if isinstance(tile_dir, str) and not str(tile_dir)
        else str(Path(tile_dir).resolve(strict=True))
    )
    config["mjolnir"]["tile_extract"] = (
        ""
        if isinstance(tile_extract, str) and not str(tile_extract)
        else str(Path(tile_extract).resolve(strict=True))
    )
    config["mjolnir"]["logging"]["type"] = "std_out" if verbose else ""

    return config


def parse_and_validate_config(
    config: Union[Path, str, dict]
) -> Tuple[str, TypingOptional[str]]:
    """Parse and validate Valhalla configuration.

    Accepts configuration as Path, str (file path or JSON), or dict.
    Validates that required mjolnir settings exist and tile data is accessible.

    :param config: Valhalla configuration as Path, string, or dict
    :returns: (config_path, temp_file_path) where temp_file_path is None for non-dict configs
    :raises AttributeError: Invalid config type or missing mjolnir.tile_extract/tile_dir
    :raises FileNotFoundError: Config file not found, invalid JSON, or tile data doesn't exist
    :raises json.JSONDecodeError: Invalid JSON in string
    """
    config_path = None
    temp_file_path = None

    # Step 1: Parse config into a file path
    if isinstance(config, dict):
        # Create temp file for dict configs to avoid "File name too long" errors
        with tempfile.NamedTemporaryFile(
            "w", suffix=".json", prefix="valhalla_config_", delete=False
        ) as f:
            json.dump(config, f)
            config_path = f.name
            temp_file_path = f.name
    elif isinstance(config, str):
        # Could be file path or JSON string
        if Path(config).is_file():
            # It's a valid file path - use as-is
            config_path = config
        else:
            # Not a file - check if it looks like JSON (starts with { or [)
            stripped = config.strip()
            if stripped.startswith(("{", "[")):
                # Looks like JSON - try to parse and let JSONDecodeError propagate
                json.loads(config)
                config_path = config
            else:
                # Doesn't look like JSON - treat as file path that doesn't exist
                raise FileNotFoundError(f"Valhalla JSON config file doesn't exist: {config}")
    elif isinstance(config, Path):
        # Check if file exists before resolving
        if not config.is_file():
            raise FileNotFoundError(f"Valhalla JSON config file doesn't exist: {config}")
        config_path = str(config.resolve())
    else:
        raise AttributeError(f"Valhalla JSON config can't be of type {type(config)}")

    # Step 2: Load and validate config structure
    # If config_path is a file, read it; otherwise parse as JSON string
    if Path(config_path).is_file():
        with open(config_path) as f:
            config_dict = json.load(f)
    else:
        config_dict = json.loads(config_path)

    # Step 3: Validate mjolnir configuration
    tile_extract_fp = config_dict.get("mjolnir", {}).get("tile_extract")
    tile_dir = config_dict.get("mjolnir", {}).get("tile_dir")

    # Raise if neither exists
    if not tile_extract_fp and not tile_dir:
        raise AttributeError(
            "Valhalla config JSON is not valid: mjolnir.tile_extract and mjolnir.tile_dir are missing."
        )

    # Step 4: Validate tile data exists on disk
    tile_extract_exists = tile_extract_fp and Path(tile_extract_fp).is_file()
    tile_dir_exists = tile_dir and Path(tile_dir).is_dir()

    if not tile_extract_exists and not tile_dir_exists:
        tile_extract_path = Path(tile_extract_fp).resolve() if tile_extract_fp else None
        tile_dir_path = Path(tile_dir).resolve() if tile_dir else None
        raise FileNotFoundError(
            f"Neither mjolnir.tile_extract ({tile_extract_path}) nor mjolnir.tile_dir ({tile_dir_path}) exists. Can't load graph."
        )

    # Return config path and temp file path (None if not a temp file)
    return (config_path, temp_file_path)
