"""Python wrapper for GraphUtils to support dict/Path/str config input."""

from pathlib import Path
from typing import Union

from ..._valhalla import _GraphUtils
from ...config import parse_and_validate_config

__all__ = ["GraphUtils"]


class GraphUtils(_GraphUtils):
    """Utility class for accessing Valhalla graph data structures.

    Provides low-level access to graph tiles and edges using Valhalla's
    tiled routing data. Supports retrieving edge shapes and other graph properties.

    Configuration can be provided as:

    - Path to a JSON config file
    - JSON string
    - Python dict

    For dict input, the config is serialized to JSON internally.
    """

    def __init__(self, config: Union[Path, str, dict]):
        """Initialize GraphUtils with Valhalla configuration.

        :param config: Valhalla configuration as Path, JSON string, or dict.
                      Must contain valid mjolnir.tile_extract or mjolnir.tile_dir.
        :raises RuntimeError: When config is invalid or cannot be parsed
        :raises TypeError: When config is not Path, str, or dict
        :raises AttributeError: When config is missing required mjolnir settings
        :raises FileNotFoundError: When config file or tile data doesn't exist
        """
        # Use shared validation function
        config_path, temp_file_path = parse_and_validate_config(config)
        self._config_path = config_path
        self._temp_file_path = temp_file_path  # Keep reference to prevent cleanup

        # Call C++ constructor with validated config file path
        super().__init__(self._config_path)
