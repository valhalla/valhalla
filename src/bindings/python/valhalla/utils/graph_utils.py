"""Python wrapper for GraphUtils to support dict/Path/str config input."""

from pathlib import Path
from typing import Union

from ..config import parse_and_validate_config
from ._graph_utils import (
    GraphId as _GraphId,
)
from ._graph_utils import (
    _GraphUtils,
    get_tile_base_lon_lat,
    get_tile_id_from_lon_lat,
    get_tile_ids_from_bbox,
    get_tile_ids_from_ring,
)

__all__ = [
    "GraphId",
    "GraphUtils",
    "get_tile_base_lon_lat",
    "get_tile_id_from_lon_lat",
    "get_tile_ids_from_bbox",
    "get_tile_ids_from_ring",
]

# Re-export GraphId unchanged
GraphId = _GraphId


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
