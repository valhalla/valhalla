from typing import List
import tarfile
from pathlib import Path
import os

try:
    from .python_valhalla import _BuildTiles
except ModuleNotFoundError:
    from python_valhalla import _BuildTiles


def BuildTiles(input_pbfs: List[str]):
    """
    Builds and tars the Valhalla tiles according to the config. Returns the path for the resulting tar file.

    :param List[str] input_pbfs: The full paths to the PBF files to be processed.
    """
    if not input_pbfs:
        raise ValueError("No PBF files specified.")

    result = _BuildTiles(input_pbfs)
    if result is False:
        raise RuntimeError("Building tiles failed.")

    return _tar_tiles()


def _tar_tiles():
    """Create a TAR ball at mjolnir.tile_extract from mjolnir.tile_dir"""
    from .config import _global_config

    # some sanity checks
    if not _global_config:
        raise RuntimeError("The service was not configured")
    tile_dir = Path(_global_config['mjolnir']['tile_dir'])
    if not tile_dir.is_dir():
        raise ValueError(f"mjolnir.tile_dir={tile_dir} is not a directory")
    tile_extract = Path(_global_config['mjolnir']['tile_extract'])
    if not tile_extract.parent.exists():
        raise ValueError(f"mjolnir.tile_extract={tile_extract} is not inside an existing directory")

    with tarfile.open(tile_extract, 'w') as tar:
        tar.add(tile_dir, arcname=os.path.basename(tile_dir))

    return str(tile_extract)