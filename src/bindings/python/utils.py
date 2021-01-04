import tarfile
from pathlib import Path
import os

from .config import _global_conf

def tar_tiles():
    """Creates a tar ball from mjolnir.tile_dir to mjolnir.tile_extract"""
    if not _global_conf:
        raise RuntimeError("The service was not configured")

    # do the paths exist?
    tile_dir = Path(_global_conf['mjolnir']['tile_dir'])
    if not tile_dir.exists():
        raise FileNotFoundError(f"mjolnir.tile_dir={tile_dir}: directory does not exist")

    tile_extract = Path(_global_conf['mjolnir']['tile_extract'])
    if not tile_extract.parent.exists():
        raise FileNotFoundError(f"mjolnir.tile_extract={tile_extract}: directory does not exist")
    
    with tarfile.open(tile_extract, "w") as tar:
        tar.add(tile_dir, arcname=tile_dir.parent)
