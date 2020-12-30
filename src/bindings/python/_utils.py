import tarfile
import os

def _tar_tiles(source_dir: str, out_fp: str):
    if not out_fp or not source_dir:
        raise ValueError("Specify both mjolnir.tile_dir and mjolnir.tile_extract")
    with tarfile.open(out_fp, "w") as tar:
        tar.add(source_dir, arcname=os.path.basename(source_dir))
