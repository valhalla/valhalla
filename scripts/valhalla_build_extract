#!/usr/bin/env python3

import argparse
from io import BytesIO
import json
import logging
from pathlib import Path
import struct
import sys
import tarfile
from tarfile import BLOCKSIZE
from typing import Dict

description = """Builds a tar extract from the tiles in mjolnir.tile_dir to the path specified in mjolnir.tile_extract."""

# "=" prefix means native byte order with standard size and no alignment:
# https://docs.python.org/3/library/struct.html#byte-order-size-and-alignment
STRUCT_FORMAT = '=LQ'
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)
INDEX_FILE = "index.bin"
TAR_ARCNAME_DIR = Path('valhalla_tiles')

parser = argparse.ArgumentParser(description=description)
parser.add_argument("-c", "--config", help="Absolute or relative path to the Valhalla config JSON.", type=Path)
parser.add_argument("-v", "--verbosity", help="Accumulative verbosity flags; -v: INFO, -vv: DEBUG", action='count', default=0)

# set up the logger basics
LOGGER = logging.getLogger(__name__)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)5s: %(message)s"))
LOGGER.addHandler(handler)


def get_padded_tar_size(data_size: int) -> int:
    """Returns the tar-internal file size, incl. header and NULL padding"""
    #      <header>    <-data->    <-------------------NULL padding---------------->
    return BLOCKSIZE + data_size + (BLOCKSIZE - (data_size % BLOCKSIZE or BLOCKSIZE))


def get_tile_info(in_path: Path, tiles: dict):
    """Recursively fills the passed map with tile path & size info"""
    for p in in_path.iterdir():
        if p.is_dir():
            get_tile_info(p, tiles)
        if p.is_file():
            tiles[p.resolve()] = get_padded_tar_size(p.stat().st_size)


def create_extract(tiles_fp: Path, extract_fp: Path):
    if tiles_fp.exists() and tiles_fp.is_dir():
        # collect the tile paths and file sizes
        files_stats: Dict[Path, int] = dict()
        get_tile_info(tiles_fp, files_stats)
        tiles_count = abs(len(list(filter(lambda f: not f.name.endswith('.gph'), files_stats))) - len(files_stats))

        index_size = STRUCT_SIZE * tiles_count
        index_bin = b''
        offset = get_padded_tar_size(index_size)  # initialize with the index file size inside the tar
        for tile_path, tile_size in files_stats.items():
            # extract level and ID portion to create a GraphId
            # tile_path is the absolute path to the tile
            try:
                tile_path = str(tile_path.relative_to(tiles_fp))[:-4]
                level, idx = tile_path.split('/', 1)
                tile_id = int(level) | (int(idx.replace('/', '')) << 3)
            except ValueError:
                # ignore invalid tile ids, so other files can be tarred
                continue

            LOGGER.debug(f"Tile {tile_path} (ID {tile_id}) with size: {tile_size}, tar offset: {offset}")

            # add 4 & 8 byte ints to the index file
            index_bin += struct.pack(STRUCT_FORMAT, tile_id, offset)
            offset += tile_size
    else:
        LOGGER.critical(f"Neither 'tile_extract': {extract_fp} nor 'tile_dir': {tiles_fp} were found on the filesystem.")
        sys.exit(1)

    # build the tar with the index file as the first one
    index_bin_fd = BytesIO(index_bin)
    index_bin_fd.seek(0)
    # write max 100 logs
    reporting_size = int(tiles_count / 100) or 1
    with tarfile.open(extract_fp, 'w') as tar:
        tarinfo = tarfile.TarInfo(str(TAR_ARCNAME_DIR.joinpath(INDEX_FILE)))
        tarinfo.size = index_size
        tar.addfile(tarinfo, index_bin_fd)

        for count, tile_path in enumerate(files_stats.keys()):
            arcname = TAR_ARCNAME_DIR.joinpath(tile_path.relative_to(tiles_fp))
            tar.add(str(tile_path), arcname=str(arcname))

            if count % reporting_size == 0:
                LOGGER.info(f"Tarred {count + 1}/{len(files_stats)} files.")

    index_bin_fd.close()

    print(f"Finished tarring {len(files_stats)} files: {extract_fp}")


if __name__ == '__main__':
    args = parser.parse_args()
    with open(args.config) as f:
        config = json.load(f)
    extract_fp: Path = Path(config["mjolnir"]["tile_extract"])
    tiles_fp: Path = Path(config["mjolnir"]["tile_dir"])

    # set the right logger level
    if args.verbosity == 0:
        LOGGER.setLevel(logging.CRITICAL)
    elif args.verbosity == 1:
        LOGGER.setLevel(logging.INFO)
    elif args.verbosity >= 2:
        LOGGER.setLevel(logging.DEBUG)

    create_extract(tiles_fp, extract_fp)


    ### logic for existing tar file, not sure if necessary ####

    # DISK_BLOCK = 4096
    # DISK_CHUNKS = 1024

    # index_bin_temp = BytesIO(b'')
    # index_size = 0
    # if extract_fp.exists() and extract_fp.is_file():
    #     with tarfile.open(config["tile_extract"]) as tar:
    #         offset = 0
    #         for ti in tar.getmembers():
    #             # get the tile ID
    #             _, level, idx = ti.name.split('/', 1)
    #             tile_id = int(level) | (int(idx.replace('/', '')) << 3)
    #
    #             # add the 32 & 64 bit fixed width to the binary
    #             index_bin_temp.write(struct.pack("LQ", tile_id, offset))
    #             index_size += struct.calcsize('LQ')
    #
    #             # extend offset with header block plus padded file contents
    #             offset += get_padded_tar_size(ti.size)

    # index_size = get_padded_tar_size(index_size)
    #
    # # add the index_size to the initial tile offsets
    # index_bin_temp.seek(0)
    # index_bin_final = BytesIO(b'')
    # # read 16 byte packets, unpack them, amend the offset and write back to final binary
    # while index_bin_temp.tell() != index_size:
    #     tile_id, old_offset = struct.unpack('LQ', index_bin_temp.read(16))
    #     index_bin_final.write(struct.pack('LQ', tile_id, old_offset + index_size))
    #
    # # stitch back the tar: first create the initial tar with only the index file in it
    # extract_temp_fp = extract_fp.parent.joinpath(extract_fp.stem + "_temp.tar")
    # with tarfile.open(extract_temp_fp, 'w') as tar:
    #     tarinfo = tarfile.TarInfo(INDEX_FILE)
    #     tarinfo.size = index_size
    #     tar.addfile(tarinfo, index_bin_final)
    # # then append the huge tar file and write to final tar in 4 MB chunks
    # with open(extract_temp_fp, 'ab') as out_f, open(extract_fp, 'rb') as in_f:
    #     while in_f.tell() != extract_fp.stat().st_size:
    #         out_f.write(in_f.read(DISK_BLOCK * DISK_CHUNKS))
