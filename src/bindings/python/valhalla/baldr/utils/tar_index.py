"""Read/write the tile index (index.bin) embedded as the first member of a
valhalla tile tar extract (the format valhalla_build_extract produces and
GraphReader consumes, locally or via HTTP range requests)."""

import struct
import tarfile
from pathlib import Path
from typing import NamedTuple, Union, overload

from .. import GraphId

# The wire format is owned by valhalla_build_extract
from ...valhalla_build_extract import INDEX_BIN_FORMAT, INDEX_BIN_SIZE, INDEX_FILE


class TarIndexEntry(NamedTuple):
    """One index.bin entry locating a tile inside the tar."""

    offset: int
    """Byte offset from the start of the tar to the tile's data (past its tar header)."""
    tile_id: GraphId
    """The tile's base GraphId."""
    size: int
    """The tile's size in bytes."""


@overload
def decode_tar_index(source: Union[bytes, bytearray, memoryview]) -> list[TarIndexEntry]: ...
@overload
def decode_tar_index(source: Union[str, Path]) -> list[TarIndexEntry]: ...


def decode_tar_index(source) -> list[TarIndexEntry]:
    """Read the tile index from a tar extract, or decode a raw ``index.bin`` body.

    Pass either the path of the (uncompressed) tile tar — the embedded index
    is read from its first member — or the index bytes themselves (e.g.
    fetched with an HTTP range request).

    :param source: Tar path, or raw index.bin bytes.
    :returns: One entry per tile, in file order.
    :raises ValueError: The tar's first member isn't ``index.bin``, or the
        bytes aren't a whole number of entries.
    """
    if isinstance(source, (bytes, bytearray, memoryview)):
        data = bytes(source)
    else:
        # turn the index file into bytes
        with tarfile.open(source, "r") as tar:
            index_tar_file = tar.next()
            if index_tar_file is None or index_tar_file.name != INDEX_FILE:
                raise ValueError(
                    f"first tar member is {index_tar_file.name if index_tar_file else 'missing'!r}, expected {INDEX_FILE!r}"
                )
            data = tar.extractfile(index_tar_file).read()

    if len(data) % INDEX_BIN_SIZE:
        raise ValueError(f"index size {len(data)} is not a multiple of {INDEX_BIN_SIZE}")
    return [
        TarIndexEntry(offset, GraphId(tile_id), size)
        for offset, tile_id, size in struct.iter_unpack(INDEX_BIN_FORMAT, data)
    ]


def write_tar_index(tar_path: Union[str, Path]) -> list[TarIndexEntry]:
    """Fill a tar extract's ``index.bin`` placeholder with the real tile index.

    The tar must have been created with an ``index.bin`` placeholder as its
    first member, sized for one entry per ``.gph`` member (the way
    valhalla_build_extract lays it out); the placeholder is overwritten in
    place from the members' actual offsets and sizes.

    :param tar_path: Path of the (uncompressed) tile tar.
    :returns: The entries that were written, in file order.
    :raises ValueError: The first member isn't ``index.bin`` or its size
        doesn't match the tile count.
    """
    entries = []
    with tarfile.open(tar_path, "r") as tar:
        index_tar_file = tar.next()
        if index_tar_file is None or index_tar_file.name != INDEX_FILE:
            raise ValueError(
                f"first tar member is {index_tar_file.name if index_tar_file else 'missing'!r}, expected {INDEX_FILE!r}"
            )
        for member in tar.getmembers():
            if member.name.endswith(".gph"):
                entries.append(
                    TarIndexEntry(member.offset_data, GraphId.from_tile_path(member.name), member.size)
                )
    if index_tar_file.size != len(entries) * INDEX_BIN_SIZE:
        raise ValueError(
            f"{INDEX_FILE} placeholder holds {index_tar_file.size // INDEX_BIN_SIZE} entries, need {len(entries)}"
        )

    with open(tar_path, "r+b") as tar_file:
        tar_file.seek(index_tar_file.offset_data)
        for entry in entries:
            tar_file.write(struct.pack(INDEX_BIN_FORMAT, entry.offset, entry.tile_id.value, entry.size))
    return entries
