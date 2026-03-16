from typing import List, Tuple

__all__ = ["decode_polyline"]


def decode_polyline(
    polyline: str, precision: int = 6, order: str = "lnglat"
) -> List[Tuple[float, float]]:
    """Decodes an encoded ``polyline`` string with ``precision`` to a list of coordinate tuples.
    The coordinate ``order`` of the output can be ``lnglat`` or ``latlng``."""

    return _decode(polyline, precision=precision, order=order, is3d=False)


def _trans(value, index):
    """
    Copyright (c) 2014 Bruno M. Custódio
    Copyright (c) 2016 Frederick Jansen
    https://github.com/hicsail/polyline/commit/ddd12e85c53d394404952754e39c91f63a808656
    """
    byte, result, shift = None, 0, 0

    while byte is None or byte >= 0x20:
        byte = ord(value[index]) - 63
        index += 1
        result |= (byte & 0x1F) << shift
        shift += 5
        comp = result & 1

    return ~(result >> 1) if comp else (result >> 1), index


def _decode(expression, precision=5, order="lnglat", is3d=False):
    """
    Copyright (c) 2014 Bruno M. Custódio
    Copyright (c) 2016 Frederick Jansen
    https://github.com/hicsail/polyline/commit/ddd12e85c53d394404952754e39c91f63a808656

    Modified to be able to work with 3D polylines and a specified coordinate order.
    """
    coordinates, index, lat, lng, z, length, factor = (
        [],
        0,
        0,
        0,
        0,
        len(expression),
        float(10**precision),
    )

    while index < length:
        lat_change, index = _trans(expression, index)
        lng_change, index = _trans(expression, index)
        lat += lat_change
        lng += lng_change
        coord = (lat / factor, lng / factor) if order == "latlng" else (lng / factor, lat / factor)
        if not is3d:
            coordinates.append(coord)
        else:
            z_change, index = _trans(expression, index)
            z += z_change
            coordinates.append((*coord, z / 100))

    return coordinates
