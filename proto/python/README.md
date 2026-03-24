# Valhalla Proto 

Generated protocol buffers code for the Valhalla routing engine.

A small and handy Python package that contains generated code from Valhalla's protobuf definitions that can be used to communicate with either the HTTP server or the Actor directly via Valhalla's Python bindings.


## Installation 

Installation is as easy as: 

```bash 
pip install valhalla-proto
```

Note that this package uses [python-betterproto2](https://github.com/betterproto/python-betterproto2), which gives a much smoother developer Experience than Google's protobuf compiler.

## Example 

```python 
from typing import List, Tuple

import requests

from valhalla_proto.valhalla import (
    Api,
    DirectionsType,
    LatLng,
    OptionsAction,
    Options,
    CostingType,
    OptionsFormat,
    Location,
    CostingOptions,
    Costing,
)
from valhalla_proto._upstream import VALHALLA_COMMIT


def make_options(
    coords: List[Tuple[float, float]],
    action: OptionsAction = OptionsAction.route,
    costing_type: CostingType = CostingType.auto_,
    format: OptionsFormat = OptionsFormat.pbf,
) -> Options:
    costing = Costing(options=CostingOptions())

    locs: List[Location] = []
    for lon, lat in coords:
        locs.append(Location(ll=LatLng(lat=lat, lng=lon)))

    return Options(
        action=action,
        costing_type=costing_type,
        format=format,
        locations=locs,
        costings={costing_type: costing},
        directions_type=DirectionsType.maneuvers,
    )


def process_directions(api: Api):
    if not api.directions:
        print("Error: no directions found")
        return

    if not api.directions.routes:
        print("Error: no route found")
        return

    if not api.directions.routes[0].legs:
        print("Error: no leg found")
        return

    leg = api.directions.routes[0].legs[0]

    if leg.shape:
        print(f"Route shape: {leg.shape}")


def main():
    print(f"Using protobuf code generated from Valhalla commit {VALHALLA_COMMIT}")

    coords = [
        (11.119188, 52.862036),
        (11.118704, 52.862658),
    ]

    options = make_options(
        action=OptionsAction.route,
        coords=coords,
    )

    api_request = Api(options=options)

    resp = requests.post(
        # "https://valhalla1.openstreetmap.de/trace_attributes",
        "http://localhost:8002/route",
        data=api_request.SerializeToString(),
        headers={"Content-Type": "application/x-protobuf"},
    )
    api_response = Api.FromString(resp.content)

    process_directions(api_response)


if __name__ == "__main__":
    main()
```
