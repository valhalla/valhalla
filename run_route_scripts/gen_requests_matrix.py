#!/usr/bin/env python3

import argparse
import json
from collections import namedtuple
from pathlib import Path
from random import uniform
import sys

Bbox = namedtuple("Bbox", "min_x min_y max_x max_y")

# parse some program arguments
parser = argparse.ArgumentParser(description="Generates matrix request payloads from random coordinates within a bbox and writes to a single file.")
parser.add_argument("-o", "--output-file", type=Path, help='the output file path', required=True)
parser.add_argument("-b", "--bbox", type=str, help="bbox for random coords in format minx,miny,maxx, maxy", required=True)
parser.add_argument("-l", "--locations", type=int, help="amount of sources/targets, default 20", default=20)
parser.add_argument("-r", "--runs", type=int, help="how many requests to generate", default=100)
parser.add_argument("-c", "--costing", type=str, help="the costing model, default auto", default="auto")
args = parser.parse_args()


if __name__ == "__main__":
    try:
        bbox = Bbox(*[float(x) for x in args.bbox.split(",")])
    except ValueError:
        print(f"BBOX {args.bbox} is not a comma-separated string of coordinates.", file=sys.stderr)
        sys.exit(1)

    req = {
        "costing": args.costing,
        "verbose": False
    }

    if args.output_file.exists():
        args.output_file.unlink()

    for run in range(args.runs):
        locations = list()
        for loc in range(args.locations):
            x = round(uniform(bbox.min_x, bbox.max_x), 6)
            y = round(uniform(bbox.min_y, bbox.max_y), 6)
            locations.append({"lon": x, "lat": y})

        req["sources"] = locations
        req["targets"] = locations

        with args.output_file.open('a') as f:
            json.dump(req, f)
            f.write("\n")
