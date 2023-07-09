#!/usr/bin/env python3
from argparse import ArgumentParser
import json
import logging
from pathlib import Path
import sys

import xml.etree.ElementTree as ET


description = "Read a TomTom Incidents XML file and write a payload for the valhalla_incidents_service"

parser = ArgumentParser(description=description)
parser.add_argument("-i", "--input", help="Input XML file path", type=Path)
parser.add_argument("-o", "--output", help="Output JSON file path", type=Path)

# set up the logger basics
LOGGER = logging.getLogger(__name__)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)5s: %(message)s"))
LOGGER.addHandler(handler)


if __name__ == '__main__':
    args = parser.parse_args()
    args.input: Path
    args.output: Path

    if not (args.input and args.output):
        LOGGER.critical("Both --input & --output need to be specified")
        sys.exit(1)
    elif not (args.input.is_file()):
        LOGGER.critical(f"Input file {args.input} doesn't exist.")
        sys.exit(1)

    tree = ET.iterparse(args.input)

    openlrs = list()
    for event, node in tree:
        if "binary" in node.tag:
            openlrs.append(node.text)

    with args.output.open("w") as f:
        f.write(json.dumps(openlrs, indent=2))
