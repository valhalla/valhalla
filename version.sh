#!/bin/bash

#grabs the version string from the header file for use autotools
#which then throws it into a pc file so that pkg-config can use it

DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DIR=$(echo ${DIR} | awk -F'/' '{print $(NF)}')

if [ ! -f valhalla/$DIR/version.h ]; then
	echo "Could not find version information"
	exit 1
fi

major=$(grep -m1 -F VERSION_MAJOR valhalla/$DIR/version.h | sed -e "s/.*MAJOR \([0-9]\+\)/\1/g")
minor=$(grep -m1 -F VERSION_MINOR valhalla/$DIR/version.h | sed -e "s/.*MINOR \([0-9]\+\)/\1/g")
patch=$(grep -m1 -F VERSION_PATCH valhalla/$DIR/version.h | sed -e "s/.*PATCH \([0-9]\+\)/\1/g")

if [ -z $major ] || [ -z $minor ] || [ -z $patch ]; then
	echo "Malformed version information"
	exit 1
fi

printf "%s" "$major.$minor.$patch"
