#!/bin/bash

#grabs the version string from the header file for use autotools
#which then throws it into a pc file so that pkg-config can use it
DIR=$(pkg-config --cflags libvalhalla_midgard | tr ' ' '\n' | grep -F -- "-I" | sed -e "s/^-I//g")

if [ ! -f ${DIR}/valhalla/version.h ]; then
	echo "Could not find version information"
	exit 1
fi

major=$(grep -m1 -F VERSION_MAJOR ${DIR}/valhalla/version.h | sed -e "s/.*MAJOR \([0-9]\+\)/\1/g")
minor=$(grep -m1 -F VERSION_MINOR ${DIR}/valhalla/version.h | sed -e "s/.*MINOR \([0-9]\+\)/\1/g")
patch=$(grep -m1 -F VERSION_PATCH ${DIR}/valhalla/version.h | sed -e "s/.*PATCH \([0-9]\+\)/\1/g")

if [ -z $major ] || [ -z $minor ] || [ -z $patch ]; then
	echo "Malformed version information"
	exit 1
fi

printf "%s" "$major.$minor.$patch"
