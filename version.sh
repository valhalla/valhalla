#!/bin/sh

#grabs the version string from the header file for use autotools
#which then throws it into a pc file so that pkg-config can use it

if [ ! -f valhalla/valhalla.h.in ]; then
	echo "Could not find version information"
	exit 1
fi

major=$(grep -F VERSION_MAJOR valhalla/valhalla.h.in | sed -e "s/.*MAJOR \([0-9][0-9]*\)/\1/g" -e 1q)
minor=$(grep -F VERSION_MINOR valhalla/valhalla.h.in | sed -e "s/.*MINOR \([0-9][0-9]*\)/\1/g" -e 1q)
patch=$(grep -F VERSION_PATCH valhalla/valhalla.h.in | sed -e "s/.*PATCH \([0-9][0-9]*\)/\1/g" -e 1q)

if [ -z $major ] || [ -z $minor ] || [ -z $patch ]; then
	echo "Malformed version information"
	exit 1
fi

printf "%s" "$major.$minor.$patch"
