#!/bin/bash
set -e

if [ -z $5 ]; then
	exit 1
fi

#make a temporary file name that isnt currently used
function tmpfilename() {
	while true; do
		local name="${1}${RANDOM}${RANDOM}${RANDOM}${RANDOM}${RANDOM}${2}"
		if [ ! -e "$name" ]; then
			echo "$name"
			break
		fi
	done
}

#always delete this tmp no matter what
bathymetry=$(tmpfilename "tmp." ".flat_bathy.tif")
intermediate=$(tmpfilename "tmp." ".itermediate.tif")
trap "rm -f $bathymetry $intermediate" EXIT

#TODO: checkout interpolation algorithms, specifically: -a linear nodata=-32768
#TODO: checkout GDAL_NUM_THREADS
#TODO: do we need overlap or bleed area added?

#start out with the bathymetry as only a filter where gmted and srtm have no data (ie -32768)
GDAL_SKIP=JPEG gdalwarp -q -multi -wo NUM_THREADS=ALL_CPUS --config GDAL_CACHEMAX 1024 -wm 1024 -r lanczos gebco/gebco.vrt gmted/gmted.vrt srtm/srtm.vrt -te $1 $2 $3 $4 $bathymetry

#gmted and srtm purposely flatten water (to 0 for contour lines) but neither have global coverage so we have bathy in a lot of places
#so what we do is take the flattened stuff and replace it with bathy so we are at least consistent
#alternatively we could ship something else that is global but has flattened water features as well
GDAL_SKIP=JPEG gdalwarp -q -multi -wo NUM_THREADS=ALL_CPUS --config GDAL_CACHEMAX 1024 -wm 1024 -r lanczos -srcnodata 0 -dstnodata -32768 gebco/gebco.vrt $bathymetry -te $1 $2 $3 $4 $intermediate

#finally store it in srtmgl1 format because its a completely raw file format with regular filenaming convention
#this ends up being way way faster than anything more flexible
GDAL_SKIP=JPEG gdal_translate -q -of SRTMHGT $intermediate $5
rm -f $5.aux.xml
gzip $5
