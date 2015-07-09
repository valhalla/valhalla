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
flat_bathy=$(tmpfilename "tmp." ".flat_bathy.tif")
trap "rm -f -- '$flat_bathy'" EXIT

#TODO: checkout interpolation algorithms, specifically: -a linear nodata=-32768
#TODO: checkout GDAL_NUM_THREADS
#TODO: do we need overlap or bleed area added?

#TODO:
#we want a consistent resolution of the output tile so we need to do some math..
#xy_size=$(gdalinfo srtm/srtm.vrt | grep -F "Pixel Size" | sed -e "s/.* //g" -e "s/[()]//g")
#x_size=$(echo $xy_size | sed -e "s/,.*//g")
#y_size=$(echo $xy_size | sed -e "s/.*,//g")
#width=$(python -c "import math; print math.abs(1.0/$x_size)")
#height=$(python -c "import math; print math.abs(1.0/$y_size)")

#start out with the bathymetry as only a filter where gmted and srtm have no data (ie -32768)
gdalwarp -q -multi -wo NUM_THREADS=ALL_CPUS --config GDAL_CACHEMAX 1024 -wm 1024 -r lanczos gebco/gebco.vrt gmted/gmted.vrt srtm/srtm.vrt -te $1 $2 $3 $4 $flat_bathy

#gmted and srtm purposely flatten water (to 0 for contour lines) but neither have global coverage so we have bathy in a lot of places
#so what we do is take the flattened stuff and replace it with bathy so we are at least consistent
#alternatively we could ship something else that is global but has flattened water features as well
gdalwarp -q -co "COMPRESS=DEFLATE" -multi -wo NUM_THREADS=ALL_CPUS --config GDAL_CACHEMAX 1024 -wm 1024 -r lanczos -srcnodata 0 -dstnodata -32768 gebco/gebco.vrt $flat_bathy -te $1 $2 $3 $4 $5
