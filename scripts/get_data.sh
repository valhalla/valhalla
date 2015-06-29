#!/bin/bash

set -e

function dice() {
	#reference the originals
	rm -f tile.*.tif *.vrt
	gdalbuildvrt $1 *.tif
	#for each sq degree
	for x0 in $(seq -180 10 170); do
		for y0 in $(seq -90 10 80); do
			set +e
			let x1=x0+10
			let y1=y0+10
			set -e
			#cut out a tile
			gdal_translate -projwin $x0 $y1 $x1 $y0 $1 "tile.$x0.$y0.tif"
		done
	done
	#reference using smaller tiles
	rm -f $1
	gdalbuildvrt $1 tile.*.tif
}

#### SRTM ####
function srtm() {
	#a place to put the data
	mkdir -p srtm
	pushd srtm

	#grab the list of the files
	if [ ! -e srtmgl1.003.html ]; then
		curl "http://e4ftl01.cr.usgs.gov/SRTM/SRTMGL1.003/2000.02.11/" -s -o srtmgl1.003.html
	fi

	#grab the abbreviated list of hgt files
	grep -F '.hgt.zip<' srtmgl1.003.html | sed -e 's@.*href="@@g' -e 's/">.*//g' > srtmgl1.003.list

	#filter out files that are already on the disk (so we don't re-download them)
	echo -n > srtmgl1.003.urls
	for f in $(cat srtmgl1.003.list); do
		if [ ! -e $f ]; then
			echo "http://e4ftl01.cr.usgs.gov/SRTM/SRTMGL1.003/2000.02.11/${f}" >> srtmgl1.003.urls
		fi
	done

	#get them onto disk
	if [ $(wc -l srtmgl1.003.urls | awk '{print $1}') -gt 0 ]; then
		cat srtmgl1.003.urls | xargs -n1 -P$(nproc)  curl -s -O
	fi

	#unzip them
	ls *.zip | xargs -n1 -P$(nproc) unzip -n

	#reference them
	gdalbuildvrt srtm.vrt *.hgt

	#and we are done
	popd
}

#### GMTED2010 ####
function gmted() {
	#a place to put the data
	mkdir -p gmted
	pushd gmted

	#grab the data
	echo -n > gmted.urls
	for y in 90S 70S 50N 70N; do
		for x in 180W 150W 120W 090W 060W 030W 000E 030E 060E 090E 120E 150E; do
			dir=$(echo $x | sed -e "s/\([0-9]\+\)\(.*\)/\2\1/g")
			if [ ${y} == 90S ]; then
				path="/GMTED_viewer/data/Global_tiles_GMTED/300darcsec/mea/${dir}/${y}${x}_20101117_gmted_mea300.tif"
			else
				path="/GMTED_viewer/data/Global_tiles_GMTED/075darcsec/mea/${dir}/${y}${x}_20101117_gmted_mea075.tif"
			fi
			if [ ! -e $(basename ${path}) ]; then
				echo "http://topotools.cr.usgs.gov${path}" >> gmted.urls
			fi
		done
	done

	#get them onto disk
	if [ $(wc -l gmted.urls | awk '{print $1}') -gt 0 ]; then
		cat gmted.urls | xargs -n1 -P$(nproc)  curl -s -O
	fi

	#cut them to a smaller size (help querying be faster) and reference them
	dice gmted.vrt

	#and we are done
	popd
}

#### GEBCO/BODC ####
function gebco() {

	#select your data here:
	#http://www.gebco.net/data_and_products/gridded_bathymetry_data/

	#then download it here:
	#http://www.bodc.ac.uk/my_account/request_status/

	mkdir -p gebco
	pushd gebco

	#cut them to a smaller size (help querying be faster) and reference them
	dice gebco.vrt

	#and we are done
	popd
}

set +e
gv=$(gdal-config  --version)
if [ $gv != "2.0.0" ]; then
	./install_gdal.sh
fi
set -e

#srtm
gmted
gebco
