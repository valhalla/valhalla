#!/bin/bash
export LD_LIBRARY_PATH=/usr/lib:/usr/local/lib/
set -e

#### SRTM ####
function srtm() {
	#a place to put the data
	mkdir -p srtm
	pushd srtm

	#grab the list of the files
	if [ ! -e srtmgl1.003.html ]; then
		echo "$(date): fetching srtm file list"
		curl "http://e4ftl01.cr.usgs.gov/SRTM/SRTMGL1.003/2000.02.11/" -s -o srtmgl1.003.html
	fi

	#grab the abbreviated list of hgt files
	grep -F '.hgt.zip<' srtmgl1.003.html | sed -e 's@.*href="@@g' -e 's/">.*//g' > srtmgl1.003.list

	#filter out files that are already on the disk (so we don't re-download them)
	echo -n > srtmgl1.003.urls
	for f in $(cat srtmgl1.003.list); do
		#no zip file
		if [ ! -e $f ]; then
			#no hgt file
			hgt=$(echo $f | sed -e 's/\..*/.hgt/g')
			if [ ! -e $hgt ]; then
				echo "http://e4ftl01.cr.usgs.gov/SRTM/SRTMGL1.003/2000.02.11/${f}" >> srtmgl1.003.urls
			fi
		fi
	done

	#get them onto disk
	if [ $(wc -l srtmgl1.003.urls | awk '{print $1}') -gt 0 ]; then
		#download the zip files
		echo "$(date): downloading srtm files"
		cat srtmgl1.003.urls | xargs -n1 -P$(nproc)  curl -s -O

		#unzip them
		echo "$(date): decompressing srtm files"
		ls *.zip | xargs -n1 -P$(nproc) unzip -n
	fi

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
	for y in 90S 70S 50S 30S 10S 10N 30N 50N 70N; do
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
		echo "$(date): downloading gmted files"
		cat gmted.urls | xargs -n1 -P$(nproc)  curl -s -O
	fi

	#TODO: cut these a little smaller than they are

	#cut them to a smaller size (help querying be faster) and reference them
	gdalbuildvrt gmted.vrt *.tif

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
	gdalbuildvrt gebco.vrt *.tif

	#and we are done
	popd
}


#check for gdal
set +e
gv=$(gdal-config  --version)
if [ $? -ne 0 ] || [ $gv != "2.0.0" ]; then
	echo "$(date): installing gdal"
	./install_gdal.sh
fi
set -e

#get each data set
srtm
gmted
gebco

#prepare directories for all these tiles
echo "$(date): preparing tile directories"

for d in $(./args.py | sed -e "s/.* //g" -e 's@/[^/]\+$@@g' | sort | uniq); do
	mkdir -p $d
done

#cut tiles that are composites of all 3 for world wide coverage
echo "$(date): cutting tiles"
set +e
which parallel
if [ $? -ne 0 ]; then
	sudo apt-get install parallel
fi
set -e
./args.py | parallel --joblog cut_tiles.log -C ' ' -P $(nproc) "./composite.sh {} 2>err.log 1>comp.log"
