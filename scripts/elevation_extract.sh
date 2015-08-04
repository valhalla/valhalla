#!/bin/bash

function usage() {
	echo "Usage: $0 min_x max_x min_y max_y"
	exit 1
}

#validate ranges
if [ -z "$4" ]; then
	usage
fi
min_x=$(($1+180))
max_x=$(($2+180))
min_y=$(($3+90))
max_y=$(($4+90))
if [ $min_x -gt $max_x ] || [ $min_x -lt 0 ] || [ $max_x -gt 360 ]; then
	usage
fi
if [ $min_y -gt $max_y ] || [ $min_y -lt 0 ] || [ $max_y -gt 180 ]; then
	usage
fi

#pull down all the vrts, even the ones that point to files you dont have
aws --region us-east-1 s3 sync s3://mapzen.valhalla/elevation ./elevation/ --exclude "*" --include "*.vrt"

#pull down the bounding box of tiles you want
for d in $(seq $min_x 1 $max_x); do
	for f in $(seq $min_y 1 $max_y); do
		aws --region us-east-1 s3 sync s3://mapzen.valhalla/elevation/tiles/${d}/ ./elevation/tiles/${d}/ --exclude "*" --include "${f}.tif"
	done
done
