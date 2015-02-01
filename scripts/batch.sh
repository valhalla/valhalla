#!/bin/bash

#./pathtest -o 47.118589,9.543217 -d 47.166110,9.511975 -t auto conf/valhalla.json

which parallel &> /dev/null
if [ $? != 0 ]; then
	echo "parallel is required please install it"
	echo "sudo apt-get install parallel"
	exit 1
fi

function usage() {
	echo "Usage: $0 pathTestArgsFile [diffDir] [concurrency] [outDir]"
        echo "Example: $0 paRoutes"
	echo "Example: $0 paRoutes 2015_06_13_14_45_03"
	echo "Example: $0 paRoutes 2015_06_13_14_45_03 8"
	echo "Example: $0 paRoutes 2015_06_13_14_45_03 8 mySpecialDir"
        echo
	echo "Note: each line in your pathTestArgsFile should look like this:"
	echo
	echo "-o 47.118589,9.543217 -d 47.166110,9.511975 -t auto conf/valhalla.json"
	exit 1
}

#get the input file
if [ -z "${1}" ]; then
	usage;
elif [ ! -f "${1}" ]; then
	usage;
else
	INPUT="${1}";
fi

#diffing or not, default to not
DIFF=
if [ -z "${2}" ]; then
	DIFF="${2}";
fi

#how many threads do you want, default to max
CONCURRENCY=$(nproc)
if [ "${3}" ]; then
	CONCURRENCY="${3}";
fi

#where do you want the output, default to current time
OUTDIR=$(date +%Y_%m_%d_%H_%S)
if [ "${4}" ]; then
	OUTDIR="${4}";
fi
mkdir -p "${OUTDIR}"

#run all of the paths, make sure to cut off the timestamps
#from the log messages otherwise every line will be a diff 
cat "${INPUT}" | parallel -k -C ' ' -P "${CONCURRENCY}" "pathtest {} 2>&1 | sed -e 's/^[^\[]*\[//' &> ${OUTDIR}/{#}.txt"

#if we need to run a diff
if [ -d "${DIFF}" ]; then
	diff "${DIFF}" "${OUTDIR}"
fi
