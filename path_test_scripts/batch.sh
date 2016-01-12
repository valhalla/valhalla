#!/bin/bash

which parallel &> /dev/null
if [ $? != 0 ]; then
	echo "parallel is required please install it"
	echo "sudo apt-get install parallel"
	exit 1
fi

function usage() {
        echo "Usage: $0 path_test_request_file [diffDir] [concurrency] [outDir]"
        echo "Example: $0 demo_routes.txt"
        echo "Example: $0 demo_routes.txt 20160104_160939_demo_routes"
        echo "Example: $0 demo_routes.txt 20160104_160939_demo_routes 8"
        echo "Example: $0 demo_routes.txt 20160104_160939_demo_routes 8 my_special_dir"
        exit 1
}

#get the input file
if [ -z "requests/${1}" ]; then
	usage
elif [ ! -f "requests/${1}" ]; then
	usage
else
	INPUT="${1}"
fi

#diffing or not, default to not
DIFF="${2}"

#how many threads do you want, default to max
CONCURRENCY=$(nproc)
if [ "${3}" ]; then
	CONCURRENCY="${3}"
fi

#where do you want the output, default to current time
OUTDIR=$(date +%Y%m%d_%H%M%S)_$(basename "${INPUT%.*}")
if [ "${4}" ]; then
	OUTDIR="${4}"
fi
RESULTS_OUTDIR="results/${OUTDIR}"
mkdir -p "${RESULTS_OUTDIR}"

#turn the nice input into something parallel can parse for args
TMP="$(mktemp)"
cp -rp "requests/${INPUT}" "${TMP}"
for arg in $(path_test --help | grep -o '\-[a-z\-]\+' | sort | uniq); do
	sed -i -e "s/[ ]\?${arg}[ ]\+/|${arg}|/g" "${TMP}"
done
sed -i -e "s/\([^\\]\)'|/\1|/g" -e "s/|'/|/g" "${TMP}"

#run all of the paths, make sure to cut off the timestamps
#from the log messages otherwise every line will be a diff
#TODO: add leading zeros to output files so they sort nicely
echo -e "\x1b[32;1mWriting routes from ${INPUT} with a concurrency of ${CONCURRENCY} into ${OUTDIR}\x1b[0m"
cat "${TMP}" | parallel --progress -k -C '\|' -P "${CONCURRENCY}" "path_test {} 2>&1 | tee -a ${RESULTS_OUTDIR}/{#}.tmp | grep -F NARRATIVE | sed -e 's/^[^\[]*\[NARRATIVE\] //' &> ${RESULTS_OUTDIR}/{#}.txt; grep -F STATISTICS ${RESULTS_OUTDIR}/{#}.tmp | sed -e 's/^[^\[]*\[STATISTICS\] //' &>> ${RESULTS_OUTDIR}/{#}_statistics.csv; rm -f ${RESULTS_OUTDIR}/{#}.tmp"
rm -f "${TMP}"
echo "orgLat, orgLng, destLat, destLng, result, #Passes, runtime, trip time, length, arcDistance, #Manuevers" > ${RESULTS_OUTDIR}/statistics.csv
cat `ls -1v ${RESULTS_OUTDIR}/*_statistics.csv` >> ${RESULTS_OUTDIR}/statistics.csv
rm -f ${RESULTS_OUTDIR}/*_statistics.csv

echo ${OUTDIR} > outdir.txt

#if we need to run a diff
cd results
if [ -d "${DIFF}" ]; then
	if [[ "${DIFF}" == *$(basename ${INPUT%.*})* ]]; then
		echo -e "\x1b[32;1mDiffing the output of ${DIFF} with ${OUTDIR} to ${DIFF}_${OUTDIR}_diff\x1b[0m"
		mkdir -p "${DIFF%/}_${OUTDIR}_diff"
		find ${DIFF}/*.txt -printf "%f\n" | parallel --progress -P "${CONCURRENCY}" "diff ${DIFF}/{} ${OUTDIR}/{} > ${DIFF%/}_${OUTDIR}_diff/{}"
	else
		echo -e "\x1b[31;1mFailed to diff ${DIFF} with ${OUTDIR} as it looks like they were generated from different input\x1b[0m"
	fi
elif [ "${DIFF}" != "" ]; then
	echo -e "\x1b[31;1mFailed to diff using non-existant ${DIFF} directory\x1b[0m"
fi
cd ..

