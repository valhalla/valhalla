#!/bin/bash

which parallel &> /dev/null
if [ $? != 0 ]; then
	echo "parallel is required please install it"
	echo "sudo apt-get install parallel"
	exit 1
fi

function usage() {
	echo "Usage: $0 pathTestArgsFile [diffDir] [concurrency] [outDir]"
        echo "Example: $0 li.txt"
	echo "Example: $0 li.txt 2015_06_13_14_45_03_li"
	echo "Example: $0 li.txt 2015_06_13_14_45_03_li 8"
	echo "Example: $0 li.txt 2015_06_13_14_45_03_li 8 mySpecialDir"
        echo
	echo "Note: each line in your pathTestArgsFile should look like this:"
	echo
	echo "-o 47.118589,9.543217 -d 47.166110,9.511975 -t auto conf/valhalla.json"
	exit 1
}

#get the input file
if [ -z "${1}" ]; then
	usage
elif [ ! -f "${1}" ]; then
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
mkdir -p "${OUTDIR}"

#turn the nice input into something parallel can parse for args
TMP="$(mktemp)"
cp -rp "${INPUT}" "${TMP}"
for arg in $(pathtest --help | grep -o '\-[a-z\-]\+' | sort | uniq); do
	sed -i -e "s/[ ]\?${arg}[ ]\+/|${arg}|/g" "${TMP}"
done
sed -i -e "s/\([^\\]\)'|/\1|/g" -e "s/|'/|/g" "${TMP}"

#run all of the paths, make sure to cut off the timestamps
#from the log messages otherwise every line will be a diff
#TODO: add leading zeros to output files so they sort nicely
echo -e "\x1b[32;1mWriting routes from ${INPUT} with a concurrency of ${CONCURRENCY} into ${OUTDIR}\x1b[0m"
cat "${TMP}" | parallel --progress -k -C '\|' -P "${CONCURRENCY}" "pathtest {} 2>&1 | tee -a ${OUTDIR}/{#}.tmp | grep -F NARRATIVE | sed -e 's/^[^\[]*\[NARRATIVE\] //' &> ${OUTDIR}/{#}.txt; grep -F STATISTICS ${OUTDIR}/{#}.tmp | sed -e 's/^[^\[]*\[STATISTICS\] //' &>> ${OUTDIR}/{#}_statistics.csv; rm -f ${OUTDIR}/{#}.tmp"
rm -f "${TMP}"
echo "orgLat, orgLng, destLat, destLng, result, #Passes, runtime, trip time, length, arcDistance, #Manuevers" > ${OUTDIR}/statistics.csv
cat `ls -1v ${OUTDIR}/*_statistics.csv` >> ${OUTDIR}/statistics.csv
rm -f ${OUTDIR}/*_statistics.csv

echo ${OUTDIR} > outdir.txt

#if we need to run a diff
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
