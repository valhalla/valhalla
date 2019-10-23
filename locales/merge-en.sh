#!/bin/bash
set -e

ENGLISH="en-US.json"

for file in ./*.json
do
    if [[ -f $file ]]; then
        TARGET=$(basename ${file})
        TMP=$(mktemp)
        echo "Merging ${ENGLISH} into ${TARGET}"
        jq -s '.[0] * .[1]' ${ENGLISH} "${TARGET}" > "${TMP}"
        mv ${TMP} ${TARGET}
    fi
done
