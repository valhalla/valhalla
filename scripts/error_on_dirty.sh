#!/usr/bin/env bash

set -o pipefail
set -o nounset

MSG="The following files have been modified:"
# ignore the package-lock, which npm install modifies if it was created with a different version of node
dirty=$(git ls-files --modified | grep -v package-lock.json)

if [[ $dirty ]]; then
    echo $MSG
    echo $dirty
    exit 1
else
    exit 0
fi
