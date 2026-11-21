#!/bin/bash
# fetch_release.sh <url> <sha256>: download, verify and extract a source tarball into the current directory
set -euo pipefail

url=$1
sha=$2
file=$(basename "$url")

curl -fsSL --retry 3 -o "$file" "$url"
echo "${sha}  ${file}" | sha256sum -c -
tar -xf "$file"
rm "$file"
