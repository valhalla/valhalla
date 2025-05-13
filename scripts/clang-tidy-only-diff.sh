#!/usr/bin/env bash
# Runs clang-tidy only on changed files

set -o errexit -o pipefail -o nounset

readonly base=$(git merge-base refs/remotes/origin/master HEAD)
readonly concurrency=${1:-$(nproc)}
readonly build_dir=${2:-build}

source scripts/bash_utils.sh

# Backup compile_commands and filter out protobuf generated .pb.cc files
readonly tidy_dir=.tidytmp
mkdir -p $tidy_dir
cat $build_dir/compile_commands.json \
  | jq '[ .[] | select(.file | endswith(".pb.cc") | not) ]' \
  > $tidy_dir/compile_commands.json

modified_filepaths=()

# To properly handle file names with spaces, we have to do some bash magic.
# We set the Internal Field Separator to nothing and read line by line.
while IFS='' read -r line
do
  absolute_filepath=$(realpath "$line")
  if [[ ${absolute_filepath: -2} == "cc" || ${absolute_filepath: -2} == "." ]]; then
    echo "Detected changes in $absolute_filepath"
    modified_filepaths+=("$absolute_filepath")
  fi

done < <(git diff-tree --no-commit-id --diff-filter=d --name-only -r "$base" HEAD)

if [ ${#modified_filepaths[@]} = 0 ]; then
  echo "No paths modified"
  exit 0
fi

readonly FIX_ERRORS="-fix -fix-errors"
py=$(setup_python)
install_py_packages $py
CLANG_TIDY_CMD="${py} -c \"from clang_tidy import clang_tidy; clang_tidy()\""

# -m specifies that `parallel` should distribute the arguments evenly across the executing jobs.
# -p Tells clang-tidy where to find the `compile_commands.json`.
# `{}` specifies where `parallel` adds the command-line arguments.
# `:::` separates the command `parallel` should execute from the arguments it should pass to the commands.
parallel \
  -m \
  -j ${concurrency} \
  --halt-on-error now,fail=1 \
  "${CLANG_TIDY_CMD}" \
  -p $tidy_dir \
  -header-filter "^$(pwd)/valhalla/[^/]+$" \
  ${FIX_ERRORS} \
  -format-style=file \
  {} ::: "${modified_filepaths[@]}"

if ! git diff --exit-code; then
  echo "Tidy introduced changes"
  exit 1
fi
