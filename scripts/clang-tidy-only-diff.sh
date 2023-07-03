#!/usr/bin/env bash
# Runs clang-tidy only on changed files

set -o errexit -o pipefail -o nounset

readonly base=$(git merge-base refs/remotes/origin/master HEAD)
readonly build_dir=build

readonly CLANG_TIDY_VERSION=7.0.0

source scripts/bash_utils.sh
setup_mason

./mason/mason install clang-tidy $CLANG_TIDY_VERSION
./mason/mason link clang-tidy $CLANG_TIDY_VERSION
readonly CLANG_TIDY=$(pwd)/mason_packages/.link/bin/clang-tidy

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

readonly num_jobs="${1:-$(nproc)}"
# -m specifies that `parallel` should distribute the arguments evenly across the executing jobs.
# -p Tells clang-tidy where to find the `compile_commands.json`.
# `{}` specifies where `parallel` adds the command-line arguments.
# `:::` separates the command `parallel` should execute from the arguments it should pass to the commands.
parallel \
  -m \
  -j $num_jobs \
  --halt-on-error now,fail=1 \
  ${CLANG_TIDY} \
  -p $tidy_dir \
  -header-filter "^$(pwd)/valhalla/[^/]+$" \
  ${FIX_ERRORS} \
  -format-style=file \
  {} ::: "${modified_filepaths[@]}"

if ! git diff --exit-code; then
  echo "Tidy introduced changes"
  exit 1
fi
