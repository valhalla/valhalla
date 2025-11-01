#!/usr/bin/env bash
# Runs clang-tidy only on changed files (or all files with --all flag)
#
# Usage:
#   ./clang-tidy-only-diff.sh [--all] [concurrency] [build_dir]
#
# Options:
#   --all         Run clang-tidy on all .cc files instead of only changed files
#   concurrency   Number of parallel jobs (default: nproc)
#   build_dir     Build directory containing compile_commands.json (default: build)
#
# Examples:
#   ./clang-tidy-only-diff.sh              # Run on changed files with default settings
#   ./clang-tidy-only-diff.sh --all        # Run on all files
#   ./clang-tidy-only-diff.sh 8 build      # Run on changed files with 8 jobs
#   ./clang-tidy-only-diff.sh --all 8 build # Run on all files with 8 jobs

set -o errexit -o pipefail -o nounset

# Parse arguments
all_files=false
positional_args=()

while [[ $# -gt 0 ]]; do
  case $1 in
    --all)
      all_files=true
      shift
      ;;
    *)
      positional_args+=("$1")
      shift
      ;;
  esac
done

readonly base=$(git merge-base refs/remotes/origin/master HEAD)
readonly concurrency=${positional_args[0]:-$(nproc)}
readonly build_dir=${positional_args[1]:-build}

source scripts/bash_utils.sh

# Backup compile_commands and filter out protobuf generated .pb.cc files
readonly tidy_dir=.tidytmp
mkdir -p $tidy_dir
cat $build_dir/compile_commands.json \
  | jq '[ .[] | select(.file | endswith(".pb.cc") | not) ]' \
  > $tidy_dir/compile_commands.json

modified_filepaths=()

if [ "$all_files" = true ]; then
  echo "Running clang-tidy on all files..."
  # Get all .cc files from src/ directory, excluding .pb.cc files
  while IFS='' read -r line
  do
    absolute_filepath=$(realpath "$line")
    echo "Found $absolute_filepath"
    modified_filepaths+=("$absolute_filepath")
  done < <(find src -name "*.cc" ! -name "*.pb.cc" -type f)
else
  echo "Running clang-tidy on changed files only..."
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
fi

if [ ${#modified_filepaths[@]} = 0 ]; then
  echo "No files to process"
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
