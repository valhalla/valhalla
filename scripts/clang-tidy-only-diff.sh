#!/usr/bin/env bash
# Runs clang-tidy only on changed files

set -o errexit -o pipefail -o nounset

readonly base=$(git merge-base refs/remotes/origin/master HEAD)
readonly build_dir=build

# Backup compile_commands and filter out protobuf generated .pb.cc files
readonly tidy_dir=.tidytmp
mkdir --parent $tidy_dir
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
    modified_filepaths+=("$absolute_filepath")
  fi

# `git diff-tree` outputs all the files that differ between the different commits.
# By specifying `--diff-filter=d`, it doesn't report deleted files.
done < <(git diff-tree --no-commit-id --diff-filter=d --name-only -r "$base" HEAD)

echo "${modified_filepaths[*]}"

# -m specifies that `parallel` should distribute the arguments evenly across the executing jobs.
# -p Tells clang-tidy where to find the `compile_commands.json`.
# `{}` specifies where `parallel` adds the command-line arguments.
# `:::` separates the command `parallel` should execute from the arguments it should pass to the commands.
# `| tee` specifies that we would like the output of clang-tidy to go to `stdout` and also to capture it in
# `$build_dir/clang-tidy-output` for later processing.
parallel \
  -m clang-tidy \
  -p $tidy_dir \
  -header-filter "^$(pwd)/valhalla/[^/]+$" \
  -fix \
  -format-style=file \
  {} ::: "${modified_filepaths[@]}" #| tee "$build_dir/clang-tidy-output"

if ! git diff --exit-code --quiet; then
  echo "Tidy introduced changes"
  exit 1
fi
