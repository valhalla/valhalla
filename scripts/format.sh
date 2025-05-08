#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# Runs the Clang Formatter in parallel on the code base.
# Return codes:
#  - 1 there are files to be formatted
#  - 0 everything looks fine


readonly CLANG_FORMAT_VERSION=11.0.1

if [[ $(uname -i) == 'aarch64' ]]; then
  echo 'Formatting is disabled on arm for the time being'
  exit
fi
source scripts/bash_utils.sh

# Python setup
py=$(setup_python)
if [[ $(python3 -m pip list | grep -c "black\|flake8") -ne 2 ]]; then
  if [[ $(python3 -c 'import sys; print(int(sys.base_prefix != sys.prefix or hasattr(sys, "real_prefix")))') -eq 1 ]]; then
    ${py} -m pip install black==24.10.0 flake8==7.1.1 clang-format=="$CLANG_FORMAT_VERSION"
  else
    sudo PIP_BREAK_SYSTEM_PACKAGES=1 ${py} -m pip install black==24.10.0 flake8==7.1.1
  fi
fi
python_sources=$(LANG=C find scripts src/bindings/python -type f -exec file {} \; | grep -F "Python script" | sed 's/:.*//')

# Python formatter
${py} -m black --line-length=105 --skip-string-normalization ${python_sources}

# Python linter
${py} -m flake8 --max-line-length=105 --extend-ignore=E501,E731,E203 --extend-exclude=src/bindings/python/__init__.py ${python_sources}

# clang-format
readonly CLANG_FORMAT="$(${py} -c 'import sys; print(sys.prefix)')"/bin/clang-format

echo "Using clang-format $CLANG_FORMAT_VERSION from ${CLANG_FORMAT}"

find src valhalla test -type f -name '*.h' -o -name '*.cc' \
  | xargs -I{} -P ${NPROC} ${CLANG_FORMAT} -i -style=file {}
