#!/bin/bash

# configure
# NOTE: -Werror disabled in CI, as we currently have >4k warnings.
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=On -DCPACK_GENERATOR=DEB \
         -DENABLE_COMPILER_WARNINGS=On -DENABLE_WERROR=Off -DCMAKE_EXPORT_COMPILE_COMMANDS=On \
         -DENABLE_PYTHON_BINDINGS=On

# build everything
make all -j$(nproc)

# extra lint
scripts/clang-tidy-only-diff.sh 4

# packaging
make install package

# code coverage
make -j2 coverage
/bin/bash <(curl -s https://codecov.io/bash) || echo "Codecov did not collect coverage reports"

