#!/bin/bash
set -eo pipefail

echo "Building with concurrency $(nproc)"

# configure
# NOTE: -Werror disabled in CI, as we currently have >4k warnings.
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=On -DCPACK_GENERATOR=DEB \
         -DENABLE_COMPILER_WARNINGS=On -DENABLE_WERROR=Off -DCMAKE_EXPORT_COMPILE_COMMANDS=On \
         -DENABLE_PYTHON_BINDINGS=On
cd ..

# build everything
make -C build all -j$(nproc)

# packaging
make -C install package

# extra lint
scripts/clang-tidy-only-diff.sh $(nproc)

# code coverage
make -C build coverage -j$(nproc)
/bin/bash <(curl -s https://codecov.io/bash) || echo "Codecov did not collect coverage reports"
