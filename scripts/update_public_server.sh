# updates Valhalla on the FOSSGIS servers
# accepts one argument, either "builder" or "service"

set -e

server=$1
src_dir="/src/valhalla"

git config --global --add safe.directory /src/valhalla

for s in $(ls /src/valhalla/third_party); do
    git config --global --add safe.directory /src/valhalla/third_party/$s
done

git -C "${src_dir}" checkout master
git -C "${src_dir}" pull
git -C "${src_dir}" submodule update --init --recursive

# remove the build folder first
rm -r "${src_dir}"/build

if [[ $server == "builder" ]]; then
    cmake -S "${src_dir}" -B "${src_dir}/build" \
        -DENABLE_TOOLS=OFF \
        -DENABLE_SERVICES=OFF \
        -DENABLE_HTTP=OFF \
        -DENABLE_PYTHON_BINDINGS=OFF \
        -DENABLE_TESTS=OFF \
        -DENABLE_SINGLE_FILES_WERROR=OFF \
        -DENABLE_GDAL=OFF

    sudo make -C "${src_dir}/build" -j$(nproc) install
    # config is updated by the build script on the server
else
    cmake -S "${src_dir}" -B "${src_dir}/build" \
    -DENABLE_DATA_TOOLS=OFF \
    -DENABLE_SERVICES=ON \
    -DENABLE_HTTP=ON \
    -DENABLE_PYTHON_BINDINGS=OFF \
    -DENABLE_TESTS=OFF \
    -DENABLE_SINGLE_FILES_WERROR=OFF

    sudo make -C "${src_dir}/build" -j$(nproc) install
    # Update the configs
    /opt/valhalla/runner_build_config.sh 8000 && /opt/valhalla/runner_build_config.sh 8001
fi
