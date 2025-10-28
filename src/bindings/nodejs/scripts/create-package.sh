#!/bin/bash

# Step 1: Set version

# Parse version from valhalla/valhalla.h (e.g., 3.5.1)
MAJOR=$(grep -oP '#define VALHALLA_VERSION_MAJOR \K\d+' valhalla/valhalla.h)
MINOR=$(grep -oP '#define VALHALLA_VERSION_MINOR \K\d+' valhalla/valhalla.h)
PATCH=$(grep -oP '#define VALHALLA_VERSION_PATCH \K\d+' valhalla/valhalla.h)
BASE_VERSION="${MAJOR}.${MINOR}.${PATCH}"

# Determine version based on event type
if [[ "${{ startsWith(github.ref, 'refs/tags/') }}" == "true" ]]; then
# For tag releases, use the tag version (strip 'v' prefix if present)
TAG_VERSION="${GITHUB_REF#refs/tags/}"
TAG_VERSION="${TAG_VERSION#v}"
VERSION="${TAG_VERSION}"
echo "[INFO] Building RELEASE version: ${VERSION}"
else
# For weekly/scheduled builds, append date and git hash
GIT_HASH=$(git rev-parse --short HEAD)
VERSION="${BASE_VERSION}-$(date +%Y%m%d).${GIT_HASH}"
echo "[INFO] Building WEEKLY version: ${VERSION}"
fi

# Update package.json version (package name stays as @valhallajs/valhallajs)
cd src/bindings/nodejs
npm version "${VERSION}" --no-git-tag-version --allow-same-version

echo "[INFO] Updated package.json:"
cat package.json | grep -E '"name"|"version"'


# Step 2: Create NPM package structure
mkdir -p valhalla-npm-package
          
# Copy package files
cp src/bindings/nodejs/index.js valhalla-npm-package/
cp src/bindings/nodejs/index.mjs valhalla-npm-package/
cp src/bindings/nodejs/index.d.ts valhalla-npm-package/
cp src/bindings/nodejs/package.json valhalla-npm-package/
cp src/bindings/nodejs/README.md valhalla-npm-package/
cp src/bindings/nodejs/.npmignore valhalla-npm-package/
cp -r src/bindings/nodejs/lib valhalla-npm-package/
cp -r src/bindings/nodejs/bin valhalla-npm-package/
cp scripts/valhalla_build_config valhalla-npm-package/

# Extract and organize bindings by platform
pushd artifacts

# Process all platforms in a loop
for platform in "linux:x64" "linux:arm64" "darwin:arm64"; do
os="${platform%%:*}"
arch="${platform##*:}"
platform_name="${os/darwin/macos}"

echo "[INFO] Processing ${platform_name} ${arch}..."
mkdir -p ../valhalla-npm-package/${os}/${arch}
mv valhalla-nodejs-${platform_name}-${arch}/valhalla_node.node ../valhalla-npm-package/${os}/${arch}/
mv valhalla-nodejs-${platform_name}-${arch}/valhalla_* ../valhalla-npm-package/${os}/${arch}/ 2>/dev/null || true
mv valhalla-nodejs-${platform_name}-${arch}/lib ../valhalla-npm-package/${os}/${arch}/
done

popd # artifacts

echo "[INFO] Package structure:"
tree -L 5 valhalla-npm-package/ 

echo "[INFO] Creating NPM package..."
pushd valhalla-npm-package
npm pack
popd # valhalla-npm-package