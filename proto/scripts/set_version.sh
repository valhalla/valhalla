#!/usr/bin/env bash
set -euo pipefail

# Derives a semver version from the latest git tag and commit distance,
# consistent with setuptools_scm's "no-guess-dev" scheme.
# On tag:        3.6.3
# After tag:     3.6.3-dev.5

DESC=$(git describe --tags --long --match '[0-9]*')
TAG=${DESC%-*-*}
DISTANCE=${DESC#"$TAG"-}
DISTANCE=${DISTANCE%-*}

if [ "$DISTANCE" = "0" ]; then
  VERSION="$TAG"
else
  VERSION="$TAG-dev.$DISTANCE"
fi

case "${1:-}" in
  ts)
    cd "$(dirname "$0")/../typescript"
    npm version --no-git-tag-version --allow-same-version "$VERSION"
    ;;
  *)
    echo "$VERSION"
    ;;
esac
