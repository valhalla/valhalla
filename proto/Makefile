SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.PHONY: generate-py generate-ts version version-for-% \
        attach-commit-py attach-commit-ts \
        build-py build-ts publish-py publish-ts publish-ts-tag \
        release-py release-ts release-ts-tag release-py-tag

VALHALLA_PROTO := descriptors
PYTHON_OUT := python/src/valhalla_proto

generate-ts:
	cd typescript && npm install && npm run generate

version-for-%:
	python scripts/set_version.py $*

version-ts:
	python scripts/set_version.py ts

attach-commit-py:
	echo "VALHALLA_COMMIT = \"$$(git -C .. rev-parse --short HEAD)\"" > $(PYTHON_OUT)/_upstream.py

attach-commit-ts:
	echo "export const VALHALLA_COMMIT = \"$$(git -C .. rev-parse --short HEAD)\"" > typescript/src/_upstream.ts

build-py:
	cd python && rm -rf dist && python -m build

build-ts:
	cd typescript && npm run build && npm pack

publish-py:
	cd python && twine upload -u __token__ -p $(PYPI_TOKEN) dist/* -r testpypi

publish-ts:
	cd typescript && npm pack
	cd typescript && npm publish --access public --tag dev

publish-ts-tag:
	cd typescript && npm pack
	cd typescript && npm publish --access public

release-py: generate-py attach-commit-py build-py publish-py

release-ts: generate-ts version-for-ts attach-commit-ts build-ts publish-ts

release-ts-tag: generate-ts version-for-ts attach-commit-ts build-ts publish-ts-tag

