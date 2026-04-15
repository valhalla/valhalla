SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.PHONY: generate-ts version-ts attach-commit-ts attach-commit-py \
        build-py build-ts publish-py publish-ts publish-ts-tag \
        release-py release-ts release-ts-tag

generate-ts:
	cd typescript && npm install && npm run generate

version-ts:
	@scripts/set_version.sh ts

attach-commit-py:
	git rev-parse --short HEAD > proto/python/COMMIT

attach-commit-ts:
	echo "export const VALHALLA_COMMIT = \"$$(git -C .. rev-parse --short HEAD)\"" > typescript/src/_upstream.ts

build-py:
	pip wheel --no-deps python/ -w python/dist

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

release-py: build-py publish-py

release-ts: generate-ts version-ts attach-commit-ts build-ts publish-ts

release-ts-tag: generate-ts version-ts attach-commit-ts build-ts publish-ts-tag

