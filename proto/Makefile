SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.PHONY: generate-python generate-ts version version-for-% \
        attach-commit-python attach-commit-ts \
        build-python build-ts publish-python publish-ts publish-ts-tag \
        release-python release-ts release-ts-tag release-python-tag

VALHALLA_PROTO := descriptors
PYTHON_OUT := python/src/valhalla_proto

generate-python:
	rm -rf $(PYTHON_OUT)
	mkdir $(PYTHON_OUT)
	protoc \
		-I $(VALHALLA_PROTO) \
		--python_betterproto2_out=$(PYTHON_OUT) \
		descriptors/*.proto

generate-ts:
	cd ts && npm install && npm run generate

version-for-%:
	python scripts/set_version.py $*

version:
	python scripts/set_version.py

attach-commit-python:
	echo "VALHALLA_COMMIT = \"$$(git -C .. rev-parse --short HEAD)\"" > $(PYTHON_OUT)/_upstream.py

attach-commit-ts:
	echo "export const VALHALLA_COMMIT = \"$$(git -C .. rev-parse --short HEAD)\"" > ts/src/_upstream.ts

build-python:
	cd python && rm -rf dist && poetry build

build-ts:
	cd ts && npm run build && npm pack

publish-python:
	cd python && poetry publish -r testpypi

publish-ts:
	cd ts && npm pack
	cd ts && npm publish --access public --tag dev

publish-ts-tag:
	cd ts && npm pack
	cd ts && npm publish --access public

release-python: generate-python version-for-py attach-commit-python build-python publish-python

release-ts: generate-ts version-for-ts attach-commit-ts build-ts publish-ts

release-ts-tag: generate-ts version-for-ts attach-commit-ts build-ts publish-ts-tag

