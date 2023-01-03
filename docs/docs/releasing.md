# Releasing a new Valhalla version

We are using http://semver.org/ for versioning with major, minor and patch versions.

## Guarantees

We are giving the following guarantees between versions:

### Major version change

- There are no guarantees about compatiblity of APIs or datasets
- Breaking changes will be noted as `BREAKING` in the changelog

### Minor version change

We may introduce forward-compatible changes: query parameters and response properties may be added in responses, but existing properties may not be changed or removed. One exception to this is the addition of new turn types, which we see as forward-compatible changes.

- Forward-compatible HTTP API
- Forward-compatible C++ library API
- Forward-compatible dataset version

### Patch version change

- No change of query parameters or response formats
- Compatible HTTP API
- Compatible C++ library API
- Compatible data datasets

## Release and branch management

- The `master` branch is for the bleeding edge development
- We create and maintain release branches `x.y` to control the release flow
- We create the release branch once we create release branches once we want to release the first RC
- RCs go in the release branch, commits needs to be cherry-picked from master
- No minor or major version will be released without a code-equal release candidates
- For quality assurance, release candidates need to be staged before tagging a final release
- Patch versions may be released without a release candidate
- We may backport fixes to older versions and release them as patch versions

## Releasing a version

1. Update the translations from Transifex, see [README](https://github.com/valhalla/valhalla/blob/master/README.md).
2. Check out the appropriate release branch `x.y`.
3. Make sure `CHANGELOG.md` is up to date.
4. Make sure all tests are passing (e.g. Circle CI gives you a :green_apple:).
5. Make sure the [`valhalla/valhalla.h`](https://github.com/valhalla/valhalla/blob/master/valhalla/valhalla.h) has been updated.
6. Use an annotated tag to mark the release: `git tag x.y.z -a`. The body of the tag description
   should be the changelog entries.
6. Push tags and commits: `git push; git push --tags`
7. On https://github.com/valhalla/valhalla/releases press `Draft a new release`, write the release
   tag `x.y.z` in the `Tag version` field, write the changelog entries in the `Describe this
   release` field and press `Publish release`.
