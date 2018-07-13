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

1. Check out the appropriate release branch `x.y`
2. Make sure `CHANGELOG.md` is up to date.
3. Make sure the `package.json` on branch `x.y` has been committed.
4. Make sure all tests are passing (e.g. Circle CI gives you a :green_apple:)
5. Use an annotated tag to mark the release: `git tag x.y.z -a` Body of the tag description should be the changelog entries.
6. Push tags and commits: `git push; git push --tags`
7. On https://github.com/valhalla/valhalla/releases press `Draft a new release`,
   write the release tag `x.y.z` in the `Tag version` field, write the changelog entries in the `Describe this release` field
   and press `Publish release`.
8. If you are publishing to npm:
    - your binaries will get published to s3 by circle when you push a tag that matches the package.json version
    - Locally you can now test binaries. Cleanup, re-install, and run the tests like:
       ```
       make clean
       npm install # will pull remote binaries
       npm ls # confirm deps are correct
       npm test
       ```
    - you can also force circle to publish binaries by putting the string "[publish binary]" in your commit message
    - Publish node-valhalla

       First ensure your local `node-pre-gyp` is up to date:

       ```
       npm ls
       ```

       This is important because it is bundled during packaging.

       If you see any errors then do:

       ```
       rm -rf node_modules/node-pre-gyp
       npm install node-pre-gyp
       ```

       Test the node module using `npm pack`. It will provide the packed file as a tarball for download. When you then run `npm install` it downloads the tarball and installs it. This test ensures you can do the same locally.
        - [ ] Run `npm install`
        - [ ] Test packaging locally with `npm pack`
          - [ ] run `npm pack`
          - [ ] move the tarball to a fresh new folder
          - [ ] run `npm install <name-of-tarball>`

       Now we're ready to publish `valhalla` to <https://www.npmjs.org/package/valhalla>:

       ```
       npm publish
       ```

       Dependent apps can now pull from the npm registry like:

       ```
       "dependencies": {
           "valhalla": "^MAJOR.MINOR.PATCH"
       }
       ```

       Or can still pull from the github tag like:

       ```
       "dependencies": {
           "valhalla": "https://github.com/Project-Valhalla/node-valhalla/archive/vMAJOR.MINOR.PATCH.tar.gz"
       }
       ```
