# Releasing

Releasing a new version of `node-valhalla` is mostly automated using CircleCI.

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

These steps all happen on `master`. After the release is out, create a branch using the MAJOR.MINOR version of the release to document code changes made for that version.

### Steps to release

1. Update the `package.json` to the correct version.

   Confirm the desired Valhalla branch and commit to `master`. Update the `CHANGELOG.md`.

1. Check that CircleCI for the latest commit on `master`.

1. Publishing binaries

   If circle builds are passing then it's time to publish binaries by committing with a message containing `[publish binary]`. Use an empty commit for this.

   ```
   git commit --allow-empty -m "[publish binary] vMAJOR.MINOR.PATCH"
   ```

1. Test

   Locally you can now test binaries. Cleanup, re-install, and run the tests like:

   ```
   make clean
   npm install # will pull remote binaries
   npm ls # confirm deps are correct
   make test
   ```

1. Tag

   Once binaries are published for Linux and OS X then its time to tag a new release and add the changelog to the tag:

   ```
   git tag vMAJOR.MINOR.PATCH -a
   git push origin <tag-name>
   ```

1. Publish node-valhalla. **we only do this for stable releases**

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

   Now we're ready to publish `node-valhalla` to <https://www.npmjs.org/package/valhalla>:

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
