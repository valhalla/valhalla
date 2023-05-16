We welcome contributions to Valhalla. If you would like to report an issue, or even better fix an existing one, please use the [Valhalla issue tracker](https://github.com/valhalla/valhalla/issues) on GitHub.

There are many ways to make meaningful contributions to the project:
- file issues & bugs with **clear and easily reproducible instructions**
- contribute bug fixes
- contribute feature implementations, for larger features/changes its best to **open an issue asking for feedback before starting implementation**
- improve translations on [Transifex](https://www.transifex.com/valhalla/valhalla-phrases/locales-en-us-json--transifex/)

## Code contributions

We appreciate the community picking up and fixing bugs or even implementing new features. There are a few things to follow/be aware of when working on Valhalla:
- we currently use the C++17 standard
- we use `pre-commit` to make sure commits are formatted & linted: run `./scripts/format.sh` once and it'll be installed
- `clang-format`/`clang-tidy` is used to format/lint the C++ code, `black` & `flake8` format/lint Python code
- [`ASan`](https://clang.llvm.org/docs/AddressSanitizer.html) is run in CI, but without its integrated leak sanitizer due to platform issues
- we ask for unit tests to demonstrate a working bug fix or feature implementation, please feel free to ask us for instructions in the PR

Before opening a PR we'd ask you to format & lint the code:
```
# installs the pre-commit hook
./scripts/format.sh
pre-commit run --all-files

# optionally run clang-tidy
# needs the compile_commands.json, we have to rebuild Valhalla
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=On && make -C build
./scripts/clang-tidy-only-diff.sh

# optionally run with ASan
cmake -B build -DENABLE_ADDRESS_SANITIZER=ON -DCMAKE_BUILD_TYPE=Debug && make -C build
# if the leak sanitizer is problematic, you can disable it with 
export ASAN_OPTIONS=detect_leaks=0
```

### Unit/integration Tests

We highly encourage running and updating the tests to make sure no regressions have been made. We use the Google test suite and also created our own test framework called "gurka" (the Norsk counterpart to OSRM's "cucumber"), to easily test custom-built maps. Check out more information in the [test docs](test/gurka/README.md).

To build and run all tests:

```
make check -j$(nproc)
```

To run an individual test, first move into the `./build` directory (lots of paths in the tests are relative):

```
cd build
make run-<test name>
```

Coverage reports are automatically generated using `codecov` for each pull request, but you can also build them locally by passing `-DENABLE_COVERAGE=On` and running `make coverage`.

### Performance tests

This will mostly be performed by the maintainers before we merge PRs which could have a significant impact on performance or quality.

1. Take a giant file with routes with > 10k
2. Spin up the service
3. Run the requests with `./run_route_scripts/run_with_server.py`: we can alter the logic for the requests inside that script, depending what we want to test
4. E.g. `./run_route_scripts/run_with_server.py --test-file auto.txt --url http://localhost:8002/route --concurrency 20 --format csv`


## Translation contributions

Valhalla currently supports almost 30 languages with > 95% translation coverage. If you find that Valhalla's output instructions are not supported or not optimal in your favorite language, it'd be great if you took the time to contribute improvements. We're using [Transifex](https://www.transifex.com/valhalla/valhalla-phrases/locales-en-us-json--transifex/) to manage translations and try to download all improvements before each release.

You can find more information in the [dedicated README](locales/README.md).
