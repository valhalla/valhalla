# Test Coverage Report #

You can build a test coverage report using the `--enable-coverage` argument to `./configure`, then running `make coverage-report` to build an HTML coverage report in the `coverage/` directory.

Note also that, because calculating the coverage requires compiler support, you will need to clean any object files from a non-coverage build by running `make clean` before `make coverage-report`.

## Prerequisites ##

For this to work, you will need to have `lcov`, `gcov` and `genhtml` installed. On Ubuntu you can just run `sudo apt-get install lcov` to get these.
