# Unit tests

Valhalla currently uses googletest for unit testing.

- The basics are covered in [googletest primer](https://github.com/google/googletest/blob/master/docs/primer.md)
- More advanced topics  are covered [here](https://github.com/google/googletest/blob/master/docs/advanced.md)

Important things to note:
* EXPECT_XXX macros mark the test as failed but continue execution
    * Allows to test more at once
    * A preferred way over ASSERT_XXX
    * ASSERT_XXX family should be used when it's pointless to continue the test
        * e.g. the method we test returns nullptr instead of  a valid pointer
* Tests execution order is unspecified
    * If the order is needed fixtures or test suite environments may help
    * Sometimes it may hint the test design needs rethinking
* Be careful with _ in test names - gtest joins a suite name with test name using _
    * TEST(Foo, Basic_test) and TEST(Foo_Basic, test) will produce a conflict
    * In practice it never happens - no need to worry much about it
* Be aware of curly braces{} inside macros
    * A typical case -> container with initializer list creation inside assert macro
    * Preprocessor is dumb and thinks commas separate the arguments
    * ASSERT_EQ(foo, vector{1,2,3});
    * Fixing this can be done by adding () - ASSERT_EQ(foo, (vector{1,2,3}));
* Disabling a test is easy - just prefix its name with DISABLED_
    * TEST(PredictiveTraffic, DISABLED_test_predictive_traffic)
    * They are still compiled and there is always a warning each time the suite runs
