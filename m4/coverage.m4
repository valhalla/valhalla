# ============================================================
# checks for whether we want to build with coverage information
# and use that when running the tests to generate test coverage
# statistics.

AC_DEFUN([CHECK_COVERAGE],
[
AC_ARG_ENABLE([coverage],
  [AS_HELP_STRING([--enable-coverage],
    [enable coverage testing])],
  [enable_coverage=yes],[enable_coverage=no])

AM_CONDITIONAL(ENABLE_COVERAGE, test "x$enable_coverage" = "xyes")

if test "x$enable_coverage" = "xyes"; then
  AC_CHECK_PROG(LCOV, lcov, lcov, [])
  AC_CHECK_PROG(GENHTML, genhtml, genhtml, [])

  if test "x$LCOV" = "x"; then
    AC_MSG_ERROR([cannot find lcov in your path, but it is required for building avecado. Please install lcov.])
  fi
  if test "x$GENHTML" = "x"; then
    AC_MSG_ERROR([cannot find genhtml in your path, but it is required for building avecado. Please install lcov.])
  fi

  # need to not have optimisation flags, as this makes the line
  # numbers go all funny and pretty much impossible to read.
  changequote({,})
  CFLAGS=`echo "$CFLAGS" | $SED -e 's/-O[0-9]*/-O0/g'`
  CXXFLAGS=`echo "$CXXFLAGS" | $SED -e 's/-O[0-9]*/-O0/g'`
  changequote([,])

  # flags to GCC to make it output the coverage information
  COVERAGE_CFLAGS="-O0 -fprofile-arcs -ftest-coverage"
  COVERAGE_CXXFLAGS="-O0 -fprofile-arcs -ftest-coverage"
  COVERAGE_LDFLAGS="-lgcov"
fi

# make variables available to automake - even if they are
# empty.
AC_SUBST(COVERAGE_CFLAGS)
AC_SUBST(COVERAGE_CXXFLAGS)
AC_SUBST(COVERAGE_LDFLAGS)

])
