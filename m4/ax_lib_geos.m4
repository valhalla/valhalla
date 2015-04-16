# SYNOPSIS
#
#   AX_LIB_GEOS([MINIMUM-VERSION])
#
# DESCRIPTION
#
#   This macro provides tests of availability of geos 'libgeos' library
#   of particular version or newer.
#
#   AX_LIB_GEOS macro takes only one argument which is optional. If
#   there is no required version passed, then macro does not run version
#   test.
#
#   The --with-geos option takes one of three possible values:
#
#   no - do not check for geos library
#
#   yes - do check for geos library in standard locations (geos-config
#   should be in the PATH)
#
#   path - complete path to geos-config utility, use this option if geos-config
#   can't be found in the PATH
#
#   This macro calls:
#
#     AC_SUBST(GEOS_CFLAGS)
#     AC_SUBST(GEOS_LDFLAGS)
#     AC_SUBST(GEOS_LIBS)
#     AC_SUBST(GEOS_VERSION)
#
#   And sets:
#
#     HAVE_GEOS
#
# LICENSE
#
#   Copyright (c) 2009 Hartmut Holzgraefe <hartmut@php.net>
#
#   Copying and distribution of this file, with or without modification, are
#   permitted in any medium without royalty provided the copyright notice
#   and this notice are preserved.

AC_DEFUN([AX_LIB_GEOS],
[
    AC_ARG_WITH([geos],
        AC_HELP_STRING([--with-geos=@<:@ARG@:>@],
            [use geos library @<:@default=yes@:>@, optionally specify path to geos-config]
        ),
        [
        if test "$withval" = "no"; then
            want_geos="no"
        elif test "$withval" = "yes"; then
            want_geos="yes"
        else
            want_geos="yes"
            GEOS_CONFIG="$withval"
        fi
        ],
        [want_geos="yes"]
    )

    GEOS_CFLAGS=""
    GEOS_LDFLAGS=""
    GEOS_LIBS=""
    GEOS_VERSION=""

    dnl
    dnl Check geos libraries (geos)
    dnl

    if test "$want_geos" = "yes"; then

        if test -z "$GEOS_CONFIG" -o test; then
            AC_PATH_PROG([GEOS_CONFIG], [geos-config], [])
        fi

        if test ! -x "$GEOS_CONFIG"; then
            AC_MSG_ERROR([${GEOS_CONFIG:-geos-config} does not exist or it is not an exectuable file])
            GEOS_CONFIG="no"
            found_geos="no"
        fi

        if test "$GEOS_CONFIG" != "no"; then
            AC_MSG_CHECKING([for geos libraries])

            GEOS_CFLAGS="`$GEOS_CONFIG --cflags`"
            GEOS_LDFLAGS="`$GEOS_CONFIG --ldflags`"
            GEOS_LIBS="`$GEOS_CONFIG --libs`"

            GEOS_VERSION=`$GEOS_CONFIG --version`

            dnl Headers are in a different package in Debian, so check again.
            ac_save_CPPFLAGS="$CPPFLAGS"
            CPPFLAGS="$CPPFLAGS $GEOS_CFLAGS"
            AC_CHECK_HEADER([geos/version.h], [],
                             [AC_MSG_ERROR([development headers for geos not found])])
            CPPFLAGS="$ac_save_CPPFLAGS"

            AC_DEFINE([HAVE_GEOS], [1],
                [Define to 1 if geos libraries are available])

            found_geos="yes"
            AC_MSG_RESULT([yes])
        else
            found_geos="no"
            AC_MSG_RESULT([no])
        fi
    fi

    dnl
    dnl Check if required version of geos is available
    dnl


    geos_version_req=ifelse([$1], [], [], [$1])


    if test "$found_geos" = "yes" -a -n "$geos_version_req"; then

        AC_MSG_CHECKING([if geos version is >= $geos_version_req])

        dnl Decompose required version string of geos
        dnl and calculate its number representation
        geos_version_req_major=`expr $geos_version_req : '\([[0-9]]*\)'`
        geos_version_req_minor=`expr $geos_version_req : '[[0-9]]*\.\([[0-9]]*\)'`
        geos_version_req_micro=`expr $geos_version_req : '[[0-9]]*\.[[0-9]]*\.\([[0-9]]*\)'`
        if test "x$geos_version_req_micro" = "x"; then
            geos_version_req_micro="0"
        fi

        geos_version_req_number=`expr $geos_version_req_major \* 1000000 \
                                   \+ $geos_version_req_minor \* 1000 \
                                   \+ $geos_version_req_micro`

        dnl Decompose version string of installed PostgreSQL
        dnl and calculate its number representation
        geos_version_major=`expr $GEOS_VERSION : '\([[0-9]]*\)'`
        geos_version_minor=`expr $GEOS_VERSION : '[[0-9]]*\.\([[0-9]]*\)'`
        geos_version_micro=`expr $GEOS_VERSION : '[[0-9]]*\.[[0-9]]*\.\([[0-9]]*\)'`
        if test "x$geos_version_micro" = "x"; then
            geos_version_micro="0"
        fi

        geos_version_number=`expr $geos_version_major \* 1000000 \
                                   \+ $geos_version_minor \* 1000 \
                                   \+ $geos_version_micro`

        geos_version_check=`expr $geos_version_number \>\= $geos_version_req_number`
        if test "$geos_version_check" = "1"; then
            AC_MSG_RESULT([yes])
        else
            AC_MSG_RESULT([no])
        fi
    fi

    AC_SUBST([GEOS_VERSION])
    AC_SUBST([GEOS_CFLAGS])
    AC_SUBST([GEOS_LDFLAGS])
    AC_SUBST([GEOS_LIBS])
])

