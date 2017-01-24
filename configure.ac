AC_INIT([tools],
	[unstable],
	[https://github.com/valhalla/tools/issues],
	[valhalla_tools-[unstable]],
	[https://github.com/valhalla/tools])
AC_CONFIG_AUX_DIR([.])
AM_INIT_AUTOMAKE([subdir-objects parallel-tests])
LT_INIT
AC_SUBST([LIBTOOL_DEPS])

AM_SILENT_RULES([yes])
AC_CONFIG_HEADERS([include/config.h])
AC_CONFIG_MACRO_DIR([m4])

# set pkgconfigdir, allow override
AC_ARG_WITH([pkgconfigdir],
            AS_HELP_STRING([--with-pkgconfigdir=PATH], [Path to the pkgconfig directory [[LIBDIR/pkgconfig]]]),
            [pkgconfigdir="$withval"],
            [pkgconfigdir='${libdir}/pkgconfig'])
AC_SUBST([pkgconfigdir])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_MAKE_SET

AC_HEADER_STDC
AC_LANG_CPLUSPLUS

# require c++11
AX_CXX_COMPILE_STDCXX_11([noext],[mandatory])

# require pthread
AX_PTHREAD(, [AC_MSG_ERROR([cannot find libpthread])])

# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable libvalhalla_sif = unstable libvalhalla_meili = unstable libvalhalla_skadi = unstable libvalhalla_loki = unstable libvalhalla_thor = unstable libvalhalla_odin = unstable libvalhalla_tyr = unstable libvalhalla_mjolnir = unstable])

# check for protocol buffers compiler and libraries
REQUIRE_PROTOC

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building tools. Please install libboost-dev.])])
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_SYSTEM
AX_BOOST_THREAD
AX_BOOST_FILESYSTEM

# if we wanted python bindings
AC_ARG_ENABLE([python_bindings],
    [  --enable-python-bindings    Create python bindings],
    [case "${enableval}" in
        yes) python_bindings=true ;;
        no)  python_bindings=false ;;
        *) AC_MSG_ERROR([bad value ${enableval} for --enable-python-bindings]) ;;
    esac],[python_bindings=false])
AM_CONDITIONAL([PYTHON_BINDINGS], [test x$python_bindings = xtrue])

# then we require python dev headers and boost python
AM_COND_IF([PYTHON_BINDINGS], [
    AX_PYTHON_DEVEL([>= '2.7'])
    AM_PATH_PYTHON([2.7],,)
    AX_BOOST_PYTHON
    AS_IF([test "x$BOOST_PYTHON_LIB" == "x"],
          [AC_MSG_ERROR([Boost::Python library not detected, please install libboost-python-dev.])],)
])

# check pkg-config dependencies
PKG_CHECK_MODULES([DEPS], [libprime_server >= 0.6.3])

# optionally enable coverage information
CHECK_COVERAGE

AC_CONFIG_FILES([Makefile])

# Debian resets this to no, but this break both Spot and the libtool
# test suite itself.  Instead of requiring developer to install a
# non-patched version of Libtool on any Debian they use, we just
# cancel the effect of Debian's patch here.
# see: http://git.lrde.epita.fr/?p=spot.git;a=commitdiff;h=0e74b76521341f670f6b76f8ef24a6dcf6e3813b
link_all_deplibs=yes
link_all_deplibs_CXX=yes

AC_OUTPUT

