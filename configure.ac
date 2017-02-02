<<<<<<< HEAD
<<<<<<< HEAD
AC_INIT([midgard],
	[unstable],
	[https://github.com/valhalla/midgard/issues],
	[valhalla_midgard-[unstable]],
	[https://github.com/valhalla/midgard])
=======
AC_INIT([baldr],
	[unstable],
	[https://github.com/valhalla/baldr/issues],
	[valhalla_baldr-[unstable]],
	[https://github.com/valhalla/baldr])
>>>>>>> baldr/master
=======
AC_INIT([sif],
	[unstable],
	[https://github.com/valhalla/sif/issues],
	[valhalla_sif-[unstable]],
	[https://github.com/valhalla/sif])
>>>>>>> sif/master
AC_CONFIG_AUX_DIR([.])
AM_INIT_AUTOMAKE([subdir-objects parallel-tests])
LT_INIT
AC_SUBST([LIBTOOL_DEPS])

AM_SILENT_RULES([yes])
AC_CONFIG_HEADERS([valhalla/config.h])
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

<<<<<<< HEAD
<<<<<<< HEAD
# require pthread
AX_PTHREAD(, [AC_MSG_ERROR([cannot find libpthread])])
=======
# require other valhalla dependencies with matching version based on tag
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable])

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building valhalla. Please install libboost-dev.])])
AX_BOOST_SYSTEM
AX_BOOST_THREAD
AX_BOOST_FILESYSTEM
AX_BOOST_DATE_TIME

AC_ARG_ENABLE([std_regex],
              AS_HELP_STRING([--enable-std-regex], [Use regex from the standard library instead of boost]))
AS_IF([test "x$enable_std_regex" = "xyes"],
      [AC_DEFINE(USE_STD_REGEX, [], [Use regex from the standard library instead of boost])],
      [AX_BOOST_REGEX])
>>>>>>> baldr/master
=======
# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable])

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building sif. Please install libboost-dev.])])
AX_BOOST_SYSTEM
AX_BOOST_THREAD
>>>>>>> sif/master

# optionally enable coverage information
CHECK_COVERAGE

<<<<<<< HEAD
<<<<<<< HEAD
AC_CONFIG_FILES([Makefile libvalhalla_midgard.pc])
=======
AC_CONFIG_FILES([Makefile libvalhalla_baldr.pc])
>>>>>>> baldr/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_sif.pc])
>>>>>>> sif/master

# Debian resets this to no, but this break both Spot and the libtool
# test suite itself.  Instead of requiring developer to install a
# non-patched version of Libtool on any Debian they use, we just
# cancel the effect of Debian's patch here.
# see: http://git.lrde.epita.fr/?p=spot.git;a=commitdiff;h=0e74b76521341f670f6b76f8ef24a6dcf6e3813b
link_all_deplibs=yes
link_all_deplibs_CXX=yes

AC_OUTPUT

