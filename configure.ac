<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
AC_INIT([meili],
	[unstable],
	[https://github.com/valhalla/meili/issues],
	[valhalla_meili-[unstable]],
	[https://github.com/valhalla/meili])
>>>>>>> meili/master
=======
AC_INIT([skadi],
	[unstable],
	[https://github.com/valhalla/skadi/issues],
	[valhalla_skadi-[unstable]],
	[https://github.com/valhalla/skadi])
>>>>>>> skadi/master
=======
AC_INIT([mjolnir],
	[unstable],
	[https://github.com/valhalla/mjolnir/issues],
	[valhalla_mjolnir-[unstable]],
	[https://github.com/valhalla/mjolnir])
>>>>>>> mjolnir/master
=======
AC_INIT([odin],
	[unstable],
	[https://github.com/valhalla/odin/issues],
	[valhalla_odin-[unstable]],
	[https://github.com/valhalla/odin])
>>>>>>> odin/master
=======
AC_INIT([loki],
	[unstable],
	[https://github.com/valhalla/loki/issues],
	[valhalla_loki-[unstable]],
	[https://github.com/valhalla/loki])
>>>>>>> loki/master
=======
AC_INIT([thor],
	[unstable],
	[https://github.com/valhalla/thor/issues],
	[valhalla_thor-[unstable]],
	[https://github.com/valhalla/thor])
>>>>>>> thor/master
=======
AC_INIT([tyr],
	[unstable],
	[https://github.com/valhalla/tyr/issues],
	[valhalla_tyr-[unstable]],
	[https://github.com/valhalla/tyr])
>>>>>>> tyr/master
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
# require pthread
AX_PTHREAD(, [AC_MSG_ERROR([cannot find libpthread])])

>>>>>>> skadi/master
# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable])

# check for boost and make sure we have the program options library
<<<<<<< HEAD
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building sif. Please install libboost-dev.])])
AX_BOOST_SYSTEM
AX_BOOST_THREAD
>>>>>>> sif/master
=======
# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable libvalhalla_sif = unstable])

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building valhalla. Please install libboost-dev.])])
=======
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building skadi. Please install libboost-dev.])])
>>>>>>> skadi/master
=======
# require pthread
AX_PTHREAD(, [AC_MSG_ERROR([cannot find libpthread])])

# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable libvalhalla_skadi = unstable])

# check for protocol buffers compiler and libraries
REQUIRE_PROTOC

<<<<<<< HEAD
# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building mjolnir. Please install libboost-dev.])])
>>>>>>> mjolnir/master
=======
# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable])

# check for protocol buffers compiler and libraries
REQUIRE_PROTOC

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building odin. Please install libboost-dev.])])
>>>>>>> odin/master
=======
# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable libvalhalla_meili = unstable libvalhalla_sif = unstable libvalhalla_odin = unstable])

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building thor. Please install libboost-dev.])])
>>>>>>> thor/master
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_SYSTEM
AX_BOOST_THREAD
AX_BOOST_FILESYSTEM
<<<<<<< HEAD
<<<<<<< HEAD
AX_PTHREAD(, [AC_MSG_ERROR([cannot find libpthread])])

# check pkg-config dependencies
PKG_CHECK_MODULES([DEPS], [libprime_server >= 0.6.3])
>>>>>>> meili/master
=======
AX_BOOST_REGEX

<<<<<<< HEAD
# check pkg-config dependencies
PKG_CHECK_MODULES([DEPS], [libprime_server >= 0.6.3])
>>>>>>> skadi/master
=======
AX_LIB_SQLITE3(3.0.0)

if test "x$SQLITE3_VERSION" = "x"; then 
  AC_MSG_ERROR(['libsqlite-dev' version >= 3.0.0 is required.  Please install libsqlite-dev.]) 
fi 

<<<<<<< HEAD
# Check for Geos library
AX_LIB_GEOS(3.0.0)
if test "x$GEOS_VERSION" = "x"
then
  AC_MSG_ERROR(['geos' version >= 3.0.0 is required.  Please install geos.]);
fi

# spatialite needed for admin info
PKG_CHECK_MODULES([LIBSPATIALITE], [spatialite >= 3.0.0], , AC_MSG_ERROR(['libspatialite-dev' version >= 3.0.0 is required.  Please install libspatialite-dev.]))

# check pkg-config packaged packages.
PKG_CHECK_MODULES([DEPS], [protobuf >= 2.4.0 libcurl >= 7.35.0])
>>>>>>> mjolnir/master
=======
AX_BOOST_REGEX
AX_BOOST_DATE_TIME

# check pkg-config dependencies
PKG_CHECK_MODULES([DEPS], [protobuf >= 2.4.0 libprime_server >= 0.6.3])
>>>>>>> odin/master
=======
# require pthread
AX_PTHREAD(, [AC_MSG_ERROR([cannot find libpthread])])

# require other valhalla dependencies
<<<<<<< HEAD
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable libvalhalla_sif = unstable])

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building loki. Please install libboost-dev.])])
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_SYSTEM
AX_BOOST_THREAD

# check pkg-config dependencies
PKG_CHECK_MODULES([DEPS], [libprime_server = 0.6.3])
>>>>>>> loki/master
=======
# check pkg-config dependencies
PKG_CHECK_MODULES([DEPS], [protobuf >= 2.4.0 libprime_server >= 0.6.3])
>>>>>>> thor/master
=======
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable libvalhalla_odin = unstable])

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building tyr. Please install libboost-dev.])])
AX_BOOST_SYSTEM
AX_BOOST_THREAD

# check zeromq version
PKG_CHECK_MODULES([DEPS], [libprime_server >= 0.6.3])
>>>>>>> tyr/master

# optionally enable coverage information
CHECK_COVERAGE

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
AC_CONFIG_FILES([Makefile libvalhalla_midgard.pc])
=======
AC_CONFIG_FILES([Makefile libvalhalla_baldr.pc])
>>>>>>> baldr/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_sif.pc])
>>>>>>> sif/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_meili.pc])
>>>>>>> meili/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_skadi.pc])
>>>>>>> skadi/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_mjolnir.pc])
>>>>>>> mjolnir/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_odin.pc])
>>>>>>> odin/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_loki.pc])
>>>>>>> loki/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_thor.pc])
>>>>>>> thor/master
=======
AC_CONFIG_FILES([Makefile libvalhalla_tyr.pc])
>>>>>>> tyr/master

# Debian resets this to no, but this break both Spot and the libtool
# test suite itself.  Instead of requiring developer to install a
# non-patched version of Libtool on any Debian they use, we just
# cancel the effect of Debian's patch here.
# see: http://git.lrde.epita.fr/?p=spot.git;a=commitdiff;h=0e74b76521341f670f6b76f8ef24a6dcf6e3813b
link_all_deplibs=yes
link_all_deplibs_CXX=yes

AC_OUTPUT

