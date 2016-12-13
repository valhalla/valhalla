AC_INIT([mjolnir],
	[unstable],
	[https://github.com/valhalla/mjolnir/issues],
	[valhalla_mjolnir-[unstable]],
	[https://github.com/valhalla/mjolnir])
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

# require pthread
AX_PTHREAD(, [AC_MSG_ERROR([cannot find libpthread])])

# require other valhalla dependencies
PKG_CHECK_MODULES([VALHALLA_DEPS], [libvalhalla_midgard = unstable libvalhalla_baldr = unstable libvalhalla_skadi = unstable])

# check for protocol buffers compiler and libraries
REQUIRE_PROTOC

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building mjolnir. Please install libboost-dev.])])
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_SYSTEM
AX_BOOST_THREAD
AX_BOOST_FILESYSTEM

# check for Lua libraries and headers
AX_PROG_LUA([5.2],[],[
    AX_LUA_HEADERS([
        AX_LUA_LIBS([
        ],[AC_MSG_ERROR([Cannot find Lua libs.   Please install lua5.2 liblua5.2-dev])])
    ],[AC_MSG_ERROR([Cannot find Lua includes.  Please install lua5.2 liblua5.2-dev])])
],[AC_MSG_ERROR([Cannot find Lua interpreter.   Please install lua5.2 liblua5.2-dev])])

AX_LIB_SQLITE3(3.0.0)

if test "x$SQLITE3_VERSION" = "x"; then 
  AC_MSG_ERROR(['libsqlite-dev' version >= 3.0.0 is required.  Please install libsqlite-dev.]) 
fi 

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

# optionally enable coverage information
CHECK_COVERAGE

AC_CONFIG_FILES([Makefile libvalhalla_mjolnir.pc])

# Debian resets this to no, but this break both Spot and the libtool
# test suite itself.  Instead of requiring developer to install a
# non-patched version of Libtool on any Debian they use, we just
# cancel the effect of Debian's patch here.
# see: http://git.lrde.epita.fr/?p=spot.git;a=commitdiff;h=0e74b76521341f670f6b76f8ef24a6dcf6e3813b
link_all_deplibs=yes
link_all_deplibs_CXX=yes

AC_OUTPUT

