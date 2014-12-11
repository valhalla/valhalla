AC_INIT([valhalla],
	[0.0.1],
	[https://github.com/mapzen/valhalla/issues],
	[valhalla-0.0.1],
	[https://github.com/mapzen/valhalla])
AM_INIT_AUTOMAKE([subdir-objects parallel-tests])
LT_INIT
AC_SUBST([LIBTOOL_DEPS])

AM_SILENT_RULES([yes])
AC_CONFIG_HEADERS([include/config.h])
AC_CONFIG_MACRO_DIR([m4])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_MAKE_SET

AC_HEADER_STDC
AC_LANG_CPLUSPLUS

# require c++11
AX_CXX_COMPILE_STDCXX_11([noext],[mandatory])

# check for protocol buffers compiler and libraries
REQUIRE_PROTOC

# check for SQLite, which we use for HTTP cache information
AX_LIB_SQLITE3([3.6.16])
AM_CONDITIONAL([HAVE_SQLITE3], [test -n "$SQLITE3_VERSION"])

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.46], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building valhalla. Please install libboost-dev.])])
AX_BOOST_PROGRAM_OPTIONS

# check for Lua libraries and headers
AX_PROG_LUA([5.2],[],[
    AX_LUA_HEADERS([
        AX_LUA_LIBS([
            AC_DEFINE([HAVE_LUA], [1], [Requirements for lua are met])
            HAVE_LUA=yes
        ],[AC_MSG_ERROR([Cannot find Lua libs.   Please install lua5.2 liblua5.2-dev])])
    ],[AC_MSG_ERROR([Cannot find Lua includes.  Please install lua5.2 liblua5.2-dev])])
],[AC_MSG_ERROR([Cannot find Lua interpreter.   Please install lua5.2 liblua5.2-dev])])

# check pkg-config packaged packages.
PKG_CHECK_MODULES([DEPS], [protobuf >= 2.4.0 libzmq >= 2.1.10 libcurl >= 7.19.5])

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

