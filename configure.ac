AC_INIT([thor],
	[0.0.1],
	[https://github.com/valhalla/thor/issues],
	[thor-0.0.1],
	[https://github.com/valhalla/thor])
AC_CONFIG_AUX_DIR([.])
AM_INIT_AUTOMAKE([subdir-objects parallel-tests])
LT_INIT
AC_SUBST([LIBTOOL_DEPS])

AM_SILENT_RULES([yes])
AC_CONFIG_HEADERS([valhalla/config.h])
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

# require other valhalla dependencies
CHECK_VALHALLA_MIDGARD
CHECK_VALHALLA_BALDR
CHECK_VALHALLA_SIF
CHECK_VALHALLA_MJOLNIR
CHECK_VALHALLA_LOKI
CHECK_VALHALLA_ODIN

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.54], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building thor. Please install libboost-dev.])])
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_SYSTEM
AX_BOOST_THREAD
AX_BOOST_FILESYSTEM

# check pkg-config dependencies
PKG_CHECK_MODULES([DEPS], [protobuf >= 2.4.0 libzmq >= 4.0 libprime_server >= 0.3.1])

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

