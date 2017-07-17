AC_INIT([valhalla],
	[m4_esyscmd([./version.sh])],
	[https://github.com/valhalla/valahlla/issues],
	[valhalla-[m4_esyscmd([./version.sh])]],
	[https://github.com/valhalla/valahlla])
AC_CONFIG_AUX_DIR([.])
AM_INIT_AUTOMAKE([subdir-objects parallel-tests])
# no static lib by default but you can ask for it
LT_INIT([disable-static])
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

# check for protocol buffers compiler and libraries
REQUIRE_PROTOC

# check for boost and make sure we have the program options library
AX_BOOST_BASE([1.51], , [AC_MSG_ERROR([cannot find Boost libraries, which are are required for building valhalla. Please install libboost-all-dev.])])
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_SYSTEM
AX_BOOST_THREAD
AX_BOOST_FILESYSTEM
AX_BOOST_REGEX
AX_BOOST_DATE_TIME
AX_BOOST_IOSTREAMS
AC_SUBST(BOOST_CPPFLAGS, "$BOOST_CPPFLAGS -DBOOST_SPIRIT_THREADSAFE -DBOOST_NO_CXX11_SCOPED_ENUMS")
AC_SUBST(RAPIDJSON_CPPFLAGS, "$RAPIDJSON_CPPFLAGS -DRAPIDJSON_HAS_STDSTRING")

PKG_CHECK_MODULES([DEPS], [protobuf >= 2.4.0])

# if we wanted services
AC_ARG_ENABLE([services],
    [  --enable-services    Create services],
    [case "${enableval}" in
        yes) services=true ;;
        no)  services=false ;;
        *) AC_MSG_ERROR([bad value ${enableval} for --enable-services]) ;;
    esac],[services=true])
AM_CONDITIONAL([SERVICES], [test x$services = xtrue])

# then we require prime_server
AM_COND_IF([SERVICES], [
    PKG_CHECK_MODULES([SERVICE_DEPS], [libprime_server = 0.6.3])
    AC_SUBST(HAVE_HTTP_DEFINE, ["#define HAVE_HTTP"])
],[AC_SUBST(HAVE_HTTP_DEFINE, ["#undef HAVE_HTTP"])])

# if we wanted python bindings
AC_ARG_ENABLE([python_bindings],
    [  --enable-python-bindings    Create python bindings],
    [case "${enableval}" in
        yes) python_bindings=true ;;
        no)  python_bindings=false ;;
        *) AC_MSG_ERROR([bad value ${enableval} for --enable-python-bindings]) ;;
    esac],[python_bindings=true])
AM_CONDITIONAL([PYTHON_BINDINGS], [test x$python_bindings = xtrue])

# then we require python dev headers and boost python
AM_COND_IF([PYTHON_BINDINGS], [
    AX_PYTHON_DEVEL([>= '2.7'])
    AM_PATH_PYTHON([2.7],,)
    AX_BOOST_PYTHON
    AS_IF([test "x$BOOST_PYTHON_LIB" == "x"], [AC_MSG_ERROR([Boost::Python is required, please install libboost-python-dev.])])
])

# if we wanted data tools
AC_ARG_ENABLE([data_tools],
    [  --enable-data-tools    Create data building tools],
    [case "${enableval}" in
        yes) data_tools=true ;;
        no)  data_tools=false ;;
        *) AC_MSG_ERROR([bad value ${enableval} for --enable-data-tools]) ;;
    esac],[data_tools=true])
AM_CONDITIONAL([DATA_TOOLS], [test x$data_tools = xtrue])

# then we require lots more stuff
AM_COND_IF([DATA_TOOLS], [

    PKG_CHECK_MODULES([DATA_DEPS], [libcurl >= 7.35.0])

    # check for lua libraries and headers
    AX_PROG_LUA([5.2],[],[
        AX_LUA_HEADERS([
            AX_LUA_LIBS([
            ],[AC_MSG_ERROR([Cannot find Lua libs.   Please install lua5.2 liblua5.2-dev])])
        ],[AC_MSG_ERROR([Cannot find Lua includes.  Please install lua5.2 liblua5.2-dev])])
    ],[AC_MSG_ERROR([Cannot find Lua interpreter.   Please install lua5.2 liblua5.2-dev])])

    # check for sqlite
    AX_LIB_SQLITE3(3.0.0)
    AS_IF([test "x$SQLITE3_VERSION" = "x"], [AC_MSG_ERROR(['libsqlite-dev' version >= 3.0.0 is required.  Please install libsqlite-dev.])])

    # check for geos library    
    AX_LIB_GEOS(3.0.0)
    AS_IF([test "x$GEOS_VERSION" = "x"], [AC_MSG_ERROR(['geos' version >= 3.0.0 is required.  Please install geos.])])

    # spatialite needed for admin info
    PKG_CHECK_MODULES([LIBSPATIALITE], [spatialite >= 3.0.0], , AC_MSG_ERROR(['libspatialite-dev' version >= 3.0.0 is required.  Please install libspatialite-dev.]))

])

# optionally enable coverage information
CHECK_COVERAGE

AC_CONFIG_FILES([Makefile libvalhalla.pc valhalla/valhalla.h])

# Debian resets this to no, but this break both Spot and the libtool
# test suite itself.  Instead of requiring developer to install a
# non-patched version of Libtool on any Debian they use, we just
# cancel the effect of Debian's patch here.
# see: http://git.lrde.epita.fr/?p=spot.git;a=commitdiff;h=0e74b76521341f670f6b76f8ef24a6dcf6e3813b
link_all_deplibs=yes
link_all_deplibs_CXX=yes

AC_OUTPUT
