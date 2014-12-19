AC_DEFUN([CHECK_VALHALLA_ODIN],
[
	AC_ARG_WITH([valhalla-odin],
		[AS_HELP_STRING([--with-valhalla-odin@<:@=ARG@:>@],
			[use the system valhalla-odin (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_ODIN_CPPFLAGS=""
			VALHALLA_ODIN_LDFLAGS=""
		else
			VALHALLA_ODIN_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_ODIN_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_ODIN_CPPFLAGS=""
		VALHALLA_ODIN_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_ODIN_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_ODIN_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::odin library is available, ax_cv_valhalla_odin,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/proto/trippath.pb.h>]],
			[[using namespace valhalla::odin;
			TripPath();
			return 0;]])],
			ax_cv_valhalla_odin=yes, ax_cv_valhalla_odin=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_odin" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_ODIN,,[define if the valhalla::odin library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_odin!)
	fi

	AC_CHECK_LIB(valhalla_odin, exit, [VALHALLA_ODIN_LIB="-lvalhalla_odin"; AC_SUBST(VALHALLA_ODIN_LIB) link_odin="yes"; break], [link_odin="no"])

	if test "x$link_odin" = "xyes"; then
		AC_SUBST(VALHALLA_ODIN_CPPFLAGS)
		AC_SUBST(VALHALLA_ODIN_LDFLAGS)
		VALHALLA_ODIN_LIBS="-lvalhalla_odin"
		AC_SUBST(VALHALLA_ODIN_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_odin!)
	fi
])
