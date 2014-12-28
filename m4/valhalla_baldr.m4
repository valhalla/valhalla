AC_DEFUN([CHECK_VALHALLA_BALDR],
[
	AC_ARG_WITH([valhalla-baldr],
		[AS_HELP_STRING([--with-valhalla-baldr@<:@=ARG@:>@],
			[use the system valhalla-baldr (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_BALDR_CPPFLAGS=""
			VALHALLA_BALDR_LDFLAGS=""
		else
			VALHALLA_BALDR_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_BALDR_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_BALDR_CPPFLAGS=""
		VALHALLA_BALDR_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::baldr library is available, ax_cv_valhalla_baldr,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/baldr/graphid.h>]],
			[[using namespace valhalla::baldr;
			GraphId(0,0,0);]])],
			ax_cv_valhalla_baldr=yes, ax_cv_valhalla_baldr=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_baldr" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_BALDR,,[define if the valhalla::baldr library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_baldr!)
	fi

	AC_CHECK_LIB(valhalla_baldr, exit, [VALHALLA_BALDR_LIB="-lvalhalla_baldr"; AC_SUBST(VALHALLA_BALDR_LIB) link_baldr="yes"; break], [link_baldr="no"])

	if test "x$link_baldr" = "xyes"; then
		AC_SUBST(VALHALLA_BALDR_CPPFLAGS)
		AC_SUBST(VALHALLA_BALDR_LDFLAGS)
		VALHALLA_BALDR_LIBS="-lvalhalla_baldr"
		AC_SUBST(VALHALLA_BALDR_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_baldr!)
	fi
])
