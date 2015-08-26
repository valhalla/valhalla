AC_DEFUN([CHECK_VALHALLA_SKADI],
[
	AC_ARG_WITH([valhalla-skadi],
		[AS_HELP_STRING([--with-valhalla-skadi@<:@=ARG@:>@],
			[use the system valhalla-skadi (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_SKADI_CPPFLAGS=""
			VALHALLA_SKADI_LDFLAGS=""
		else
			VALHALLA_SKADI_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_SKADI_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_SKADI_CPPFLAGS=""
		VALHALLA_SKADI_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_CPPFLAGS $VALHALLA_SIF_CPPFLAGS $VALHALLA_MJOLNIR_CPPFLAGS $VALHALLA_SKADI_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_LDFLAGS $VALHALLA_BALDR_LDFLAGS $VALHALLA_SIF_LDFLAGS $VALHALLA_MJOLNIR_LDFLAGS $VALHALLA_SKADI_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::skadi library is available, ax_cv_valhalla_skadi,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/skadi/sample.h>]],
			[[using namespace valhalla::skadi;
			try{sample("");}catch(...){}]])],
			ax_cv_valhalla_skadi=yes, ax_cv_valhalla_skadi=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_skadi" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_SKADI,,[define if the valhalla::skadi library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_skadi!)
	fi

	AC_CHECK_LIB(valhalla_skadi, exit, [VALHALLA_SKADI_LIB="-lvalhalla_skadi"; AC_SUBST(VALHALLA_SKADI_LIB) link_skadi="yes"; break], [link_skadi="no"])

	if test "x$link_skadi" = "xyes"; then
		AC_SUBST(VALHALLA_SKADI_CPPFLAGS)
		AC_SUBST(VALHALLA_SKADI_LDFLAGS)
		VALHALLA_SKADI_LIBS="-lvalhalla_skadi"
		AC_SUBST(VALHALLA_SKADI_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_skadi!)
	fi
])
