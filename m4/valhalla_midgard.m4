AC_DEFUN([CHECK_VALHALLA_MIDGARD],
[
	AC_ARG_WITH([valhalla-midgard],
		[AS_HELP_STRING([--with-valhalla-midgard@<:@=ARG@:>@],
			[use the system valhalla-midgard (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_MIDGARD_CPPFLAGS=""
			VALHALLA_MIDGARD_LDFLAGS=""
		else
			VALHALLA_MIDGARD_CPPFLAGS="-I$withval/include"
			VALHALLA_MIDGARD_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_MIDGARD_CPPFLAGS=""
		VALHALLA_MIDGARD_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::midgard library is available, ax_cv_valhalla_midgard,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/midgard/util.h>]],
			[[using namespace valhalla::midgard;
			FastInvSqrt(7.7f);
			return 0;]])],
			ax_cv_valhalla_midgard=yes, ax_cv_valhalla_midgard=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_midgard" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_MIDGARD,,[define if the valhalla::midgard library is available])
	else
		AC_MSG_ERROR(Could not link against valhalla_midgard!)
	fi

	AC_CHECK_LIB(valhalla_midgard, exit, [VALHALLA_MIDGARD_LIB="-lvalhalla_midgard"; AC_SUBST(VALHALLA_MIDGARD_LIB) link_midgard="yes"; break], [link_midgard="no"])

	if test "x$link_midgard" = "xyes"; then
		AC_SUBST(VALHALLA_MIDGARD_CPPFLAGS)
		AC_SUBST(VALHALLA_MIDGARD_LDFLAGS)
		VALHALLA_MIDGARD_LIBS="-lvalhalla_midgard"
		AC_SUBST(VALHALLA_MIDGARD_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_midgard!)
	fi
])
