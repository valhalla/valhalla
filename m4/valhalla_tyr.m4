AC_DEFUN([CHECK_VALHALLA_TYR],
[
	AC_ARG_WITH([valhalla-tyr],
		[AS_HELP_STRING([--with-valhalla-tyr@<:@=ARG@:>@],
			[use the system valhalla-tyr (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_TYR_CPPFLAGS=""
			VALHALLA_TYR_LDFLAGS=""
		else
			VALHALLA_TYR_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_TYR_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_TYR_CPPFLAGS=""
		VALHALLA_TYR_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_CPPFLAGS $VALHALLA_TYR_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_LDFLAGS $VALHALLA_BALDR_LDFLAGS $VALHALLA_TYR_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::tyr library is available, ax_cv_valhalla_tyr,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/tyr/service.h>]],
			[[using namespace valhalla::tyr;
			return run_service == nullptr;]])],
			ax_cv_valhalla_tyr=yes, ax_cv_valhalla_tyr=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_tyr" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_TYR,,[define if the valhalla::tyr library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_tyr!)
	fi

	AC_CHECK_LIB(valhalla_tyr, exit, [VALHALLA_TYR_LIB="-lvalhalla_tyr"; AC_SUBST(VALHALLA_TYR_LIB) link_tyr="yes"; break], [link_tyr="no"])

	if test "x$link_tyr" = "xyes"; then
		AC_SUBST(VALHALLA_TYR_CPPFLAGS)
		AC_SUBST(VALHALLA_TYR_LDFLAGS)
		VALHALLA_TYR_LIBS="-lvalhalla_tyr"
		AC_SUBST(VALHALLA_TYR_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_tyr!)
	fi
])
