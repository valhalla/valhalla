AC_DEFUN([CHECK_VALHALLA_LOKI],
[
	AC_ARG_WITH([valhalla-loki],
		[AS_HELP_STRING([--with-valhalla-loki@<:@=ARG@:>@],
			[use the system valhalla-loki (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_LOKI_CPPFLAGS=""
			VALHALLA_LOKI_LDFLAGS=""
		else
			VALHALLA_LOKI_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_LOKI_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_LOKI_CPPFLAGS=""
		VALHALLA_LOKI_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_CPPFLAGS $VALHALLA_SIF_CPPFLAGS $VALHALLA_MJOLNIR_CPPFLAGS $VALHALLA_LOKI_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_LDFLAGS $VALHALLA_BALDR_LDFLAGS $VALHALLA_SIF_LDFLAGS $VALHALLA_MJOLNIR_LDFLAGS $VALHALLA_LOKI_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::loki library is available, ax_cv_valhalla_loki,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/loki/search.h>]],
			[[using namespace valhalla::loki;
			auto f = valhalla::loki::PassThroughFilter;]])],
			ax_cv_valhalla_loki=yes, ax_cv_valhalla_loki=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_loki" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_LOKI,,[define if the valhalla::loki library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_loki!)
	fi

	AC_CHECK_LIB(valhalla_loki, exit, [VALHALLA_LOKI_LIB="-lvalhalla_loki"; AC_SUBST(VALHALLA_LOKI_LIB) link_loki="yes"; break], [link_loki="no"])

	if test "x$link_loki" = "xyes"; then
		AC_SUBST(VALHALLA_LOKI_CPPFLAGS)
		AC_SUBST(VALHALLA_LOKI_LDFLAGS)
		VALHALLA_LOKI_LIBS="-lvalhalla_loki"
		AC_SUBST(VALHALLA_LOKI_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_loki!)
	fi
])
