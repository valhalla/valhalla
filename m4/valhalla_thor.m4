AC_DEFUN([CHECK_VALHALLA_THOR],
[
	AC_ARG_WITH([valhalla-thor],
		[AS_HELP_STRING([--with-valhalla-thor@<:@=ARG@:>@],
			[use the system valhalla-thor (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_THOR_CPPFLAGS=""
			VALHALLA_THOR_LDFLAGS=""
		else
			VALHALLA_THOR_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_THOR_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_THOR_CPPFLAGS=""
		VALHALLA_THOR_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_CPPFLAGS $VALHALLA_SIF_CPPFLAGS $VALHALLA_ODIN_CPPFLAGS $VALHALLA_THOR_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_LDFLAGS $VALHALLA_BALDR_LDFLAGS $VALHALLA_SIF_LDFLAGS $VALHALLA_ODIN_CPPFLAGS $VALHALLA_THOR_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::thor library is available, ax_cv_valhalla_thor,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/thor/adjacencylist.h>]],
			[[using namespace valhalla::thor;
			AdjacencyList a(1.f, 5.f, 7.f);]])],
			ax_cv_valhalla_thor=yes, ax_cv_valhalla_thor=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_thor" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_THOR,,[define if the valhalla::thor library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_thor!)
	fi

	AC_CHECK_LIB(valhalla_thor, exit, [VALHALLA_THOR_LIB="-lvalhalla_thor"; AC_SUBST(VALHALLA_THOR_LIB) link_thor="yes"; break], [link_thor="no"])

	if test "x$link_thor" = "xyes"; then
		AC_SUBST(VALHALLA_THOR_CPPFLAGS)
		AC_SUBST(VALHALLA_THOR_LDFLAGS)
		VALHALLA_THOR_LIBS="-lvalhalla_thor"
		AC_SUBST(VALHALLA_THOR_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_thor!)
	fi
])
