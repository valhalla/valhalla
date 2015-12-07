AC_DEFUN([CHECK_VALHALLA_MJOLNIR],
[
	AC_ARG_WITH([valhalla-mjolnir],
		[AS_HELP_STRING([--with-valhalla-mjolnir@<:@=ARG@:>@],
			[use the system valhalla-mjolnir (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_MJOLNIR_CPPFLAGS=""
			VALHALLA_MJOLNIR_LDFLAGS=""
		else
			VALHALLA_MJOLNIR_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_MJOLNIR_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_MJOLNIR_CPPFLAGS=""
		VALHALLA_MJOLNIR_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_CPPFLAGS $VALHALLA_MJOLNIR_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_LDFLAGS $VALHALLA_BALDR_LDFLAGS $VALHALLA_MJOLNIR_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::mjolnir library is available, ax_cv_valhalla_mjolnir,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/mjolnir/osmway.h>]],
			[[using namespace valhalla::mjolnir;
			OSMWay w;]])],
			ax_cv_valhalla_mjolnir=yes, ax_cv_valhalla_mjolnir=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_mjolnir" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_MJOLNIR,,[define if the valhalla::mjolnir library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_mjolnir!)
	fi

	AC_CHECK_LIB(valhalla_mjolnir, exit, [VALHALLA_MJOLNIR_LIB="-lvalhalla_mjolnir"; AC_SUBST(VALHALLA_MJOLNIR_LIB) link_mjolnir="yes"; break], [link_mjolnir="no"])

	if test "x$link_mjolnir" = "xyes"; then
		AC_SUBST(VALHALLA_MJOLNIR_CPPFLAGS)
		AC_SUBST(VALHALLA_MJOLNIR_LDFLAGS)
		VALHALLA_MJOLNIR_LIBS="-lvalhalla_mjolnir"
		AC_SUBST(VALHALLA_MJOLNIR_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_mjolnir!)
	fi
])
