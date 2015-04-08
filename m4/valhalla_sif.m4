AC_DEFUN([CHECK_VALHALLA_SIF],
[
	AC_ARG_WITH([valhalla-sif],
		[AS_HELP_STRING([--with-valhalla-sif@<:@=ARG@:>@],
			[use the system valhalla-sif (ARG=yes), or from a specified location (ARG=<path>).])],
	[
		if test "$withval" = "yes"; then
			VALHALLA_SIF_CPPFLAGS=""
			VALHALLA_SIF_LDFLAGS=""
		else
			VALHALLA_SIF_CPPFLAGS="-I$withval/include -I$withval"
			VALHALLA_SIF_LDFLAGS="-L$withval/lib"
		fi
	],
	[
		VALHALLA_SIF_CPPFLAGS=""
		VALHALLA_SIF_LDFLAGS=""
	])

	CPPFLAGS_SAVED="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS $VALHALLA_MIDGARD_CPPFLAGS $VALHALLA_BALDR_CPPFLAGS $VALHALLA_SIF_CPPFLAGS"
	export CPPFLAGS
	LDFLAGS_SAVED="$LDFLAGS"
	LDFLAGS="$LDFLAGS $VALHALLA_MIDGARD_LDFLAGS $VALHALLA_BALDR_LDFLAGS $VALHALLA_SIF_LDFLAGS"
	export LDFLAGS

	AC_REQUIRE([AC_PROG_CC])

        AC_CACHE_CHECK(whether the valhalla::sif library is available, ax_cv_valhalla_sif,
        	[AC_LANG_PUSH([C++])
		AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <valhalla/sif/costfactory.h>]],
			[[using namespace valhalla::sif;
			CostFactory<DynamicCost> f;]])],
			ax_cv_valhalla_sif=yes, ax_cv_valhalla_sif=no)
		AC_LANG_POP([C++])
	])

	if test "x$ax_cv_valhalla_sif" = "xyes"; then
		AC_DEFINE(HAVE_VALHALLA_SIF,,[define if the valhalla::sif library is available])
	else
		AC_MSG_ERROR(Could not find valhalla_sif!)
	fi

	AC_CHECK_LIB(valhalla_sif, exit, [VALHALLA_SIF_LIB="-lvalhalla_sif"; AC_SUBST(VALHALLA_SIF_LIB) link_sif="yes"; break], [link_sif="no"])

	if test "x$link_sif" = "xyes"; then
		AC_SUBST(VALHALLA_SIF_CPPFLAGS)
		AC_SUBST(VALHALLA_SIF_LDFLAGS)
		VALHALLA_SIF_LIBS="-lvalhalla_sif"
		AC_SUBST(VALHALLA_SIF_LIBS)
		CPPFLAGS="$CPPFLAGS_SAVED"
		export CPPFLAGS
		LDFLAGS="$LDFLAGS_SAVED"
		export LDFLAGS
	else
		AC_MSG_ERROR(Could not link against valhalla_sif!)
	fi
])
