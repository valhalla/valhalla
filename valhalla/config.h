/* Minimal valhalla/config.h template for minimal CMake build configuration */

#include <valhalla/valhalla.h>

#define VALHALLA_STRINGIZE_NX(A) #A
#define VALHALLA_STRINGIZE(A) VALHALLA_STRINGIZE_NX(A)

/* Version number of package */

// clang-format off
#define VALHALLA_VERSION VALHALLA_STRINGIZE(VALHALLA_VERSION_MAJOR) "." VALHALLA_STRINGIZE(VALHALLA_VERSION_MINOR) "." VALHALLA_STRINGIZE(VALHALLA_VERSION_PATCH)
// clang-format on

/* Name of package */
#define PACKAGE "valhalla-" VALHALLA_VERSION

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT "https://github.com/valhalla/valhalla/issues"

/* Define to the full name of this package. */
#define PACKAGE_NAME "valhalla"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "valhalla " VALHALLA_VERSION

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "valhalla-" VALHALLA_VERSION

/* Define to the home page for this package. */
#define PACKAGE_URL "https://github.com/valhalla/valhalla"

/* Define to the version of this package. */
#define PACKAGE_VERSION VALHALLA_VERSION
