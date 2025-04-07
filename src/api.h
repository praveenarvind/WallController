#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define WallController_DLLIMPORT __declspec(dllimport)
#  define WallController_DLLEXPORT __declspec(dllexport)
#  define WallController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define WallController_DLLIMPORT __attribute__((visibility("default")))
#    define WallController_DLLEXPORT __attribute__((visibility("default")))
#    define WallController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define WallController_DLLIMPORT
#    define WallController_DLLEXPORT
#    define WallController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef WallController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define WallController_DLLAPI
#  define WallController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef WallController_EXPORTS
#    define WallController_DLLAPI WallController_DLLEXPORT
#  else
#    define WallController_DLLAPI WallController_DLLIMPORT
#  endif // WallController_EXPORTS
#  define WallController_LOCAL WallController_DLLLOCAL
#endif // WallController_STATIC