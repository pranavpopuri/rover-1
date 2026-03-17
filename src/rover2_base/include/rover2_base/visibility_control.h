#ifndef ROVER2_BASE__VISIBILITY_CONTROL_H_
#define ROVER2_BASE__VISIBILITY_CONTROL_H_

// Visibility control macros for shared library exports

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROVER2_BASE_EXPORT __attribute__ ((dllexport))
    #define ROVER2_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROVER2_BASE_EXPORT __declspec(dllexport)
    #define ROVER2_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROVER2_BASE_BUILDING_DLL
    #define ROVER2_BASE_PUBLIC ROVER2_BASE_EXPORT
  #else
    #define ROVER2_BASE_PUBLIC ROVER2_BASE_IMPORT
  #endif
  #define ROVER2_BASE_PUBLIC_TYPE ROVER2_BASE_PUBLIC
  #define ROVER2_BASE_LOCAL
#else
  #define ROVER2_BASE_EXPORT __attribute__ ((visibility("default")))
  #define ROVER2_BASE_IMPORT
  #if __GNUC__ >= 4
    #define ROVER2_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define ROVER2_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROVER2_BASE_PUBLIC
    #define ROVER2_BASE_LOCAL
  #endif
  #define ROVER2_BASE_PUBLIC_TYPE
#endif

#endif  // ROVER2_BASE__VISIBILITY_CONTROL_H_
