#ifndef DUMMY_ROBOT_HARDWARE__VISIBILITY_CONTROL_H_
#define DUMMY_ROBOT_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DUMMY_ROBOT_HARDWARE_EXPORT __attribute__((dllexport))
#define DUMMY_ROBOT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define DUMMY_ROBOT_HARDWARE_EXPORT __declspec(dllexport)
#define DUMMY_ROBOT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef DUMMY_ROBOT_HARDWARE_BUILDING_DLL
#define DUMMY_ROBOT_HARDWARE_PUBLIC DUMMY_ROBOT_HARDWARE_EXPORT
#else
#define DUMMY_ROBOT_HARDWARE_PUBLIC DUMMY_ROBOT_HARDWARE_IMPORT
#endif
#define DUMMY_ROBOT_HARDWARE_PUBLIC_TYPE DUMMY_ROBOT_HARDWARE_PUBLIC
#define DUMMY_ROBOT_HARDWARE_LOCAL
#else
#define DUMMY_ROBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define DUMMY_ROBOT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define DUMMY_ROBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define DUMMY_ROBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define DUMMY_ROBOT_HARDWARE_PUBLIC
#define DUMMY_ROBOT_HARDWARE_LOCAL
#endif
#define DUMMY_ROBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // DUMMY_ROBOT_HARDWARE__VISIBILITY_CONTROL_H_