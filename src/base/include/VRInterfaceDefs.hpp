//$Id$
//------------------------------------------------------------------------------
//                            VRVDefs
//------------------------------------------------------------------------------
// GMAT: General Mission Analysis Tool.
/**
 * DLL interface definitions for the VRV plugin
 *
 * Replace SAMPLE_EXPORTS and SAMPLE_API with names that match your plugin
 * library's functionality.
 */
//------------------------------------------------------------------------------

//so VRV must become ORBITSUBCRIBER or something -- what?

#ifndef VRInterfaceDefs_hpp
#define VRInterfaceDefs_hpp

#include "gmatdefs.hpp"

#ifdef _WIN32  // Windows
   #ifdef _MSC_VER  // Microsoft Visual C++

      #define WIN32_LEAN_AND_MEAN  // Exclude rarely-used stuff from Windows headers
      // Windows Header File entry point:
      #include <windows.h>

      #define  _USE_MATH_DEFINES  // makes Msoft define pi

   #endif  // End Microsoft C++ specific block

   #ifdef _DYNAMICLINK  // Only used for Visual C++ Windows DLLs
      #ifdef VRInterface_EXPORTS
         #define VRInterface_API __declspec(dllexport)
      #else
         #define VRInterface_API __declspec(dllimport)
      #endif

      // Provide the storage class specifier (extern for an .exe file, null
      // for DLL) and the __declspec specifier (dllimport for .an .exe file,
      // dllexport for DLL).
      // You must define EXP_STL when compiling the DLL.
      // You can now use this header file in both the .exe file and DLL - a
      // much safer means of using common declarations than two different
      // header files.
      #ifdef EXP_STL
      #    define DECLSPECIFIER __declspec(dllexport)
      #    define EXPIMP_TEMPLATE
      #else
      #    define DECLSPECIFIER __declspec(dllimport)
      #    define EXPIMP_TEMPLATE extern
      #endif

      #define EXPORT_TEMPLATES
   #endif
#endif //  End of OS nits

#ifndef VRInterface_API
   #define VRInterface_API
#endif

// custom definitions here

enum { Q1, Q2, Q3, Q4 };

typedef std::vector<std::vector<Real>> RealArray2D;

#endif /* VRVDefs_hpp */