#pragma once

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__)
  #define AQUILA_EXPORTS __declspec(dllexport)
  #define TEMPLATE_EXTERN extern
#elif defined __GNUC__ && __GNUC__ >= 4
  #define AQUILA_EXPORTS __attribute__ ((visibility ("default")))
  #define TEMPLATE_EXTERN 
#else
  #define AQUILA_EXPORTS
  #define TEMPLATE_EXTERN 
#endif

#ifdef _MSC_VER
  #ifndef Aquila_EXPORTS
    #ifdef _DEBUG
      #pragma comment(lib, "Aquilad.lib")
      #pragma comment(lib, "pplx_2_7d.lib")
    #else
      #pragma comment(lib, "Aquila.lib")
      #pragma comment(lib, "pplx_2_7.lib")
    #endif
  #endif
#endif

#ifndef _MSC_VER
  #include "RuntimeLinkLibrary.h"
  #ifdef NDEBUG
    RUNTIME_COMPILER_LINKLIBRARY("-lAquila")
  #else
    RUNTIME_COMPILER_LINKLIBRARY("-lAquilad")
  #endif
#endif

#ifndef BUILD_TYPE
#ifdef _DEBUG
#define BUILD_TYPE 0
#else
#define BUILD_TYPE 1
#endif
#endif
