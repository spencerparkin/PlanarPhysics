#pragma once

#if defined PLANAR_PHYSICS_EXPORT
#	define PLANAR_PHYSICS_API			__declspec(dllexport)
#elif defined PLANAR_PHYSICS_IMPORT
#	define PLANAR_PHYSICS_API			__declspec(dllimport)
#else
#	define PLANAR_PHYSICS_API
#endif

#include <math.h>