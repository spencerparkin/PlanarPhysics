#pragma once

#if defined PLANAR_PHYSICS_EXPORT
#	define PLANAR_PHYSICS_API			__declspec(dllexport)
#elif defined PLANAR_PHYSICS_IMPORT
#	define PLANAR_PHYSICS_API			__declspec(dllimport)
#else
#	define PLANAR_PHYSICS_API
#endif

#define PLNR_PHY_EPSILON			1e-6
#define PLNR_PHY_PI					3.1415926536
#define PLNR_PHY_MIN(x, y)			((x) < (y) ? (x) : (y))
#define PLNR_PHY_MAX(x, y)			((x) > (y) ? (x) : (y))
#define PLNR_PHY_SIGN(x)			((x) < 0.0 ? -1.0 : 1.0)
#define PLNR_PHY_ABS(x)				((x) < 0.0 ? -(x) : (x))
#define PLNR_PHY_CLAMP(x, a, b)		PLNR_PHY_MIN(PLNR_PHY_MAX(x, a), b)

#include <math.h>
#include <time.h>
#include <stdint.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <list>
#include <string>
#include <functional>
#include <assert.h>
#include <memory>