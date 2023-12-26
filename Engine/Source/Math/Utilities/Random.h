#pragma once

#include "Common.h"

namespace PlanarPhysics
{
	class Random
	{
	public:
		static double Number(double min, double max);
		static int Integer(int min, int max);
	};
}