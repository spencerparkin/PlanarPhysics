#include "Random.h"

using namespace PlanarPhysics;

/*static*/ double Random::Number(double min, double max)
{
	double alpha = double(rand()) / double(RAND_MAX);
	return min + alpha * (max - min);
}

/*static*/ int Random::Integer(int min, int max)
{
	int result = (int)::round(Number(double(min) - 0.5, double(max) + 0.5));
	return PLNR_PHY_CLAMP(result, min, max);
}