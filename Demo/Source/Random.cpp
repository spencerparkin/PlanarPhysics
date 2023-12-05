#include "Random.h"
#include <stdlib.h>
#include <math.h>

using namespace PlanarPhysics;

/*static*/ void Random::Seed(int seed)
{
	srand(seed);
}

/*static*/ int Random::Integer(int minValue, int maxValue)
{
	int randomValue = (int)::round(Random::Number(double(minValue) - 0.5, double(maxValue) + 0.5));
	randomValue = PLNR_PHY_MAX(randomValue, minValue);
	randomValue = PLNR_PHY_MIN(randomValue, maxValue);
	return randomValue;
}

/*static*/ double Random::Number(double minValue, double maxValue)
{
	double lerp = double(rand()) / double(RAND_MAX);
	double randomValue = minValue + lerp * (maxValue - minValue);
	return randomValue;
}

/*static*/ Vector2D Random::Vector(const Vector2D& minVector, const Vector2D& maxVector)
{
	double lerp = double(rand()) / double(RAND_MAX);
	Vector2D randomVector = minVector + lerp * (maxVector - minVector);
	return randomVector;
}

/*static*/ Vector2D Random::Vector(const BoundingBox& box)
{
	return Random::Vector(box.min, box.max);
}

/*static*/ Vector2D Random::Vector(double minRadius, double maxRadius)
{
	return Vector(0.0, 2.0 * PLNR_PHY_PI, minRadius, maxRadius);
}

/*static*/ Vector2D Random::Vector(double minTheta, double maxTheta, double minRadius, double maxRadius)
{
	double theta = Random::Number(minTheta, maxTheta);
	double radius = Random::Number(minRadius, maxRadius);
	Vector2D randomVector(radius * ::cos(theta), radius * ::sin(theta));
	return randomVector;
}