#include "Random.h"

using namespace PlanarPhysics;

/*static*/ void Random::Seed(int seed)
{
	::srand(seed);
}

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

/*static*/ void Random::ShuffleArray(std::vector<int>& intArray)
{
	std::vector<int> shuffledArray;

	while (intArray.size() > 0)
	{
		int i = Random::Integer(0, int(intArray.size()) - 1);
		int j = intArray[i];
		intArray[i] = intArray[intArray.size() - 1];
		intArray.pop_back();
		shuffledArray.push_back(j);
	}

	for (int i : shuffledArray)
		intArray.push_back(i);
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