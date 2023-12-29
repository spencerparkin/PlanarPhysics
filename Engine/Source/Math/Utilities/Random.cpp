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