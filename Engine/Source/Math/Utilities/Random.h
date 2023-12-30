#pragma once

#include "Common.h"
#include "Math/GeometricAlgebra/Vector2D.h"
#include "Math/Utilities/BoundingBox.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API Random
	{
	public:
		static void Seed(int seed);
		static double Number(double min, double max);
		static int Integer(int min, int max);
		static void ShuffleArray(std::vector<int>& intArray);
		static PlanarPhysics::Vector2D Vector(const PlanarPhysics::Vector2D& minVector, const PlanarPhysics::Vector2D& maxVector);
		static PlanarPhysics::Vector2D Vector(const PlanarPhysics::BoundingBox& box);
		static PlanarPhysics::Vector2D Vector(double minRadius, double maxRadius);
		static PlanarPhysics::Vector2D Vector(double minTheta, double maxTheta, double minRadius, double maxRadius);
	};
}