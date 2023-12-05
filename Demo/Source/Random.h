#pragma once

#include "Math/GeometricAlgebra/Vector2D.h"
#include "Math/Utilities/BoundingBox.h"

class Random
{
public:
	static void Seed(int seed);
	static int Integer(int minValue, int maxValue);
	static double Number(double minValue, double maxValue);
	static PlanarPhysics::Vector2D Vector(const PlanarPhysics::Vector2D& minVector, const PlanarPhysics::Vector2D& maxVector);
	static PlanarPhysics::Vector2D Vector(const PlanarPhysics::BoundingBox& box);
	static PlanarPhysics::Vector2D Vector(double minRadius, double maxRadius);
	static PlanarPhysics::Vector2D Vector(double minTheta, double maxTheta, double minRadius, double maxRadius);
};