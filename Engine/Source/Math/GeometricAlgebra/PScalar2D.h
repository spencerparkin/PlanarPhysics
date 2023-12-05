#pragma once

#include "Common.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API PScalar2D
	{
	public:
		PScalar2D();
		PScalar2D(const PScalar2D& pscalar);
		PScalar2D(double z);
		virtual ~PScalar2D();

		void operator=(const PScalar2D& pscalar);

		PScalar2D Reversed() const;
		PScalar2D Inverted() const;
		PScalar2D Magnitude() const;

		double z;
	};

	PLANAR_PHYSICS_API PScalar2D operator*(const PScalar2D& pscalar, double scalar);
	PLANAR_PHYSICS_API PScalar2D operator/(const PScalar2D& pscalar, double scalar);
	PLANAR_PHYSICS_API PScalar2D operator*(double scalar, const PScalar2D& pscalar);
	PLANAR_PHYSICS_API PScalar2D operator+(const PScalar2D& pscalar1, const PScalar2D& pscalar2);
	PLANAR_PHYSICS_API PScalar2D operator-(const PScalar2D& pscalar1, const PScalar2D& pscalar2);
	PLANAR_PHYSICS_API double operator*(const PScalar2D& pscalar1, const PScalar2D& pscalar2);
	PLANAR_PHYSICS_API double operator/(const PScalar2D& pscalar1, const PScalar2D& pscalar2);
	PLANAR_PHYSICS_API double operator|(const PScalar2D& pscalar1, const PScalar2D& pscalar2);
	PLANAR_PHYSICS_API double operator^(const PScalar2D& pscalar1, const PScalar2D& pscalar2);
}