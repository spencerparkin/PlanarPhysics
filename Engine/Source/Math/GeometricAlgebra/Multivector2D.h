#pragma once

#include "Common.h"

namespace PlanarPhysics
{
	class PScalar2D;
	class Vector2D;
	class Rotor2D;

	class PLANAR_PHYSICS_API Multivector2D
	{
	public:
		Multivector2D();
		Multivector2D(const Multivector2D& multivector);
		Multivector2D(double w, double x, double y, double z);
		Multivector2D(double scalar);
		Multivector2D(const Vector2D& vector);
		Multivector2D(const Rotor2D& rotor);
		Multivector2D(const PScalar2D& pscalar);
		virtual ~Multivector2D();

		void operator=(const Multivector2D& multivector);
		void operator=(double scalar);
		void operator=(const Vector2D& vector);
		void operator=(const Rotor2D& rotor);
		void operator=(const PScalar2D& pscalar);

		Multivector2D Reversed() const;
		Multivector2D Inverted() const;

		double w, x, y, z;
	};

	PLANAR_PHYSICS_API Multivector2D operator*(const Multivector2D& multivector, double scalar);
	PLANAR_PHYSICS_API Multivector2D operator/(const Multivector2D& multivector, double scalar);
	PLANAR_PHYSICS_API Multivector2D operator*(double scalar, const Multivector2D& multivector);
	PLANAR_PHYSICS_API Multivector2D operator+(const Multivector2D& multivector1, const Multivector2D& multivector2);
	PLANAR_PHYSICS_API Multivector2D operator-(const Multivector2D& multivector1, const Multivector2D& multivector2);
	PLANAR_PHYSICS_API Multivector2D operator*(const Multivector2D& multivector1, const Multivector2D& multivector2);
	PLANAR_PHYSICS_API Multivector2D operator/(const Multivector2D& multivector1, const Multivector2D& multivector2);
	PLANAR_PHYSICS_API Multivector2D operator|(const Multivector2D& multivector1, const Multivector2D& multivector2);
	PLANAR_PHYSICS_API Multivector2D operator^(const Multivector2D& multivector1, const Multivector2D& multivector2);
}