#pragma once

#include "Common.h"

namespace PlanarPhysics
{
	class Vector2D;
	class PScalar2D;

	class PLANAR_PHYSICS_API Rotor2D
	{
	public:
		Rotor2D();
		Rotor2D(const Rotor2D& rotor);
		Rotor2D(double w, double z);
		virtual ~Rotor2D();

		void operator=(const Rotor2D& rotor);
		Vector2D operator*(const Vector2D& vector) const;

		Rotor2D Reversed() const;
		Rotor2D Inverted() const;
		Rotor2D Normalized() const;
		double Magnitude() const;
		void Interpolate(const Rotor2D& rotorA, const Rotor2D& rotorB, double alpha);
		PScalar2D Log() const;

		double w, z;
	};

	PLANAR_PHYSICS_API Rotor2D operator*(const Rotor2D& rotor, double scalar);
	PLANAR_PHYSICS_API Rotor2D operator/(const Rotor2D& rotor, double scalar);
	PLANAR_PHYSICS_API Rotor2D operator*(double scalar, const Rotor2D& rotor);
	PLANAR_PHYSICS_API Rotor2D operator*(const Rotor2D& rotor1, const Rotor2D& rotor2);
	PLANAR_PHYSICS_API Rotor2D operator/(const Rotor2D& rotor1, const Rotor2D& rotor2);
	PLANAR_PHYSICS_API Rotor2D operator|(const Rotor2D& rotor1, const Rotor2D& rotor2);
	PLANAR_PHYSICS_API Rotor2D operator^(const Rotor2D& rotor1, const Rotor2D& rotor2);
}