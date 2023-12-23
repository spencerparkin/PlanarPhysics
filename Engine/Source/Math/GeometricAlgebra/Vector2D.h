#pragma once

#include "Common.h"

namespace PlanarPhysics
{
	class Rotor2D;
	class PScalar2D;

	class PLANAR_PHYSICS_API Vector2D
	{
	public:
		Vector2D();
		Vector2D(const Vector2D& vector);
		Vector2D(double x, double y);
		virtual ~Vector2D();

		void operator=(const Vector2D& vector);
		void operator*=(double scalar);
		void operator/=(double scalar);
		void operator+=(const Vector2D& vector);
		void operator-=(const Vector2D& vector);
		Vector2D operator-() const;
		bool operator==(const Vector2D& vector) const;
		bool operator!=(const Vector2D& vector) const;

		Vector2D Inverted() const;
		Vector2D Normalized() const;
		double Magnitude() const;
		bool Normalize(double* magnitude = nullptr);
		bool IsPoint(const Vector2D& point, double tolerance = PLNR_PHY_EPSILON) const;

		double x, y;
	};

	PLANAR_PHYSICS_API Vector2D operator*(const Vector2D& vector, double scalar);
	PLANAR_PHYSICS_API Vector2D operator/(const Vector2D& vector, double scalar);
	PLANAR_PHYSICS_API Vector2D operator*(double scalar, const Vector2D& vector);
	PLANAR_PHYSICS_API Vector2D operator+(const Vector2D& vector1, const Vector2D& vector2);
	PLANAR_PHYSICS_API Vector2D operator-(const Vector2D& vector1, const Vector2D& vector2);
	PLANAR_PHYSICS_API Rotor2D operator*(const Vector2D& vector1, const Vector2D& vector2);
	PLANAR_PHYSICS_API Rotor2D operator/(const Vector2D& vector1, const Vector2D& vector2);
	PLANAR_PHYSICS_API double operator|(const Vector2D& vector1, const Vector2D& vector2);
	PLANAR_PHYSICS_API PScalar2D operator^(const Vector2D& vector1, const Vector2D& vector2);
	PLANAR_PHYSICS_API Vector2D operator*(const Vector2D& vector, const Rotor2D& rotor);
	PLANAR_PHYSICS_API Vector2D operator*(const Vector2D& vector, const PScalar2D& pscalar);
}