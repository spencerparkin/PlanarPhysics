#pragma once

#include "Common.h"
#include "Math/GeometricAlgebra/Vector2D.h"
#include "Math/GeometricAlgebra/Rotor2D.h"

namespace PlanarPhysics
{
	class LineSegment;

	class PLANAR_PHYSICS_API Transform
	{
	public:
		Transform();
		Transform(const Transform& transform);
		virtual ~Transform();

		void operator=(const Transform& transform);
		
		void Identity();
		Vector2D TransformVector(const Vector2D& vector) const;
		Vector2D TransformPoint(const Vector2D& vector) const;
		LineSegment TransformLineSegment(const LineSegment& lineSeg) const;
		void Interpolate(const Transform& transformA, const Transform& transformB, double alpha);

		Vector2D translation;
		Rotor2D rotation;
		double scale;
	};

	Transform operator*(const Transform& transformA, const Transform& transformB);
}