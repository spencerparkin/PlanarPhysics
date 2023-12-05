#pragma once

#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API LineSegment
	{
	public:
		LineSegment();
		LineSegment(const LineSegment& lineSegment);
		LineSegment(const Vector2D& pointA, const Vector2D& pointB);
		virtual ~LineSegment();

		void operator=(const LineSegment& lineSegment);

		double DistanceTo(const Vector2D& point) const;
		Vector2D NearestPoint(const Vector2D& point) const;
		double Length() const;

		Vector2D vertexA, vertexB;
	};
}