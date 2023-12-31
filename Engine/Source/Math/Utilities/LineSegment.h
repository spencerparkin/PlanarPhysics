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
		
		double SquareDistanceTo(const Vector2D& point) const;
		double DistanceTo(const Vector2D& point) const;
		Vector2D NearestPoint(const Vector2D& point) const;
		double Length() const;
		double CalculateLineLerpAlpha(const Vector2D& linePoint) const;
		bool CalcIntersectionPoint(const LineSegment& lineSegment, Vector2D& intersectionPoint) const;
		bool SameGeometryAs(const LineSegment& lineSegment, double tolerance = PLNR_PHY_EPSILON) const;
		bool Merge(const LineSegment& lineSegA, const LineSegment& lineSegB, double tolerance = PLNR_PHY_EPSILON);
		bool ContainsPoint(const Vector2D& point, double tolerance = PLNR_PHY_EPSILON) const;
		Vector2D MidPoint() const;

		Vector2D& operator[](int i);
		const Vector2D& operator[](int i) const;

		Vector2D vertexA, vertexB;
	};
}