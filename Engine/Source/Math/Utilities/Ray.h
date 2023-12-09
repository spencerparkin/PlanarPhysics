#pragma once

#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class LineSegment;
	class ConvexPolygon;
	class Line;

	class PLANAR_PHYSICS_API Ray
	{
	public:
		Ray();
		Ray(const Ray& ray);
		Ray(const Vector2D& origin, const Vector2D& direction);
		virtual ~Ray();

		Vector2D CalculateRayPoint(double lambda) const;
		bool CastAgainst(const LineSegment& lineSegment, double& lambda, double eps = PLNR_PHY_EPSILON) const;
		bool CastAgainst(const ConvexPolygon& convexPolygon, double& lambda, Vector2D* hitNormal = nullptr, double eps = PLNR_PHY_EPSILON) const;
		bool CastAgainst(const Line& line, double& lambda) const;

		Vector2D origin;
		Vector2D direction;
	};
}