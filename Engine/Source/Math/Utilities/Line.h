#pragma once

#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class LineSegment;

	class PLANAR_PHYSICS_API Line
	{
	public:
		Line();
		Line(const Line& line);
		Line(const Vector2D& center, const Vector2D& normal);
		Line(const LineSegment& lineSegment);
		virtual ~Line();

		enum class Side
		{
			FRONT,
			BACK,
			NEITHER
		};

		Vector2D Center() const;
		double SignedDistanceTo(const Vector2D& point) const;
		Side WhichSide(const Vector2D& point, double thickness = PLNR_PHY_EPSILON) const;

		Vector2D normal;
		double D;
	};
}