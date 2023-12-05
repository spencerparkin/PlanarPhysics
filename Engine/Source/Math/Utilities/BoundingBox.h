#pragma once

#include "Common.h"
#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class BoundingBox
	{
	public:
		BoundingBox();
		BoundingBox(const BoundingBox& box);
		BoundingBox(const Vector2D& min, const Vector2D& max);
		virtual ~BoundingBox();

		enum class MatchMethod
		{
			EXPAND,
			CONTRACT
		};

		void operator=(const BoundingBox& box);
		bool IsValid() const;
		double Width() const;
		double Height() const;
		double Area() const;
		double AspectRatio() const;
		bool ContainsPoint(const Vector2D& point, double thickness = PLNR_PHY_EPSILON) const;
		bool PointOnEdge(const Vector2D& point, double thickness = PLNR_PHY_EPSILON) const;
		bool PointIsInterior(const Vector2D& point, double thickness = PLNR_PHY_EPSILON) const;
		void Merge(const BoundingBox& boxA, const BoundingBox& boxB);
		bool Intersect(const BoundingBox& boxA, const BoundingBox& boxB);
		void MatchAspectRatio(double aspectRatio, MatchMethod matchMethod);
		Vector2D PointToUVs(const Vector2D& point) const;
		Vector2D PointFromUVs(const Vector2D& uvs) const;
		void ExpandToIncludePoint(const Vector2D& point);
		Vector2D Center() const;

		Vector2D min, max;
	};
}