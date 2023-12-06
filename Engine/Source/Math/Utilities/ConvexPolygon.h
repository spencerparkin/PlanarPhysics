#pragma once

#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class BoundingBox;
	class Line;

	class PLANAR_PHYSICS_API ConvexPolygon
	{
	public:
		ConvexPolygon();
		virtual ~ConvexPolygon();

		bool CalcConvexHull(const std::vector<Vector2D>& pointArray);
		void Clear();
		void SetVertexCount(int count);
		int GetVertexCount() const;
		bool IsValid() const;
		bool CalcBoundingBox(BoundingBox& box) const;
		bool ContainsPoint(const Vector2D& point, double thickness = PLNR_PHY_EPSILON) const;
		bool AllPointsOnOrBehindLine(const Line& line) const;

		const Vector2D& operator[](int i) const;
		Vector2D& operator[](int i);

		const std::vector<Vector2D>& GetVertexArray() const;
		std::vector<Vector2D>& GetVertexArray();

	private:
		std::vector<Vector2D>* vertexArray;
	};
}