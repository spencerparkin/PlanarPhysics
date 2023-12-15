#pragma once

#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class BoundingBox;
	class Line;
	class LineSegment;

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
		void CalculateIntersection(const ConvexPolygon& polygonA, const ConvexPolygon& polygonB);
		void UpdateEdgeLineCacheIfNeeded() const;
		int NearestVertexToPoint(const Vector2D& point, double& minSquareDistance) const;
		void NearestEdgeToPoint(const Vector2D& point, LineSegment& nearestEdgeSegment) const;

		const Vector2D& operator[](int i) const;
		Vector2D& operator[](int i);

		const std::vector<Vector2D>& GetVertexArray() const;
		std::vector<Vector2D>& GetVertexArray();

		const std::vector<Line>& GetEdgeLineArray() const;

		mutable bool edgeLineCacheArrayValid;

	private:
		std::vector<Vector2D>* vertexArray;

		mutable std::vector<Line>* edgeLineCacheArray;
	};
}