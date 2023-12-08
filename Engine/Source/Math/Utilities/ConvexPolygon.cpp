#include "ConvexPolygon.h"
#include "BoundingBox.h"
#include "Math/GeometricAlgebra/PScalar2D.h"
#include "Math/Utilities/Line.h"
#include "Math/Utilities/LineSegment.h"

using namespace PlanarPhysics;

ConvexPolygon::ConvexPolygon()
{
	this->vertexArray = new std::vector<Vector2D>();
}

/*virtual*/ ConvexPolygon::~ConvexPolygon()
{
	delete this->vertexArray;
}

bool ConvexPolygon::CalcConvexHull(const std::vector<Vector2D>& pointArray)
{
	if (pointArray.size() < 3)
		return false;

	this->vertexArray->clear();

	// Find the initial convex hull as a triangle.
	bool foundInitialTriangle = false;
	for (int i = 0; i < (signed)pointArray.size() && !foundInitialTriangle; i++)
	{
		for (int j = i + 1; j < (signed)pointArray.size() && !foundInitialTriangle; j++)
		{
			for (int k = j + 1; k < (signed)pointArray.size() && !foundInitialTriangle; k++)
			{
				this->vertexArray->push_back(pointArray[i]);
				this->vertexArray->push_back(pointArray[j]);
				this->vertexArray->push_back(pointArray[k]);

				if (this->IsValid())
					foundInitialTriangle = true;
				else
				{
					this->vertexArray->clear();
					this->vertexArray->push_back(pointArray[i]);
					this->vertexArray->push_back(pointArray[k]);
					this->vertexArray->push_back(pointArray[j]);

					if (this->IsValid())
						foundInitialTriangle = true;
					else
						this->vertexArray->clear();
				}
			}
		}
	}

	std::list<Vector2D> pointList;
	for (const Vector2D& point : pointArray)
		pointList.push_back(point);

	while (true)
	{
		// Cull our list of points of those that exist in the current convex hull.
		auto iter = pointList.begin();
		while (iter != pointList.end())
		{
			auto nextIter(iter);
			nextIter++;

			const Vector2D& point = *iter;
			if (this->ContainsPoint(point))
				pointList.erase(iter);

			iter = nextIter;
		}

		// If the list is now empty, we're done!
		if (pointList.size() == 0)
			break;

		// Any point that remains is on the outside of the hull.  Pick one arbitrarily.
		iter = pointList.begin();
		const Vector2D& point = *iter;

		// Look for the left-most point.
		int i = 0;
		LineSegment lineSegment;
		while (i < (signed)this->vertexArray->size())
		{
			lineSegment = LineSegment(point, (*this->vertexArray)[i]);
			Line line(lineSegment);
			if (this->AllPointsOnOrBehindLine(line))
				break;

			i++;
		}

		// This shouldn't happen.
		if (i == (signed)this->vertexArray->size())
			return false;

		// Look for the right-most point.
		int j = 0;
		while (j < (signed)this->vertexArray->size())
		{
			lineSegment = LineSegment((*this->vertexArray)[j], point);
			Line line(lineSegment);
			if (this->AllPointsOnOrBehindLine(line))
				break;

			j++;
		}

		// This shouldn't happen either.
		if (j == (signed)this->vertexArray->size())
			return false;

		// Form the newly expanded convex hull.
		std::vector<Vector2D> newVertexArray;
		newVertexArray.push_back(point);
		while (true)
		{
			newVertexArray.push_back((*this->vertexArray)[j]);
			if (j == i)
				break;

			j = (j + 1) % this->vertexArray->size();
		}

		this->vertexArray->clear();
		for (const Vector2D& vertex : newVertexArray)
			this->vertexArray->push_back(vertex);
	}

	// TODO: Are we sure that all redundant points on the hull are removed?

	return true;
}

bool ConvexPolygon::AllPointsOnOrBehindLine(const Line& line) const
{
	for (const Vector2D& vertex : *this->vertexArray)
		if (line.WhichSide(vertex) == Line::Side::FRONT)
			return false;

	return true;
}

void ConvexPolygon::Clear()
{
	this->vertexArray->clear();
}

void ConvexPolygon::SetVertexCount(int count)
{
	this->vertexArray->resize(count);
}

int ConvexPolygon::GetVertexCount() const
{
	return (int)this->vertexArray->size();
}

bool ConvexPolygon::IsValid() const
{
	for (int i = 0; i < (signed)this->vertexArray->size(); i++)
	{
		int j = (i + 1) % this->vertexArray->size();

		const Vector2D& vertexA = (*this->vertexArray)[i];
		const Vector2D& vertexB = (*this->vertexArray)[j];

		Line line(LineSegment(vertexB, vertexA));

		if (!this->AllPointsOnOrBehindLine(line))
			return false;
	}

	return true;
}

bool ConvexPolygon::ContainsPoint(const Vector2D& point, double thickness /*= PLNR_PHY_EPSILON*/) const
{
	for (int i = 0; i < (signed)this->vertexArray->size(); i++)
	{
		int j = (i + 1) % this->vertexArray->size();

		const Vector2D& vertexA = (*this->vertexArray)[i];
		const Vector2D& vertexB = (*this->vertexArray)[j];

		PScalar2D det = (point - vertexA) ^ (vertexB - vertexA).Normalized();
		if (det.z > thickness)
			return false;
	}

	return true;
}

bool ConvexPolygon::CalcBoundingBox(BoundingBox& box) const
{
	if (this->vertexArray->size() == 0)
		return false;

	box.min = (*this->vertexArray)[0];
	box.max = box.min;
	for (int i = 1; i < (signed)this->vertexArray->size(); i++)
		box.ExpandToIncludePoint((*this->vertexArray)[i]);

	return true;
}

const Vector2D& ConvexPolygon::operator[](int i) const
{
	return (*this->vertexArray)[i];
}

Vector2D& ConvexPolygon::operator[](int i)
{
	return (*this->vertexArray)[i];
}

const std::vector<Vector2D>& ConvexPolygon::GetVertexArray() const
{
	return *this->vertexArray;
}

std::vector<Vector2D>& ConvexPolygon::GetVertexArray()
{
	return *this->vertexArray;
}