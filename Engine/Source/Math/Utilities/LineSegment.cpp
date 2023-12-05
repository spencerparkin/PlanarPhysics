#include "LineSegment.h"

using namespace PlanarPhysics;

LineSegment::LineSegment()
{
}

LineSegment::LineSegment(const LineSegment& lineSegment)
{
	this->vertexA = lineSegment.vertexA;
	this->vertexB = lineSegment.vertexB;
}

LineSegment::LineSegment(const Vector2D& pointA, const Vector2D& pointB)
{
	this->vertexA = pointA;
	this->vertexB = pointB;
}

/*virtual*/ LineSegment::~LineSegment()
{
}

void LineSegment::operator=(const LineSegment& lineSegment)
{
	this->vertexA = lineSegment.vertexA;
	this->vertexB = lineSegment.vertexB;
}

double LineSegment::Length() const
{
	return (this->vertexB - this->vertexA).Magnitude();
}

Vector2D LineSegment::NearestPoint(const Vector2D& point) const
{
	Vector2D normal = (this->vertexB - this->vertexA).Normalized();

	double dot = normal | (point - this->vertexA);
	if (dot < 0.0)
		return this->vertexA;
	else if (dot > this->Length())
		return this->vertexB;

	return this->vertexA + normal * dot;
}

double LineSegment::DistanceTo(const Vector2D& point) const
{
	return (this->NearestPoint(point) - point).Magnitude();
}