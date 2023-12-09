#include "LineSegment.h"
#include "Ray.h"

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

double LineSegment::CalculateLineLerpAlpha(const Vector2D& linePoint) const
{
	Vector2D vector = this->vertexB - this->vertexA;
	return ((linePoint - this->vertexA) | vector) / (vector | vector);
}

Vector2D& LineSegment::operator[](int i)
{
	if (i % 2 == 0)
		return this->vertexA;
	return this->vertexB;
}

const Vector2D& LineSegment::operator[](int i) const
{
	return (*const_cast<LineSegment*>(this))[i];
}

bool LineSegment::CalcIntersectionPoint(const LineSegment& lineSegment, Vector2D& intersectionPoint) const
{
	Ray ray(lineSegment.vertexA, lineSegment.vertexB - lineSegment.vertexA);
	double lambda = 0.0;
	if (!ray.CastAgainst(*this, lambda))
		return false;

	intersectionPoint = ray.CalculateRayPoint(lambda);
	return true;
}