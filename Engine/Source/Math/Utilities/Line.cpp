#include "Line.h"
#include "LineSegment.h"
#include "Math/GeometricAlgebra/PScalar2D.h"

using namespace PlanarPhysics;

Line::Line()
{
	this->normal = Vector2D(1.0, 0.0);
	this->D = 0.0;
}

Line::Line(const Line& line)
{
	this->normal = line.normal;
	this->D = line.D;
}

Line::Line(const Vector2D& center, const Vector2D& normal)
{
	this->normal = normal.Normalized();
	this->D = this->normal | center;
}

Line::Line(const LineSegment& lineSegment)
{
	this->normal = ((lineSegment.vertexB - lineSegment.vertexA) * PScalar2D(1.0)).Normalized();
	this->D = this->normal | lineSegment.vertexA;
}

/*virtual*/ Line::~Line()
{
}

Vector2D Line::Center() const
{
	return this->normal * this->D;
}

double Line::SignedDistanceTo(const Vector2D& point) const
{
	return (point | this->normal) - this->D;
}

Line::Side Line::WhichSide(const Vector2D& point, double thickness /*= PLNR_PHY_EPSILON*/) const
{
	double signedDistance = this->SignedDistanceTo(point);
	if (signedDistance < -thickness)
		return Side::BACK;
	else if (signedDistance > thickness)
		return Side::FRONT;
	return Side::NEITHER;
}