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

double LineSegment::SquareDistanceTo(const Vector2D& point) const
{
	Vector2D distanceVector = this->NearestPoint(point) - point;
	return distanceVector | distanceVector;
}

double LineSegment::DistanceTo(const Vector2D& point) const
{
	return ::sqrt(this->SquareDistanceTo(point));
}

double LineSegment::CalculateLineLerpAlpha(const Vector2D& linePoint) const
{
	Vector2D vector = this->vertexB - this->vertexA;
	return ((linePoint - this->vertexA) | vector) / (vector | vector);
}

Vector2D& LineSegment::operator[](int i)
{
	return (i % 2 == 0) ? this->vertexA : this->vertexB;
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

	if (lambda < 0.0 || lambda > 1.0)
		return false;

	intersectionPoint = ray.CalculateRayPoint(lambda);
	return true;
}

bool LineSegment::SameGeometryAs(const LineSegment& lineSegment, double tolerance /*= PLNR_PHY_EPSILON*/) const
{
	double squareTolerance = tolerance * tolerance;

	Vector2D distanceVectorA = lineSegment.vertexA - this->vertexA;
	Vector2D distanceVectorB = lineSegment.vertexB - this->vertexB;

	if ((distanceVectorA | distanceVectorA) < squareTolerance && (distanceVectorB | distanceVectorB) < squareTolerance)
		return true;

	distanceVectorA = lineSegment.vertexA - this->vertexB;
	distanceVectorB = lineSegment.vertexB - this->vertexA;

	if ((distanceVectorA | distanceVectorA) < squareTolerance && (distanceVectorB | distanceVectorB) < squareTolerance)
		return true;

	return false;
}

bool LineSegment::ContainsPoint(const Vector2D& point, double tolerance /*= PLNR_PHY_EPSILON*/) const
{
	return this->SquareDistanceTo(point) < tolerance * tolerance;
}

bool LineSegment::Merge(const LineSegment& lineSegA, const LineSegment& lineSegB, double tolerance /*= PLNR_PHY_EPSILON*/)
{
	Vector2D point;

	if (lineSegA.SameGeometryAs(lineSegB))
	{
		*this = lineSegA;
		return true;
	}

	if (lineSegA.vertexA.IsPoint(lineSegB.vertexA, tolerance))
	{
		this->vertexA = lineSegA.vertexB;
		this->vertexB = lineSegB.vertexB;
		point = (lineSegA.vertexA + lineSegB.vertexA) / 2.0;
	}
	else if (lineSegA.vertexA.IsPoint(lineSegB.vertexB, tolerance))
	{
		this->vertexA = lineSegA.vertexB;
		this->vertexB = lineSegB.vertexA;
		point = (lineSegA.vertexA + lineSegB.vertexB) / 2.0;
	}
	else if (lineSegA.vertexB.IsPoint(lineSegB.vertexA, tolerance))
	{
		this->vertexA = lineSegA.vertexA;
		this->vertexB = lineSegB.vertexB;
		point = (lineSegA.vertexB + lineSegB.vertexA) / 2.0;
	}
	else if (lineSegA.vertexB.IsPoint(lineSegB.vertexB, tolerance))
	{
		this->vertexA = lineSegA.vertexA;
		this->vertexB = lineSegB.vertexA;
		point = (lineSegA.vertexB + lineSegB.vertexB) / 2.0;
	}
	else
	{
		return false;
	}

	return this->ContainsPoint(point, tolerance);
}