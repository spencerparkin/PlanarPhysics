#include "Ray.h"
#include "Line.h"
#include "ConvexPolygon.h"
#include "LineSegment.h"
#include "Math/GeometricAlgebra/PScalar2D.h"

using namespace PlanarPhysics;

Ray::Ray()
{
}

Ray::Ray(const Ray& ray)
{
	this->origin = ray.origin;
	this->direction = ray.direction;
}

Ray::Ray(const Vector2D& origin, const Vector2D& direction)
{
	this->origin = origin;
	this->direction = direction;
}

/*virtual*/ Ray::~Ray()
{
}

Vector2D Ray::CalculateRayPoint(double lambda) const
{
	return this->origin + this->direction * lambda;
}

bool Ray::CastAgainst(const LineSegment& lineSegment, double& lambda, double eps /*= PLNR_PHY_EPSILON*/) const
{
	Vector2D lineSegVector = lineSegment.vertexB - lineSegment.vertexA;
	PScalar2D numerator = lineSegment.vertexA ^ this->origin;
	PScalar2D denominator = this->direction ^ lineSegVector;
	PScalar2D denominatorInv;
	if (!denominator.Invert(denominatorInv))
		return false;

	lambda = numerator * denominatorInv;

	Vector2D linePoint = this->CalculateRayPoint(lambda);
	double lerpAlpha = lineSegment.CalculateLineLerpAlpha(linePoint);
	if (lerpAlpha < -eps || lerpAlpha > 1.0 + eps)
		return false;

	return true;
}

bool Ray::CastAgainst(const ConvexPolygon& convexPolygon, double& lambda, Vector2D* hitNormal /*= nullptr*/, double eps /*= PLNR_PHY_EPSILON*/) const
{
	lambda = std::numeric_limits<double>::max();
	bool hitFound = false;

	const std::vector<Vector2D>& vertexArray = convexPolygon.GetVertexArray();

	for (int i = 0; i < (signed)vertexArray.size(); i++)
	{
		int j = (i + 1) % vertexArray.size();
		LineSegment edgeSegment(vertexArray[i], vertexArray[j]);
		double tentativeLambda = 0.0;
		if (this->CastAgainst(edgeSegment, tentativeLambda, eps) && tentativeLambda < lambda)
		{
			hitFound = true;
			lambda = tentativeLambda;
			if (hitNormal)
				*hitNormal = ((edgeSegment.vertexA - edgeSegment.vertexB) * PScalar2D(1.0)).Normalized();
		}
	}

	return hitFound;
}

bool Ray::CastAgainst(const Line& line, double& lambda) const
{
	double numerator = line.D - (this->origin | line.normal);
	double denominator = this->direction | line.normal;

	if (denominator == 0.0)
		return false;

	lambda = numerator / denominator;
	if (::isinf(lambda) || ::isnan(lambda))
		return false;

	return true;
}