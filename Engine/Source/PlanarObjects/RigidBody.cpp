#include "RigidBody.h"
#include "Engine.h"
#include "Math/GeometricAlgebra/Rotor2D.h"
#include "Math/GeometricAlgebra/PScalar2D.h"
#include "Math/Utilities/BoundingBox.h"
#include "Math/Utilities/LineSegment.h"
#include "Math/Utilities/Line.h"

using namespace PlanarPhysics;

RigidBody::RigidBody()
{
	this->inertia = 0.0;
	this->mass = 0.0;
	this->worldPolygonValid = false;
	this->inRestingContact = false;
}

/*virtual*/ RigidBody::~RigidBody()
{
}

/*static*/ PlanarObject::Type RigidBody::StaticType()
{
	return Type::RIGID_BODY;
}

/*static*/ RigidBody* RigidBody::Create()
{
	return new RigidBody();
}

/*virtual*/ PlanarObject* RigidBody::CreateNew() const
{
	return new RigidBody();
}

/*virtual*/ PlanarObject::Type RigidBody::GetType() const
{
	return Type::RIGID_BODY;
}

bool RigidBody::MakeShape(const std::vector<Vector2D>& pointArray, double uniformDensity)
{
	if (pointArray.size() == 0)
		return false;

	if (!this->localPolygon.CalcConvexHull(pointArray))
		return false;

	Vector2D desiredPosition = this->position;
	PScalar2D desiredOrientation = this->orientation;
	this->position = Vector2D(0.0, 0.0);
	this->orientation = PScalar2D(0.0);

	this->worldPolygonValid = false;
	this->UpdateWorldPolygonIfNeeded();
	BoundingBox box;
	if (!this->worldPolygon.CalcBoundingBox(box))
		return false;

	// Here we approximate the mass and the center of mass.  We could calculate it exactly, but
	// we are also approximating the moment of inertia, so this is just consistent with that.
	int resolution = 100;
	Vector2D totalMoments(0.0, 0.0);
	this->mass = 0.0;
	box.IntegrateOverArea(resolution, [this, uniformDensity, &totalMoments](const BoundingBox& subBox) {
		Vector2D subBoxCenter = subBox.Center();
		if (this->ContainsPoint(subBoxCenter))
		{
			double subBoxMass = subBox.Area() * uniformDensity;
			totalMoments += subBoxMass * subBoxCenter;
			this->mass += subBoxMass;
		}
	});

	// To simplify our equations, we want the origin to be the center of mass in local space.
	Vector2D centerOfMass = totalMoments / this->mass;
	for (Vector2D& vertex : this->localPolygon.GetVertexArray())
		vertex -= centerOfMass;

	this->inertia = 0.0;
	this->worldPolygonValid = false;
	this->UpdateWorldPolygonIfNeeded();
	if (!this->worldPolygon.CalcBoundingBox(box))
		return false;

	// Go approximate the angular inertia.
	box.IntegrateOverArea(resolution, [this, uniformDensity](const BoundingBox& subBox) {
		Vector2D subBoxCenter = subBox.Center();
		if (this->ContainsPoint(subBoxCenter))
		{
			double subBoxMass = subBox.Area() * uniformDensity;
			this->inertia += subBoxMass * (subBoxCenter | subBoxCenter);
		}
	});

	this->position = desiredPosition;
	this->orientation = desiredOrientation;
	this->worldPolygonValid = false;
	return true;
}

bool RigidBody::ContainsPoint(const Vector2D& point) const
{
	this->UpdateWorldPolygonIfNeeded();

	return this->worldPolygon.ContainsPoint(point);
}

void RigidBody::UpdateWorldPolygonIfNeeded() const
{
	if (!this->worldPolygonValid)
	{
		this->worldPolygon.SetVertexCount(this->localPolygon.GetVertexCount());

		Rotor2D rotor = this->orientation.Exponent();

		for (int i = 0; i < this->localPolygon.GetVertexCount(); i++)
			this->worldPolygon[i] = this->position + this->localPolygon[i] * rotor;

		this->worldPolygonValid = true;
		this->worldPolygon.edgeLineCacheArrayValid = false;
	}
}

const ConvexPolygon& RigidBody::GetWorldPolygon() const
{
	this->UpdateWorldPolygonIfNeeded();
	
	return this->worldPolygon;
}

/*virtual*/ void RigidBody::Integrate(double deltaTime)
{
	Vector2D acceleration = this->netForce / this->mass;
	this->velocity += acceleration * deltaTime;

	PScalar2D angularAcceleration = this->netTorque / this->inertia;
	this->angularVelocity += angularAcceleration * deltaTime;

	this->position += this->velocity * deltaTime;
	this->orientation += this->angularVelocity * deltaTime;

	if (this->velocity != Vector2D(0.0, 0.0) || this->angularVelocity != PScalar2D(0.0))
	{
		this->cachedBoundingBoxValid = false;
		this->worldPolygonValid = false;
	}
}

/*virtual*/ void RigidBody::AccumulateForces(const Engine* engine)
{
	if ((this->flags & PLNR_OBJ_FLAG_INFLUENCED_BY_GRAVITY) != 0)
	{
		this->netForce += engine->accelerationDueToGravity * this->mass;
	}

	if (this->inRestingContact)
	{
		double frictionCoeficient = 10.0;
		Vector2D frictionForce = this->mass * this->velocity * -1.0 * frictionCoeficient;
		this->netForce += frictionForce;
	}
}

/*virtual*/ void RigidBody::AdvanceBegin()
{
	this->netForce = Vector2D(0.0, 0.0);
	this->netTorque = PScalar2D(0.0);
}

bool RigidBody::PointPenetratesConvexPolygon(const Vector2D& point, Contact& contact, double vertexRadius /*= 0.1*/) const
{
	this->UpdateWorldPolygonIfNeeded();

	if (!this->worldPolygon.ContainsPoint(point))
		return false;

	for (int i = 0; i < this->worldPolygon.GetVertexCount(); i++)
	{
		const Vector2D& vertex = this->worldPolygon.GetVertexArray()[i];
		Vector2D distanceVector = point - vertex;
		double distanceSquared = distanceVector | distanceVector;
		if (distanceSquared < vertexRadius)
		{
			contact.point = vertex;
			contact.penetrationDepth = ::sqrt(distanceSquared);
			
			int j = (i + 1) % this->worldPolygon.GetVertexCount();
			Vector2D normalA = (vertex - this->worldPolygon.GetVertexArray()[j]).Normalized();

			j = (i + this->worldPolygon.GetVertexCount() - 1) % this->worldPolygon.GetVertexCount();
			Vector2D normalB = (vertex - this->worldPolygon.GetVertexArray()[j]).Normalized();

			contact.normal = (normalA + normalB).Normalized();
			return true;
		}
	}

	contact.penetrationDepth = std::numeric_limits<double>::max();

	for (int i = 0; i < this->worldPolygon.GetVertexCount(); i++)
	{
		int j = (i + 1) % this->worldPolygon.GetVertexCount();

		const Vector2D& vertexA = this->worldPolygon[i];
		const Vector2D& vertexB = this->worldPolygon[j];

		LineSegment edgeSegment(vertexA, vertexB);

		Vector2D edgePoint = edgeSegment.NearestPoint(point);
		double distance = (edgePoint - point).Magnitude();
		if (distance < contact.penetrationDepth)
		{
			contact.penetrationDepth = distance;
			contact.point = edgePoint;
			contact.normal = ((edgeSegment.vertexA - edgeSegment.vertexB) * PScalar2D(1.0)).Normalized();
		}
	}

	return true;
}

bool RigidBody::AllVerticesOnOrInFrontOfLine(const Line& line, double& penetrationDistance, double lineThickness /*= PLNR_PHY_EPSILON*/) const
{
	this->UpdateWorldPolygonIfNeeded();

	penetrationDistance = std::numeric_limits<double>::max();

	for (const Vector2D& vertex : this->worldPolygon.GetVertexArray())
	{
		double signedDistance = line.SignedDistanceTo(vertex);
		if (signedDistance < penetrationDistance)
			penetrationDistance = signedDistance;
	}

	if (penetrationDistance >= -lineThickness)
		return true;

	return false;
}

/*virtual*/ void RigidBody::CalcBoundingBox(BoundingBox& box) const
{
	box.min = this->position;
	box.max = this->position;

	this->UpdateWorldPolygonIfNeeded();

	for (const Vector2D& vertex : this->worldPolygon.GetVertexArray())
		box.ExpandToIncludePoint(vertex);
}