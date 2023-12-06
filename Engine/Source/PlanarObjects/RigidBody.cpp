#include "RigidBody.h"
#include "Engine.h"
#include "Math/GeometricAlgebra/Rotor2D.h"
#include "Math/GeometricAlgebra/PScalar2D.h"
#include "Math/Utilities/BoundingBox.h"
#include "Math/Utilities/LineSegment.h"

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

	Vector2D center(0.0, 0.0);
	for (const Vector2D& vertex : this->localPolygon.GetVertexArray())
		center += vertex;
	center /= double(this->localPolygon.GetVertexCount());
	for (Vector2D& vertex : this->localPolygon.GetVertexArray())
		vertex -= center;

	this->worldPolygonValid = false;

	PScalar2D area = 0.0;
	for (int i = 0; i < (signed)this->localPolygon.GetVertexCount(); i++)
	{
		int j = (i + 1) % this->localPolygon.GetVertexCount();

		const Vector2D& vertexA = this->localPolygon[i];
		const Vector2D& vertexB = this->localPolygon[j];

		area += (vertexA ^ vertexB) / 2.0;
	}

	this->mass = area.z * uniformDensity;
	this->inertia = 0.0;

	this->UpdateWorldPolygonIfNeeded();

	BoundingBox box;
	if (!this->worldPolygon.CalcBoundingBox(box))
		return false;

	// Approximate our rotational inertia.
	int resolution = 100;
	for (int i = 0; i < resolution; i++)
	{
		for (int j = 0; j < resolution; j++)
		{
			Vector2D uvA(double(i) / double(resolution), double(j) / double(resolution));
			Vector2D uvB(double(i + 1) / double(resolution), double(j + 1) / double(resolution));

			BoundingBox subBox(box.PointFromUVs(uvA), box.PointFromUVs(uvB));

			Vector2D subBoxCenter = subBox.Center();
			if (this->ContainsPoint(subBoxCenter))
			{
				Vector2D vector = subBoxCenter - this->position;
				this->inertia += subBox.Area() * uniformDensity * (vector | vector);
			}
		}
	}

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

	this->worldPolygonValid = false;
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

bool RigidBody::PointPenetratesConvexPolygon(const Vector2D& point, Contact& contact) const
{
	this->UpdateWorldPolygonIfNeeded();

	if (!this->worldPolygon.ContainsPoint(point))
		return false;

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