#include "RigidBody.h"
#include "Engine.h"
#include "Math/GeometricAlgebra/Rotor2D.h"
#include "Math/Utilities/BoundingBox.h"

using namespace PlanarPhysics;

RigidBody::RigidBody()
{
	this->inertia = 0.0;
	this->mass = 0.0;
	this->worldVertexArrayValid = false;
	this->inRestingContact = false;
	this->localVertexArray = new std::vector<Vector2D>();
	this->worldVertexArray = new std::vector<Vector2D>();
}

/*virtual*/ RigidBody::~RigidBody()
{
	delete this->localVertexArray;
	delete this->worldVertexArray;
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

void RigidBody::MakeShape(const std::vector<Vector2D>& pointArray, double uniformDensity)
{
	if (pointArray.size() == 0)
		return;

	// TODO: Find convex hull of given point set.  For now, just assume it forms a convex call and is wound CCW.

	this->localVertexArray->clear();
	for (const Vector2D& point : pointArray)
		this->localVertexArray->push_back(point);

	Vector2D center(0.0, 0.0);
	for (const Vector2D& vertex : *this->localVertexArray)
		center += vertex;
	center /= double(this->localVertexArray->size());
	for (Vector2D& vertex : *this->localVertexArray)
		vertex -= center;

	this->worldVertexArrayValid = false;

	PScalar2D area = 0.0;
	for (int i = 0; i < (signed)this->localVertexArray->size(); i++)
	{
		int j = (i + 1) % this->localVertexArray->size();

		const Vector2D& vertexA = (*this->localVertexArray)[i];
		const Vector2D& vertexB = (*this->localVertexArray)[j];

		area += (vertexA ^ vertexB) / 2.0;
	}

	this->mass = area.z * uniformDensity;
	this->inertia = 0.0;

	this->UpdateWorldVertexArrayIfNeeded();

	BoundingBox box;
	box.min = (*this->worldVertexArray)[0];
	box.max = (*this->worldVertexArray)[0];
	for (const Vector2D& vertex : *this->worldVertexArray)
		box.ExpandToIncludePoint(vertex);

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
}

bool RigidBody::ContainsPoint(const Vector2D& point) const
{
	this->UpdateWorldVertexArrayIfNeeded();

	for (int i = 0; i < (signed)this->worldVertexArray->size(); i++)
	{
		int j = (i + 1) % this->worldVertexArray->size();

		const Vector2D& vertexA = (*this->worldVertexArray)[i];
		const Vector2D& vertexB = (*this->worldVertexArray)[j];

		PScalar2D det = (point - vertexA) ^ (vertexB - vertexA);
		if (det.z > 0.0)
			return false;
	}

	return true;
}

void RigidBody::UpdateWorldVertexArrayIfNeeded() const
{
	if (!this->worldVertexArrayValid)
	{
		this->worldVertexArray->clear();

		Rotor2D rotor = this->orientation.Exponent();

		for (const Vector2D& localVertex : *this->localVertexArray)
		{
			Vector2D worldVertex = this->position + localVertex * rotor;
			this->worldVertexArray->push_back(worldVertex);
		}

		this->worldVertexArrayValid = true;
	}
}

const std::vector<Vector2D>& RigidBody::GetWorldVertexArray() const
{
	this->UpdateWorldVertexArrayIfNeeded();
	
	return *this->worldVertexArray;
}

/*virtual*/ void RigidBody::Integrate(double deltaTime)
{
	Vector2D acceleration = this->netForce / this->mass;
	this->velocity += acceleration * deltaTime;

	PScalar2D angularAcceleration = this->netTorque / this->inertia;
	this->angularVelocity += angularAcceleration * deltaTime;

	this->position += this->velocity * deltaTime;
	this->orientation += this->angularVelocity * deltaTime;

	this->worldVertexArrayValid = false;
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