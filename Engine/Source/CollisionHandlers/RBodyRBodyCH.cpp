#include "RBodyRBodyCH.h"
#include "PlanarObjects/RigidBody.h"
#include "Math/Utilities/Ray.h"

using namespace PlanarPhysics;

RBodyRBodyCH::RBodyRBodyCH()
{
	this->pointRadius = 0.1;
}

/*virtual*/ RBodyRBodyCH::~RBodyRBodyCH()
{
}

void RBodyRBodyCH::AddUniqueContact(const PlanarObject::Contact& contact)
{
	for (const PlanarObject::Contact& existingContact : this->contactArray)
	{
		Vector2D distanceVector = existingContact.point - contact.point;
		double squareDistance = distanceVector | distanceVector;
		if (squareDistance < this->pointRadius)
			return;
	}

	this->contactArray.push_back(contact);
}

/*virtual*/ void RBodyRBodyCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	auto bodyA = dynamic_cast<RigidBody*>(objectA);
	auto bodyB = dynamic_cast<RigidBody*>(objectB);

	if (!bodyA || !bodyB)
		return;

	this->contactArray.clear();

	const ConvexPolygon& polygonA = bodyA->GetWorldPolygon();
	const ConvexPolygon& polygonB = bodyB->GetWorldPolygon();

	for (const Vector2D& vertexA : polygonA.GetVertexArray())
	{
		PlanarObject::Contact contact;

		if (bodyB->PointPenetratesConvexPolygon(vertexA, contact, pointRadius))
		{
			this->AddUniqueContact(contact);
		}
	}

	for (const Vector2D& vertexB : polygonB.GetVertexArray())
	{
		PlanarObject::Contact contact;

		if (bodyA->PointPenetratesConvexPolygon(vertexB, contact, pointRadius))
		{
			contact.normal = -contact.normal;
			this->AddUniqueContact(contact);
		}
	}

	// TODO: Not so sure about this in the case that we have multiple contact points.
	//       David Baraff solves a quadratic programming problem in some cases, but I can't understand it.
	for (const PlanarObject::Contact& contact : this->contactArray)
	{
		bodyA->position += contact.normal * contact.penetrationDepth / 2.0;
		bodyB->position -= contact.normal * contact.penetrationDepth / 2.0;

		bodyA->worldPolygonValid = false;
		bodyB->worldPolygonValid = false;

		Vector2D rA = contact.point - bodyA->position;
		Vector2D rB = contact.point - bodyB->position;

		Vector2D contactPointVelocityA = bodyA->velocity + rA * bodyA->angularVelocity;
		Vector2D contactPointVelocityB = bodyB->velocity + rB * bodyB->angularVelocity;

		double relativeVelocity = contact.normal | (contactPointVelocityA - contactPointVelocityB);
		if (::abs(relativeVelocity) < 0.2)
		{
			bodyA->inRestingContact = true;
			bodyB->inRestingContact = true;
		}
		else
		{
			bodyA->inRestingContact = false;
			bodyB->inRestingContact = false;
		}

		if (relativeVelocity < 0.0)
		{
			double coeficientOfRestitution = 0.9;

			double termA = 1.0 / bodyA->mass - (rA ^ contact.normal) * (rA ^ contact.normal) / bodyA->inertia;
			double termB = 1.0 / bodyB->mass - (rB ^ contact.normal) * (rB ^ contact.normal) / bodyB->inertia;

			double j = -(1.0 + coeficientOfRestitution) * relativeVelocity / (termA + termB);

			Vector2D impulse = j * contact.normal;

			bodyA->velocity += impulse / bodyA->mass;
			bodyB->velocity -= impulse / bodyB->mass;

			PScalar2D impulsiveTorqueA = rA ^ impulse / bodyA->inertia;
			PScalar2D impulsiveTorqueB = rB ^ impulse / bodyB->inertia;

			bodyA->angularVelocity += impulsiveTorqueA;
			bodyB->angularVelocity -= impulsiveTorqueB;
		}
	}

	// TODO: Maybe at this point do a hail-marry to make sure the two bodies are just not intersecting anymore at all?
	//       It's not entirely obvious to me how to do that in every case.

#if 0
	bodyA->worldPolygonValid = false;
	bodyB->worldPolygonValid = false;

	const ConvexPolygon& polygonA = bodyA->GetWorldPolygon();
	const ConvexPolygon& polygonB = bodyB->GetWorldPolygon();

	for (int i = 0; i < (signed)polygonA.GetVertexCount(); i++)
	{
		int j = (i + 1) % polygonA.GetVertexCount();

		const Vector2D& vertexA = polygonA.GetVertexArray()[i];
		const Vector2D& vertexB = polygonA.GetVertexArray()[j];

		Ray ray(vertexA, vertexB - vertexA);

		double lambda = 0.0;
		Vector2D hitNormal;
		if (ray.CastAgainst(polygonB, lambda, &hitNormal))
		{
			Vector2D hitPoint = ray.CalculateRayPoint(lambda);

			int b = 0;
			b++;
		}
	}
#endif
}