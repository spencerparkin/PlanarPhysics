#include "RBodyRBodyCH.h"
#include "PlanarObjects/RigidBody.h"

using namespace PlanarPhysics;

RBodyRBodyCH::RBodyRBodyCH()
{
}

/*virtual*/ RBodyRBodyCH::~RBodyRBodyCH()
{
}

/*virtual*/ void RBodyRBodyCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	auto bodyA = dynamic_cast<RigidBody*>(objectA);
	auto bodyB = dynamic_cast<RigidBody*>(objectB);

	if (!bodyA || !bodyB)
		return;

	std::vector<PlanarObject::Contact> contactArray;

	// TODO: It is possible for two polygons to overlap and yet, neither has a vertex contained within the other.

	for (const Vector2D& vertexA : bodyA->GetWorldPolygon().GetVertexArray())
	{
		PlanarObject::Contact contact;

		if (bodyB->PointPenetratesConvexPolygon(vertexA, contact))
		{
			contactArray.push_back(contact);
		}
	}

	for (const Vector2D& vertexB : bodyB->GetWorldPolygon().GetVertexArray())
	{
		PlanarObject::Contact contact;

		if (bodyA->PointPenetratesConvexPolygon(vertexB, contact))
		{
			contact.normal = -contact.normal;
			contactArray.push_back(contact);
		}
	}

	for (const PlanarObject::Contact& contact : contactArray)
	{
		bodyA->position += contact.normal * contact.penetrationDepth / 2.0;
		bodyB->position -= contact.normal * contact.penetrationDepth / 2.0;

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
			double coeficientOfRestitution = 0.7;

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
}