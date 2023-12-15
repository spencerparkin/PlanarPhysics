#include "RBodyRBodyCH.h"
#include "PlanarObjects/RigidBody.h"
#include "Math/Utilities/Ray.h"

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

#if 0
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
			
			for (const PlanarObject::Contact& contact : contactArray)
			{
				if ((contact.point - hitPoint).Magnitude() < PLNR_PHY_EPSILON)
				{
					int b = 0;
					b++;
				}
			}
		}
	}
#endif

	// TODO: Not so sure about this in the case that we have multiple contact points.
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
}