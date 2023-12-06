#include "RigidBodyWallCH.h"
#include "PlanarObjects/Wall.h"
#include "PlanarObjects/RigidBody.h"

using namespace PlanarPhysics;

RigidBodyWallCH::RigidBodyWallCH()
{
}

/*virtual*/ RigidBodyWallCH::~RigidBodyWallCH()
{
}

/*virtual*/ void RigidBodyWallCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	RigidBody* body = dynamic_cast<RigidBody*>(objectA);
	const Wall* wall = dynamic_cast<Wall*>(objectB);

	if (!body || !wall)
	{
		body = dynamic_cast<RigidBody*>(objectB);
		wall = dynamic_cast<Wall*>(objectA);

		if (!body || !wall)
			return;
	}

	body->UpdateWorldPolygonIfNeeded();

	Vector2D contactNormal = wall->Normal();

	for (const Vector2D& vertex : body->GetWorldPolygon().GetVertexArray())
	{
		Vector2D nearestPoint = wall->lineSeg.NearestPoint(vertex);
		Vector2D direction = vertex - nearestPoint;
		double penetrationDistance = direction | contactNormal;
		if (penetrationDistance >= 0.0)
			continue;

		body->position -= contactNormal * penetrationDistance;
		Vector2D contactPoint = vertex - contactNormal * penetrationDistance;

		Vector2D r = contactPoint - body->position;
		Vector2D contactPointVelocity = body->velocity + r * body->angularVelocity;
		double relativeVelocity = contactNormal | contactPointVelocity;
		if (::abs(relativeVelocity) < 0.2)
			body->inRestingContact = true;
		else
			body->inRestingContact = false;

		if (relativeVelocity < 0.0)
		{
			double coeficientOfRestitution = 0.7;

			double j = -(1.0 + coeficientOfRestitution) * relativeVelocity / (1.0 / body->mass - (r ^ contactNormal) * (r ^ contactNormal) / body->inertia);

			Vector2D impulse = j * contactNormal;
			body->velocity += impulse / body->mass;

			PScalar2D impulsiveTorque = r ^ impulse / body->inertia;
			body->angularVelocity += impulsiveTorque;
		}
	}
}