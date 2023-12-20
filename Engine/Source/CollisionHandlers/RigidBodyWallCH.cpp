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

	std::vector<PlanarObject::Contact> contactArray;

	const ConvexPolygon& polygon = body->GetWorldPolygon();

	for (int i = 0; i < polygon.GetVertexCount(); i++)
	{
		int j = (i + 1) % polygon.GetVertexCount();

		const Vector2D& vertexA = polygon.GetVertexArray()[i];
		const Vector2D& vertexB = polygon.GetVertexArray()[j];

		LineSegment edgeSegment(vertexA, vertexB);

		PlanarObject::Contact contact;

		if (edgeSegment.CalcIntersectionPoint(wall->lineSeg, contact.point))
		{
			bool containsVertexA = polygon.ContainsPoint(wall->lineSeg.vertexA);
			bool containsVertexB = polygon.ContainsPoint(wall->lineSeg.vertexB);

			if (containsVertexA || containsVertexB)
			{
				contact.normal = ((vertexB - vertexA) * PScalar2D(1.0)).Normalized();

				if (containsVertexA)
					contact.penetrationDepth = edgeSegment.DistanceTo(vertexA);
				else
					contact.penetrationDepth = edgeSegment.DistanceTo(vertexB);
			}
			else
			{
				contact.normal = wall->Normal();

				const Vector2D* chosenVertex = nullptr;
				double minDistance = std::numeric_limits<double>::max();
				for (const Vector2D& vertex : polygon.GetVertexArray())
				{
					double distance = wall->lineSeg.DistanceTo(vertex);
					if (distance < minDistance)
					{
						minDistance = distance;
						chosenVertex = &vertex;
					}
				}

				if ((contact.normal | (body->position - *chosenVertex)) < 0.0)
					contact.normal = -contact.normal;

				contact.penetrationDepth = minDistance;
			}

			contactArray.push_back(contact);
		}
	}

	// TODO: Again, not so sure about this when there is more than one contact to address.
	for(const PlanarObject::Contact& contact : contactArray)
	{
		body->position += contact.normal * contact.penetrationDepth;
		body->worldPolygonValid = false;

		Vector2D r = contact.point - body->position;
		Vector2D contactPointVelocity = body->velocity + r * body->angularVelocity;
		double relativeVelocity = contact.normal | contactPointVelocity;
		if (::abs(relativeVelocity) < 0.5)
			body->inRestingContact = true;
		else
			body->inRestingContact = false;

		if (relativeVelocity < 0.0)
		{
			double j = -(1.0 + this->coeficientOfRestitution) * relativeVelocity / (1.0 / body->mass - (r ^ contact.normal) * (r ^ contact.normal) / body->inertia);

			Vector2D impulse = j * contact.normal;
			body->velocity += impulse / body->mass;

			PScalar2D impulsiveTorque = r ^ impulse / body->inertia;
			body->angularVelocity += impulsiveTorque;
		}
	}
}