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

/*virtual*/ bool RigidBodyWallCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	RigidBody* body = dynamic_cast<RigidBody*>(objectA);
	const Wall* wall = dynamic_cast<Wall*>(objectB);

	if (!body || !wall)
	{
		body = dynamic_cast<RigidBody*>(objectB);
		wall = dynamic_cast<Wall*>(objectA);

		if (!body || !wall)
			return false;
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
			const Vector2D* closestVertex = nullptr;
			double minSquareDistance = std::numeric_limits<double>::max();
			for (const Vector2D& vertex : polygon.GetVertexArray())
			{
				Vector2D distanceVector = vertex - contact.point;
				double squareDistance = distanceVector | distanceVector;
				if (squareDistance < minSquareDistance)
				{
					minSquareDistance = squareDistance;
					closestVertex = &vertex;
				}
			}

			assert(closestVertex != nullptr);

			Vector2D distanceVectorA = wall->lineSeg.vertexA - contact.point;
			Vector2D distanceVectorB = wall->lineSeg.vertexB - contact.point;

			double squareDistanceA = distanceVectorA | distanceVectorA;
			double squareDistanceB = distanceVectorB | distanceVectorB;

			if (minSquareDistance < squareDistanceA && minSquareDistance < squareDistanceB)
			{
				contact.normal = wall->Normal();
				contact.penetrationDepth = wall->lineSeg.DistanceTo(*closestVertex);
				if ((contact.normal | (body->position - *closestVertex)) < 0.0)
					contact.normal = -contact.normal;
			}
			else
			{
				if (squareDistanceA < squareDistanceB)
					closestVertex = &wall->lineSeg.vertexA;
				else
					closestVertex = &wall->lineSeg.vertexB;

				contact.normal = ((vertexB - vertexA) * PScalar2D(1.0)).Normalized();
				contact.penetrationDepth = edgeSegment.DistanceTo(*closestVertex);
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
		if (relativeVelocity < 0.0)
		{
			double j = -(1.0 + this->coefficientOfRestitution) * relativeVelocity / (1.0 / body->mass - (r ^ contact.normal) * (r ^ contact.normal) / body->inertia);

			Vector2D impulse = j * contact.normal;
			body->velocity += impulse / body->mass;

			PScalar2D impulsiveTorque = r ^ impulse / body->inertia;
			body->angularVelocity += impulsiveTorque;
		}
	}

	return contactArray.size() > 0;
}