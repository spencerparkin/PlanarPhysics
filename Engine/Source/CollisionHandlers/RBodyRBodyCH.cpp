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
	}
}