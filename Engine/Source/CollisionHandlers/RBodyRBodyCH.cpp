#include "RBodyRBodyCH.h"
#include "PlanarObjects/RigidBody.h"
#include "Math/Utilities/Line.h"

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

/*virtual*/ bool RBodyRBodyCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	auto bodyA = dynamic_cast<RigidBody*>(objectA);
	auto bodyB = dynamic_cast<RigidBody*>(objectB);

	if (!bodyA || !bodyB)
		return false;

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
		if (relativeVelocity < 0.0)
		{
			double termA = 1.0 / bodyA->mass - (rA ^ contact.normal) * (rA ^ contact.normal) / bodyA->inertia;
			double termB = 1.0 / bodyB->mass - (rB ^ contact.normal) * (rB ^ contact.normal) / bodyB->inertia;

			double j = -(1.0 + this->coefficientOfRestitution) * relativeVelocity / (termA + termB);

			Vector2D impulse = j * contact.normal;

			bodyA->velocity += impulse / bodyA->mass;
			bodyB->velocity -= impulse / bodyB->mass;

			PScalar2D impulsiveTorqueA = rA ^ impulse / bodyA->inertia;
			PScalar2D impulsiveTorqueB = rB ^ impulse / bodyB->inertia;

			bodyA->angularVelocity += impulsiveTorqueA;
			bodyB->angularVelocity -= impulsiveTorqueB;
		}
	}

	// At this point, despite all we've done thus far, make darn sure that the two rigid bodies are separated!
	// Two convex polygons do not overlap if there exists a separation plane between them.  If they really
	// are separate, then one such plane always exists that contains a face of one of the polygons.  Note that
	// the analog of this idea in 3 dimensions does not work!  But it does work in 2 dimensions.

	bodyA->UpdateWorldPolygonIfNeeded();
	bodyB->UpdateWorldPolygonIfNeeded();

	polygonA.UpdateEdgeLineCacheIfNeeded();
	polygonB.UpdateEdgeLineCacheIfNeeded();

	const std::vector<Line>& edgeLineArrayA = polygonA.GetEdgeLineArray();
	const std::vector<Line>& edgeLineArrayB = polygonB.GetEdgeLineArray();

	struct SeparationCase
	{
		double distance;
		Vector2D normal;
	};

	std::vector<SeparationCase> separationCaseArray;

	for (const Line& edgeLineA : edgeLineArrayA)
	{
		double penetrationDistance = 0.0;
		if (bodyB->AllVerticesOnOrInFrontOfLine(edgeLineA, penetrationDistance))
			return contactArray.size() > 0;

		separationCaseArray.push_back(SeparationCase{ ::abs(penetrationDistance), -edgeLineA.normal });
	}

	for (const Line& edgeLineB : edgeLineArrayB)
	{
		double penetrationDistance = 0.0;
		if (bodyA->AllVerticesOnOrInFrontOfLine(edgeLineB, penetrationDistance))
			return contactArray.size() > 0;

		separationCaseArray.push_back(SeparationCase{ ::abs(penetrationDistance), edgeLineB.normal });
	}

	double smallestDistance = std::numeric_limits<double>::max();
	const SeparationCase* chosenCase = nullptr;
	for (const SeparationCase& separationCase: separationCaseArray)
	{
		if (separationCase.distance < smallestDistance)
		{
			smallestDistance = separationCase.distance;
			chosenCase = &separationCase;
		}
	}

	if (chosenCase)
	{
		// After this nudge, the bodies *should* no longer be overlapping.
		bodyA->position += chosenCase->normal * chosenCase->distance / 2.0;
		bodyB->position -= chosenCase->normal * chosenCase->distance / 2.0;

		bodyA->worldPolygonValid = false;
		bodyB->worldPolygonValid = false;
	}

	return contactArray.size() > 0;
}