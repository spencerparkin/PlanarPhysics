#include "RigidBodyBallCH.h"
#include "PlanarObjects/Ball.h"
#include "PlanarObjects/RigidBody.h"
#include "Math/Utilities/LineSegment.h"

using namespace PlanarPhysics;

RigidBodyBallCH::RigidBodyBallCH()
{
}

/*virtual*/ RigidBodyBallCH::~RigidBodyBallCH()
{
}

/*virtual*/ void RigidBodyBallCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	RigidBody* body = dynamic_cast<RigidBody*>(objectA);
	Ball* ball = dynamic_cast<Ball*>(objectB);

	if (!body || !ball)
	{
		body = dynamic_cast<RigidBody*>(objectB);
		ball = dynamic_cast<Ball*>(objectA);

		if (!body || !ball)
			return;
	}

	// TODO: There is a bug here where rigid bodies are tunneling through walls,
	//       and not reacting correctly, maybe in the case that the wall normal
	//       is in an unexpected direction.  Fix it.

	body->UpdateWorldPolygonIfNeeded();

	const ConvexPolygon& worldPolygon = body->GetWorldPolygon();
	
	PlanarObject::Contact contact;

	for (int i = 0; i < worldPolygon.GetVertexCount(); i++)
	{
		int j = (i + 1) % worldPolygon.GetVertexCount();

		const Vector2D& vertexA = worldPolygon.GetVertexArray()[i];
		const Vector2D& vertexB = worldPolygon.GetVertexArray()[j];

		LineSegment edgeSegment(vertexA, vertexB);

		double distance = edgeSegment.DistanceTo(ball->position);

		if (distance < ball->radius)
		{
			Vector2D edgePoint = edgeSegment.NearestPoint(ball->position);

			contact.point = edgePoint;
			contact.normal = (ball->position - edgePoint).Normalized();
			contact.penetrationDepth = ball->radius - distance;

			Vector2D rBall = contact.point - ball->position;
			Vector2D rBody = contact.point - body->position;

			Vector2D ballPointVelocity = ball->velocity;
			Vector2D bodyPointVelocity = body->velocity + rBody * body->angularVelocity;

			double relativeVelocity = contact.normal | (ballPointVelocity - bodyPointVelocity);
			if (relativeVelocity < 0.0)
			{
				ball->position += contact.normal * contact.penetrationDepth / 2.0;
				body->position -= contact.normal * contact.penetrationDepth / 2.0;

				body->worldPolygonValid = false;

				double termA = 1.0 / ball->Mass() - (rBall ^ contact.normal) * (rBall ^ contact.normal) / ball->Inertia();
				double termB = 1.0 / body->mass - (rBody ^ contact.normal) * (rBody ^ contact.normal) / body->inertia;

				double j = -(1.0 + this->coeficientOfRestitution) * relativeVelocity / (termA + termB);

				Vector2D impulse = j * contact.normal;

				ball->velocity += impulse / ball->Mass();
				body->velocity -= impulse / body->mass;

				PScalar2D impulsiveTorque = rBody ^ impulse / body->inertia;

				body->angularVelocity -= impulsiveTorque;

				return;
			}
		}
	}
}