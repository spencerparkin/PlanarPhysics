#include "BallBallCH.h"
#include "PlanarObjects/Ball.h"

using namespace PlanarPhysics;

BallBallCH::BallBallCH()
{
}

/*virtual*/ BallBallCH::~BallBallCH()
{
}

/*virtual*/ bool BallBallCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	Ball* ballA = dynamic_cast<Ball*>(objectA);
	Ball* ballB = dynamic_cast<Ball*>(objectB);

	if (!ballA || !ballB)
		return false;

	Vector2D contactNormal = ballA->position - ballB->position;
	double distanceBetweenCenters = contactNormal.Magnitude();
	double totalRadius = ballA->radius + ballB->radius;
	if (distanceBetweenCenters < totalRadius)
	{
		double correctionLength = (totalRadius - distanceBetweenCenters) / 2.0;
		contactNormal /= distanceBetweenCenters;

		ballA->position += contactNormal * correctionLength;
		ballB->position -= contactNormal * correctionLength;

		double totalMass = ballA->Mass() + ballB->Mass();
		double relativeVelocity = (ballB->velocity - ballA->velocity) | contactNormal;

		double coefficientOfRestitution = ballA->GetBounceFactor() * ballB->GetBounceFactor();

		double impulseA = (1.0 + coefficientOfRestitution) * ballB->Mass() * relativeVelocity / totalMass;
		double impulseB = (1.0 + coefficientOfRestitution) * ballA->Mass() * relativeVelocity / totalMass;

		ballA->velocity += contactNormal * impulseA;
		ballB->velocity -= contactNormal * impulseB;

		return true;
	}

	return false;
}