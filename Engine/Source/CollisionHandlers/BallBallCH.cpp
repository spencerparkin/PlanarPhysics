#include "BallBallCH.h"
#include "PlanarObjects/Ball.h"

using namespace PlanarPhysics;

BallBallCH::BallBallCH()
{
}

/*virtual*/ BallBallCH::~BallBallCH()
{
}

/*virtual*/ void BallBallCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	Ball* ballA = dynamic_cast<Ball*>(objectA);
	Ball* ballB = dynamic_cast<Ball*>(objectB);

	if (!ballA || !ballB)
		return;

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

		double impulseA = (1.0 + this->coefficientOfRestitution) * ballB->Mass() * relativeVelocity / totalMass;
		double impulseB = (1.0 + this->coefficientOfRestitution) * ballA->Mass() * relativeVelocity / totalMass;

		ballA->velocity += contactNormal * impulseA;
		ballB->velocity -= contactNormal * impulseB;
	}
}