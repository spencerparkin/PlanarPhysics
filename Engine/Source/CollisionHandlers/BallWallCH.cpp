#include "BallWallCH.h"
#include "PlanarObjects/Ball.h"
#include "PlanarObjects/Wall.h"

using namespace PlanarPhysics;

BallWallCH::BallWallCH()
{
}

/*virtual*/ BallWallCH::~BallWallCH()
{
}

/*virtual*/ void BallWallCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	Ball* ball = dynamic_cast<Ball*>(objectA);
	const Wall* wall = dynamic_cast<Wall*>(objectB);

	if (!ball || !wall)
	{
		ball = dynamic_cast<Ball*>(objectB);
		wall = dynamic_cast<Wall*>(objectA);

		if (!ball || !wall)
			return;
	}

	Vector2D nearestPoint = wall->lineSeg.NearestPoint(ball->position);
	Vector2D direction = ball->position - nearestPoint;
	double distance = direction.Magnitude();
	if (distance >= ball->radius)
		return;

	Vector2D contactNormal = direction.Normalized();
	ball->position += contactNormal * (ball->radius - distance);

	double relativeVelocity = contactNormal | ball->velocity;
	ball->inRestingContact = ::fabs(relativeVelocity) < 0.05;
	if (relativeVelocity > 0.0)
		return;

	double coeficientOfRestitution = 0.9;	// This ranges 0 to 1.
	double impulseMag = (1.0 + coeficientOfRestitution) * ::fabs(relativeVelocity);
	Vector2D impulse = contactNormal * impulseMag;
	ball->velocity += impulse;
}