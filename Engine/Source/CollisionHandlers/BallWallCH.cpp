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
	Vector2D nearstPointToBall = nearestPoint - ball->position;
	double distance = nearstPointToBall.Magnitude();
	if (distance >= ball->radius)
		return;

	Vector2D contactNormal = nearstPointToBall.Normalized();
	ball->position += contactNormal * (ball->radius - distance);

	double coeficientOfRestitution = 0.9;	// This ranges 0 to 1.
	double impulseMag = (1.0 + coeficientOfRestitution) * ::fabs(contactNormal | ball->velocity);
	Vector2D impulse = contactNormal * impulseMag;

	ball->inRestingContact = (impulseMag < 0.05);

	ball->velocity += impulse;
}