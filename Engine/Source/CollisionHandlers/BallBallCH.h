#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class BallBallCH : public CollisionHandler
	{
	public:
		BallBallCH();
		virtual ~BallBallCH();

		virtual void HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}