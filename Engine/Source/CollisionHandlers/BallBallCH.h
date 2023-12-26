#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class BallBallCH : public CollisionHandler
	{
	public:
		BallBallCH();
		virtual ~BallBallCH();

		virtual bool HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}