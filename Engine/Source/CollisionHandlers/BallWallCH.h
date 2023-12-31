#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class BallWallCH : public CollisionHandler
	{
	public:
		BallWallCH();
		virtual ~BallWallCH();

		virtual bool HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}