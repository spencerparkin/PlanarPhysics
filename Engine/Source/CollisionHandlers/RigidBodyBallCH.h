#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class RigidBodyBallCH : public CollisionHandler
	{
	public:
		RigidBodyBallCH();
		virtual ~RigidBodyBallCH();

		virtual bool HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}