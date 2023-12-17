#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class RigidBodyBallCH : public CollisionHandler
	{
	public:
		RigidBodyBallCH();
		virtual ~RigidBodyBallCH();

		virtual void HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}