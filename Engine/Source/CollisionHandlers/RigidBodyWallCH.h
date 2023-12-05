#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class RigidBodyWallCH : public CollisionHandler
	{
	public:
		RigidBodyWallCH();
		virtual ~RigidBodyWallCH();

		virtual void HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}