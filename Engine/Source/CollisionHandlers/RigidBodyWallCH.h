#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class RigidBodyWallCH : public CollisionHandler
	{
	public:
		RigidBodyWallCH();
		virtual ~RigidBodyWallCH();

		virtual bool HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}