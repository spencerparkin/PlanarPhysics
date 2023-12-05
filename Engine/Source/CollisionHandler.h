#pragma once

#include "Common.h"

namespace PlanarPhysics
{
	class PlanarObject;

	class PLANAR_PHYSICS_API CollisionHandler
	{
	public:
		CollisionHandler();
		virtual ~CollisionHandler();

		virtual void HandleCollision(PlanarObject* objectA, PlanarObject* objectB) = 0;
	};
}