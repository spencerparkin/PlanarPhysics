#pragma once

#include "Common.h"
#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class PlanarObject;

	class PLANAR_PHYSICS_API CollisionHandler
	{
	public:
		CollisionHandler();
		virtual ~CollisionHandler();

		virtual bool HandleCollision(PlanarObject* objectA, PlanarObject* objectB) = 0;
	};
}