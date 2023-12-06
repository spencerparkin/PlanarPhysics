#pragma once

#include "CollisionHandler.h"

namespace PlanarPhysics
{
	class RBodyRBodyCH : public CollisionHandler
	{
	public:
		RBodyRBodyCH();
		virtual ~RBodyRBodyCH();

		virtual void HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;
	};
}