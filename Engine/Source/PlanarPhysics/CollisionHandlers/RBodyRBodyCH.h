#pragma once

#include "PlanarPhysics/CollisionHandler.h"
#include "PlanarPhysics/PlanarObject.h"

namespace PlanarPhysics
{
	class RBodyRBodyCH : public CollisionHandler
	{
	public:
		RBodyRBodyCH();
		virtual ~RBodyRBodyCH();

		virtual bool HandleCollision(PlanarObject* objectA, PlanarObject* objectB) override;

	private:
		
		void AddUniqueContact(const PlanarObject::Contact& contact);

		std::vector<PlanarObject::Contact> contactArray;

		double pointRadius;
	};
}