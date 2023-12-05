#pragma once

#include "Common.h"
#include "PlanarObject.h"

namespace PlanarPhysics
{
	class CollisionHandler;

	class PLANAR_PHYSICS_API Engine
	{
	public:
		Engine();
		virtual ~Engine();

		template<typename T>
		T* AddPlanarObject()
		{
			PlanarObject* object = T::Create();
			this->planarObjectArray->push_back(object);
			return object;
		}

		void Clear();
		void Tick();

	private:

		CollisionHandler* collisionHandlerMatrix[(int)PlanarObject::Type::NUM_TYPES][(int)PlanarObject::Type::NUM_TYPES];
		std::vector<PlanarObject*>* planarObjectArray;
		double currentTime;
		double maxDeltaTime;
	};
}