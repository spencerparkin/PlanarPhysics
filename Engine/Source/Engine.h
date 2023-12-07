#pragma once

#include "Common.h"
#include "PlanarObject.h"
#include "Math/GeometricAlgebra/Vector2D.h"
#include "BoxTree.h"

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
			T* object = T::Create();
			this->planarObjectArray->push_back(object);
			return object;
		}

		const std::vector<PlanarObject*>& GetPlanarObjectArray() const;

		void Clear();
		void Tick();
		void SetWorldBox(const BoundingBox& worldBox);
		const BoundingBox& GetWorldBox() const;

		Vector2D accelerationDueToGravity;

	private:

		CollisionHandler* collisionHandlerMatrix[(int)PlanarObject::Type::NUM_TYPES][(int)PlanarObject::Type::NUM_TYPES];
		std::vector<PlanarObject*>* planarObjectArray;
		BoxTree boxTree;
		double currentTime;
		double maxDeltaTime;
	};
}