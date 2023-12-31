#pragma once

#include "Common.h"
#include "PlanarObject.h"
#include "Math/GeometricAlgebra/Vector2D.h"
#include "CollisionHandler.h"
#include "BoxTree.h"

namespace PlanarPhysics
{
	class Wall;

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
		virtual double Tick();
		void SetWorldBox(const BoundingBox& worldBox);
		const BoundingBox& GetWorldBox() const;
		void ConsolidateWalls();

		Vector2D accelerationDueToGravity;

		struct CollisionEvent
		{
			PlanarObject* objectA;
			PlanarObject* objectB;

			template<typename Ta, typename Tb>
			bool Cast(Ta*& givenA, Tb*& givenB)
			{
				givenA = dynamic_cast<Ta*>(this->objectA);
				givenB = dynamic_cast<Tb*>(this->objectB);
				if (givenA && givenB)
					return true;

				givenA = dynamic_cast<Ta*>(this->objectB);
				givenB = dynamic_cast<Tb*>(this->objectA);
				if (givenA && givenB)
					return true;

				return false;
			}
		};

		void EnqueueCollisionEvent(const CollisionEvent& event);
		bool DequeueCollisionEvent(CollisionEvent& event);
		int GetCollisionEventQueueSize() const;
		void ClearCollisionEventQueue();

	private:

		Wall* MergeWalls(const Wall* wallA, const Wall* wallB);

		CollisionHandler* collisionHandlerMatrix[(int)PlanarObject::Type::NUM_TYPES][(int)PlanarObject::Type::NUM_TYPES];
		std::vector<CollisionHandler*>* collisionHandlerArray;
		std::vector<PlanarObject*>* planarObjectArray;
		std::list<CollisionEvent>* collisionEventQueue;
		BoxTree boxTree;
		double currentTime;
		double maxDeltaTime;
	};
}