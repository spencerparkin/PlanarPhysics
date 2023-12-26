#pragma once

#include "Common.h"
#include "Math/GeometricAlgebra/Vector2D.h"
#include "Math/Utilities/BoundingBox.h"
#include "BoxTree.h"

#define PLNR_OBJ_FLAG_INFLUENCED_BY_GRAVITY		0x00000001
#define PLNR_OBJ_FLAG_GEN_COLLISION_EVENTS		0x00000002
#define PLNR_OBJ_FLAG_CALL_COLLISION_FUNC		0x00000004

namespace PlanarPhysics
{
	class Engine;
	class BoundingBox;

	class PLANAR_PHYSICS_API PlanarObject : public BoxTree::Member
	{
		friend class Engine;

	public:
		PlanarObject();
		virtual ~PlanarObject();

		enum class Type
		{
			WALL,
			BALL,
			RIGID_BODY,
			NUM_TYPES
		};

		virtual Type GetType() const = 0;
		virtual PlanarObject* CreateNew() const = 0;
		virtual void CollisionOccurredWith(PlanarObject* planarObject);

		uint32_t GetFlags() const;
		void SetFlags(uint32_t flags);

		struct Contact
		{
			double penetrationDepth;
			Vector2D normal;
			Vector2D point;
		};

	protected:
		virtual void AccumulateForces(const Engine* engine);
		virtual void Integrate(double deltaTime);
		virtual void DeleteSelf();	// May need to override this so that deletion happens in proper heap.
		virtual void AdvanceBegin();
		virtual void AdvanceEnd();
		virtual bool IsStatic() const;

		uint32_t flags;
	};
}