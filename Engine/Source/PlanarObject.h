#pragma once

#include "Common.h"

#define PLNR_OBJ_FLAG_INFLUENCED_BY_GRAVITY		0x00000001

namespace PlanarPhysics
{
	class Engine;

	class PLANAR_PHYSICS_API PlanarObject
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

		uint32_t GetFlags() const;
		void SetFlags(uint32_t flags);

	protected:
		virtual void AccumulateForces(const Engine* engine);
		virtual void Integrate(double deltaTime);
		virtual void DeleteSelf();	// May need to override this so that deletion happens in proper heap.
		virtual void AdvanceBegin();
		virtual void AdvanceEnd();

		uint32_t flags;
	};
}