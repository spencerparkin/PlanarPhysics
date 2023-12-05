#pragma once

#include "Common.h"

namespace PlanarPhysics
{
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

	protected:
		virtual void AccumulateForces();
		virtual void Integrate(double deltaTime);
		virtual void DeleteSelf();	// May need to override this so that deletion happens in proper heap.
		virtual void AdvanceBegin();
		virtual void AdvanceEnd();

	private:
		PlanarObject* nextObject;
		PlanarObject* prevObject;
	};
}