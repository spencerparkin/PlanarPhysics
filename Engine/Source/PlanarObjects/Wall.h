#pragma once

#include "PlanarObject.h"
#include "Math/Utilities/LineSegment.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API Wall : public PlanarObject
	{
		friend class Engine;

	public:
		Wall();
		virtual ~Wall();

		static Type StaticType();
		static Wall* Create();

		virtual PlanarObject* CreateNew() const override;

	protected:
		virtual Type GetType() const override;
		virtual void Integrate(double deltaTime) override;
		virtual void AccumulateForces(const Engine* engine) override;
		virtual void CalcBoundingBox(BoundingBox& box) const override;
		virtual bool IsStatic() const override;

	public:
		Vector2D Normal() const;

		LineSegment lineSeg;
	};
}