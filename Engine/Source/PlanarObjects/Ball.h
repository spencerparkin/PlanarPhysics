#pragma once

#include "PlanarObject.h"
#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API Ball : public PlanarObject
	{
		friend class Engine;
		friend class BallWallCH;

	public:
		Ball();
		virtual ~Ball();

	protected:
		static Type StaticType();
		static Ball* Create();

		virtual Type GetType() const override;
		virtual PlanarObject* CreateNew() const override;
		virtual void Integrate(double deltaTime) override;
		virtual void AccumulateForces(const Engine* engine) override;
		virtual void AdvanceBegin() override;
		virtual void CalcBoundingBox(BoundingBox& box) const override;

	public:
		double Area() const;
		double Mass() const;
		double Inertia() const;
	
		// TODO: Add angular velocity and a net-torque force.  Apply torque when
		//       in resting contact so that the ball can, e.g., roll across the floor.
		Vector2D position;
		Vector2D velocity;
		double radius;
		double density;
		Vector2D netForce;

	private:
		bool inRestingContact;
	};
}