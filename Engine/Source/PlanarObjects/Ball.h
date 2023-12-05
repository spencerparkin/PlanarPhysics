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
		virtual void Integrate(double deltaTime) override;
		virtual void AccumulateForces(const Engine* engine) override;
		virtual void AdvanceBegin() override;
	
	public:
		double Area() const;
		double Mass() const;
	
		Vector2D position;
		Vector2D velocity;
		double radius;
		double density;
		Vector2D netForce;

	private:
		bool inRestingContact;
	};
}