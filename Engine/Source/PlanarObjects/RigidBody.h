#pragma once

#include "PlanarObject.h"
#include "Math/GeometricAlgebra/Vector2D.h"
#include "Math/GeometricAlgebra/PScalar2D.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API RigidBody : public PlanarObject
	{
		friend class Engine;
		friend class RigidBodyWallCH;

	public:
		RigidBody();
		virtual ~RigidBody();

	protected:
		static Type StaticType();
		static RigidBody* Create();

		virtual Type GetType() const override;
		virtual void Integrate(double deltaTime) override;
		virtual void AccumulateForces() override;
		virtual void AdvanceBegin() override;

	public:
		void MakeShape(const std::vector<Vector2D>& pointArray, double uniformDensity);
		bool ContainsPoint(const Vector2D& point) const;
		void UpdateWorldVertexArrayIfNeeded() const;
		const std::vector<Vector2D>& GetWorldVertexArray() const;

		Vector2D position;
		PScalar2D orientation;
		Vector2D velocity;
		PScalar2D angularVelocity;

	private:
		Vector2D netForce;
		PScalar2D netTorque;
		double inertia;
		double mass;

		std::vector<Vector2D>* localVertexArray;
		mutable std::vector<Vector2D>* worldVertexArray;
		mutable bool worldVertexArrayValid;
	};
}