#pragma once

#include "PlanarObject.h"
#include "Math/GeometricAlgebra/Vector2D.h"
#include "Math/GeometricAlgebra/PScalar2D.h"
#include "Math/Utilities/ConvexPolygon.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API RigidBody : public PlanarObject
	{
		friend class Engine;
		friend class RigidBodyWallCH;
		friend class RBodyRBodyCH;

	public:
		RigidBody();
		virtual ~RigidBody();

	protected:
		static Type StaticType();
		static RigidBody* Create();

		virtual Type GetType() const override;
		virtual void Integrate(double deltaTime) override;
		virtual void AccumulateForces(const Engine* engine) override;
		virtual void AdvanceBegin() override;
		virtual void CalcBoundingBox(BoundingBox& box) const override;

	public:
		bool MakeShape(const std::vector<Vector2D>& pointArray, double uniformDensity);
		bool ContainsPoint(const Vector2D& point) const;
		void UpdateWorldPolygonIfNeeded() const;
		const ConvexPolygon& GetWorldPolygon() const;

		Vector2D position;
		PScalar2D orientation;
		Vector2D velocity;
		PScalar2D angularVelocity;

	private:
		bool PointPenetratesConvexPolygon(const Vector2D& point, Contact& contact, double vertexRadius = 0.1) const;

		Vector2D netForce;
		PScalar2D netTorque;
		double inertia;
		double mass;
		bool inRestingContact;

		ConvexPolygon localPolygon;
		mutable ConvexPolygon worldPolygon;
		mutable bool worldPolygonValid;
	};
}