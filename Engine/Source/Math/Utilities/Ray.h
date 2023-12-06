#pragma once

#include "Math/GeometricAlgebra/Vector2D.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API Ray
	{
	public:
		Ray();
		Ray(const Ray& ray);
		Ray(const Vector2D& center, const Vector2D& normal);
		virtual ~Ray();

		//...

		Vector2D center;
		Vector2D normal;
	};
}