#include "Ray.h"

using namespace PlanarPhysics;

Ray::Ray()
{
}

Ray::Ray(const Ray& ray)
{
	this->center = ray.center;
	this->normal = ray.normal;
}

Ray::Ray(const Vector2D& center, const Vector2D& normal)
{
	this->center = center;
	this->normal = normal;
}

/*virtual*/ Ray::~Ray()
{
}