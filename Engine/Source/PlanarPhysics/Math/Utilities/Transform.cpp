#include "Transform.h"
#include "LineSegment.h"

using namespace PlanarPhysics;

Transform::Transform()
{
	this->Identity();
}

Transform::Transform(const Transform& transform)
{
	this->translation = transform.translation;
	this->rotation = transform.rotation;
	this->scale = transform.scale;
}

/*virtual*/ Transform::~Transform()
{
}

void Transform::Identity()
{
	this->translation = Vector2D(0.0, 0.0);
	this->rotation = Rotor2D(1.0, 0.0);
	this->scale = 1.0;
}

void Transform::operator=(const Transform& transform)
{
	this->translation = transform.translation;
	this->rotation = transform.rotation;
	this->scale = transform.scale;
}

Vector2D Transform::TransformVector(const Vector2D& vector) const
{
	return this->rotation * (vector * this->scale) * this->rotation.Reversed();
}

Vector2D Transform::TransformPoint(const Vector2D& vector) const
{
	return vector * this->scale * this->rotation + this->translation;
}

LineSegment Transform::TransformLineSegment(const LineSegment& lineSeg) const
{
	return LineSegment(this->TransformPoint(lineSeg.vertexA), this->TransformPoint(lineSeg.vertexB));
}

void Transform::Interpolate(const Transform& transformA, const Transform& transformB, double alpha)
{
	this->translation = transformA.translation + alpha * (transformB.translation - transformA.translation);
	this->scale = transformA.scale + alpha * (transformB.scale - transformA.scale);
	this->rotation.Interpolate(transformA.rotation, transformB.rotation, alpha);
}

namespace PlanarPhysics
{
	Transform operator*(const Transform& transformA, const Transform& transformB)
	{
		Transform result;
		
		result.translation = transformB.TransformPoint(transformA.translation);
		result.rotation = transformA.rotation * transformB.rotation;
		result.scale = transformA.scale * transformB.scale;

		return result;
	}
}