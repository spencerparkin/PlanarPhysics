#include "Wall.h"
#include "Math/GeometricAlgebra/PScalar2D.h"

using namespace PlanarPhysics;

Wall::Wall()
{
}

/*virtual*/ Wall::~Wall()
{
}

/*static*/ PlanarObject::Type Wall::StaticType()
{
	return Type::WALL;
}

/*static*/ Wall* Wall::Create()
{
	return new Wall();
}

/*virtual*/ PlanarObject::Type Wall::GetType() const
{
	return Type::WALL;
}

/*virtual*/ void Wall::Integrate(double deltaTime)
{
	// Walls don't move.
}

/*virtual*/ void Wall::AccumulateForces(const Engine* engine)
{
	// Walls have infinite mass and so they just can't be moved.
}

Vector2D Wall::Normal() const
{
	return ((this->lineSeg.vertexB - this->lineSeg.vertexA) * PScalar2D(1.0)).Normalized();
}

/*virtual*/ void Wall::CalcBoundingBox(BoundingBox& box) const
{
	box.min = this->lineSeg.vertexA;
	box.max = box.min;
	box.ExpandToIncludePoint(this->lineSeg.vertexB);
}

/*virtual*/ bool Wall::IsStatic() const
{
	return true;
}