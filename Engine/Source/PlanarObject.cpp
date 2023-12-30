#include "PlanarObject.h"

using namespace PlanarPhysics;

PlanarObject::PlanarObject()
{
	this->flags = 0;
	this->bounceFactor = 1.0;
}

/*virtual*/ PlanarObject::~PlanarObject()
{
}

uint32_t PlanarObject::GetFlags() const
{
	return this->flags;
}

void PlanarObject::SetFlags(uint32_t flags)
{
	this->flags = flags;
}

double PlanarObject::GetBounceFactor() const
{
	return this->bounceFactor;
}

void PlanarObject::SetBounceFactor(double bounceFactor)
{
	this->bounceFactor = bounceFactor;
}

/*virtual*/ void PlanarObject::AccumulateForces(const Engine* engine)
{
}

/*virtual*/ void PlanarObject::Integrate(double deltaTime)
{
}

/*virtual*/ void PlanarObject::AdvanceBegin()
{
}

/*virtual*/ void PlanarObject::AdvanceEnd()
{
}

/*virtual*/ void PlanarObject::DeleteSelf()
{
	delete this;
}

/*virtual*/ bool PlanarObject::IsStatic() const
{
	return false;
}

/*virtual*/ void PlanarObject::CollisionOccurredWith(PlanarObject* planarObject, Engine* engine)
{
}