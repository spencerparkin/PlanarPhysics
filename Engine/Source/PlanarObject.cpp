#include "PlanarObject.h"

using namespace PlanarPhysics;

PlanarObject::PlanarObject()
{
	this->nextObject = nullptr;
	this->prevObject = nullptr;
}

/*virtual*/ PlanarObject::~PlanarObject()
{
}

/*virtual*/ void PlanarObject::AccumulateForces()
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