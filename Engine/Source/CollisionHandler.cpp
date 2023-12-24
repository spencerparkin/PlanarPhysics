#include "CollisionHandler.h"

using namespace PlanarPhysics;

CollisionHandler::CollisionHandler()
{
	this->coefficientOfRestitution = 0.9;
}

/*virtual*/ CollisionHandler::~CollisionHandler()
{
}