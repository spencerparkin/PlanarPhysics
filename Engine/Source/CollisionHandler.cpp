#include "CollisionHandler.h"

using namespace PlanarPhysics;

CollisionHandler::CollisionHandler()
{
	this->coeficientOfRestitution = 0.9;
}

/*virtual*/ CollisionHandler::~CollisionHandler()
{
}