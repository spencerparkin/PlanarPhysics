#include "RBodyRBodyCH.h"
#include "PlanarObjects/RigidBody.h"

using namespace PlanarPhysics;

RBodyRBodyCH::RBodyRBodyCH()
{
}

/*virtual*/ RBodyRBodyCH::~RBodyRBodyCH()
{
}

/*virtual*/ void RBodyRBodyCH::HandleCollision(PlanarObject* objectA, PlanarObject* objectB)
{
	auto bodyA = dynamic_cast<RigidBody*>(objectA);
	auto bodyB = dynamic_cast<RigidBody*>(objectB);

	if (!bodyA || !bodyB)
		return;

	std::vector<Contact> contactArray;

	//...
}