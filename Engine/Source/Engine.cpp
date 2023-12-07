#include "Engine.h"
#include "CollisionHandler.h"
#include "CollisionHandlers/BallBallCH.h"
#include "CollisionHandlers/BallWallCH.h"
#include "CollisionHandlers/RigidBodyWallCH.h"
#include "CollisionHandlers/RBodyRBodyCH.h"
#include "PlanarObjects/Ball.h"
#include "PlanarObjects/Wall.h"
#include "PlanarObjects/RigidBody.h"

using namespace PlanarPhysics;

Engine::Engine()
{
	this->accelerationDueToGravity = Vector2D(0.0, -9.8);
	this->currentTime = 0.0;
	this->maxDeltaTime = 0.00025;
	this->planarObjectArray = new std::vector<PlanarObject*>();
	
	const int numTypes = (int)PlanarObject::Type::NUM_TYPES;
	::memset(this->collisionHandlerMatrix, 0, sizeof(CollisionHandler*[numTypes][numTypes]));

	this->collisionHandlerMatrix[(int)Ball::StaticType()][(int)Ball::StaticType()] = new BallBallCH();
	this->collisionHandlerMatrix[(int)Ball::StaticType()][(int)Wall::StaticType()] = new BallWallCH();
	this->collisionHandlerMatrix[(int)Wall::StaticType()][(int)Ball::StaticType()] = new BallWallCH();
	this->collisionHandlerMatrix[(int)RigidBody::StaticType()][(int)Wall::StaticType()] = new RigidBodyWallCH();
	this->collisionHandlerMatrix[(int)Wall::StaticType()][(int)RigidBody::StaticType()] = new RigidBodyWallCH();
	this->collisionHandlerMatrix[(int)RigidBody::StaticType()][(int)RigidBody::StaticType()] = new RBodyRBodyCH();
}

/*virtual*/ Engine::~Engine()
{
	this->Clear();

	delete this->planarObjectArray;

	const int numTypes = (int)PlanarObject::Type::NUM_TYPES;
	for (int i = 0; i < numTypes; i++)
		for (int j = 0; j < numTypes; j++)
			delete this->collisionHandlerMatrix[i][j];
}

void Engine::SetWorldBox(const BoundingBox& worldBox)
{
	this->boxTree.treeBox = worldBox;
	this->boxTree.Clear();
}

const BoundingBox& Engine::GetWorldBox() const
{
	return this->boxTree.treeBox;
}

const std::vector<PlanarObject*>& Engine::GetPlanarObjectArray() const
{
	return *this->planarObjectArray;
}

void Engine::Clear()
{
	this->boxTree.Clear();

	for (PlanarObject* object : *this->planarObjectArray)
		object->DeleteSelf();

	this->planarObjectArray->clear();
}

void Engine::Tick()
{
	if (currentTime == 0.0)
	{
		currentTime = double(::clock()) / double(CLOCKS_PER_SEC);
		return;
	}
	
	double presentTime = double(::clock()) / double(CLOCKS_PER_SEC);
	double elapsedTime = presentTime - currentTime;
	if (elapsedTime > 0.5)
	{
		currentTime = presentTime;
		return;		// This is to prevent debugger breaks from producing very large time-steps.
	}

	while (currentTime < presentTime)
	{
		double deltaTime = this->maxDeltaTime;
		if (currentTime + deltaTime > presentTime)
			deltaTime = presentTime - currentTime;

		for (PlanarObject* object : *this->planarObjectArray)
			object->AdvanceBegin();

		for (PlanarObject* object : *this->planarObjectArray)
			object->AccumulateForces(this);

		for (PlanarObject* object : *this->planarObjectArray)
			object->Integrate(deltaTime);

		for (PlanarObject* object : *this->planarObjectArray)
			object->UpdateBoxTreeLocation(&this->boxTree);

		// TODO: This is still very slow?  Profile to figure out where we're spending so much time.
		std::unordered_set<uint64_t> pairSet;
		for (PlanarObject* objectA : *this->planarObjectArray)
		{
			this->boxTree.ForAllOverlaps(objectA, [this, objectA, &pairSet](BoxTree::Member* member) {
				auto objectB = dynamic_cast<PlanarObject*>(member);
				if (objectB != objectA)
				{
					uint64_t key = 0;

					if (uintptr_t(objectA) < uintptr_t(objectB))
						key = uint64_t(objectA) | (uint64_t(objectB) << 32);
					else
						key = uint64_t(objectB) | (uint64_t(objectA) << 32);

					if (pairSet.find(key) == pairSet.end())
					{
						pairSet.insert(key);

						CollisionHandler* handler = this->collisionHandlerMatrix[(int)objectA->GetType()][(int)objectB->GetType()];
						if (handler)
							handler->HandleCollision(objectA, objectB);
					}
				}
			});
		}

		for (PlanarObject* object : *this->planarObjectArray)
			object->AdvanceEnd();

		currentTime += deltaTime;
	}
}