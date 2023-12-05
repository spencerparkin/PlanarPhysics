#include "Engine.h"
#include "CollisionHandler.h"
#include "CollisionHandlers/BallBallCH.h"
#include "CollisionHandlers/BallWallCH.h"
#include "CollisionHandlers/RigidBodyWallCH.h"
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

const std::vector<PlanarObject*>& Engine::GetPlanarObjectArray() const
{
	return *this->planarObjectArray;
}

void Engine::Clear()
{
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

		// TODO: This is O(n^2).  Make it better.
		for (int i = 0; i < (signed)this->planarObjectArray->size(); i++)
		{
			PlanarObject* objectA = (*this->planarObjectArray)[i];

			for (int j = i + 1; j < (signed)this->planarObjectArray->size(); j++)
			{
				PlanarObject* objectB = (*this->planarObjectArray)[j];

				CollisionHandler* handler = collisionHandlerMatrix[(int)objectA->GetType()][(int)objectB->GetType()];
				if (handler)
					handler->HandleCollision(objectA, objectB);
			}
		}

		for (PlanarObject* object : *this->planarObjectArray)
			object->AdvanceEnd();

		currentTime += deltaTime;
	}
}