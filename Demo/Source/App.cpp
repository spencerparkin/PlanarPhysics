#include "App.h"
#include "Random.h"
#include "RenderObject.h"
#include "RenderObjects/RenderBall.h"
#include "RenderObjects/RenderRigidBody.h"
#include "RenderObjects/RenderWall.h"

using namespace PlanarPhysics;

App::App()
{
}

/*virtual*/ App::~App()
{
}

bool App::Setup()
{
	Random::Seed(0);

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		fprintf(stderr, "Error: %s", SDL_GetError());
		return false;
	}

	if (SDL_CreateWindowAndRenderer(1000, 800, SDL_WINDOW_RESIZABLE, &this->drawHelper.window, &this->drawHelper.renderer) < 0)
	{
		fprintf(stderr, "Error: %s", SDL_GetError());
		return false;
	}

	SDL_SetWindowTitle(drawHelper.window, "Planar Physics Engine Demo");

	RenderWall* bottomWall = this->engine.AddPlanarObject<RenderWall>();
	bottomWall->lineSeg.vertexA = Vector2D(-49.5, -49.5);
	bottomWall->lineSeg.vertexB = Vector2D(49.5, -49.5);

	RenderWall* leftWall = this->engine.AddPlanarObject<RenderWall>();
	leftWall->lineSeg.vertexA = Vector2D(-49.5, 49.5);
	leftWall->lineSeg.vertexB = Vector2D(-49.5, -49.5);

	RenderWall* rightWall = this->engine.AddPlanarObject<RenderWall>();
	rightWall->lineSeg.vertexA = Vector2D(49.5, -49.5);
	rightWall->lineSeg.vertexB = Vector2D(49.5, 49.5);

	return true;
}

bool App::Shutdown()
{
	this->engine.Clear();

	SDL_DestroyRenderer(this->drawHelper.renderer);
	SDL_DestroyWindow(this->drawHelper.window);
	SDL_Quit();

	return true;
}

int App::Run()
{
	while (this->HandleKeyboard())
	{
		this->engine.Tick();

		this->drawHelper.BeginRender();

		const std::vector<PlanarObject*>& planarObjectArray = this->engine.GetPlanarObjectArray();
		for (const PlanarObject* planarObject : planarObjectArray)
		{
			const RenderObject* renderObject = dynamic_cast<const RenderObject*>(planarObject);
			if (renderObject)
				renderObject->Render(this->drawHelper);
		}

		this->drawHelper.EndRender();
	}

	return 0;
}

bool App::HandleKeyboard()
{
	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		switch (event.type)
		{
			case SDL_QUIT:
			{
				return false;
			}
			case SDL_KEYDOWN:
			case SDL_KEYUP:
			{
				if (event.type == SDL_KEYUP)
				{
					switch (event.key.keysym.scancode)
					{
						case SDL_SCANCODE_B:
						{
							RenderBall* ball = this->engine.AddPlanarObject<RenderBall>();
							ball->position = Vector2D(0.0, 15.0);
							ball->velocity = Random::Vector(2.0, 4.0);
							ball->density = 1.0;
							ball->r = Random::Integer(0, 255);
							ball->g = Random::Integer(0, 255);
							ball->b = Random::Integer(0, 255);
							break;
						}
						case SDL_SCANCODE_R:
						{
							RenderRigidBody* body = this->engine.AddPlanarObject<RenderRigidBody>();
							body->position = Vector2D(0.0, 15.0);
							body->velocity = Random::Vector(2.0, 4.0);
							body->orientation.z = Random::Number(0.0, 2.0 * PLNR_PHY_PI);
							body->angularVelocity.z = Random::Number(-PLNR_PHY_PI / 3.0, PLNR_PHY_PI / 3.0);
							body->r = Random::Integer(0, 255);
							body->g = Random::Integer(0, 255);
							body->b = Random::Integer(0, 255);

							std::vector<Vector2D> pointArray;
							pointArray.push_back(Vector2D(-3.0, -1.0));
							pointArray.push_back(Vector2D(3.0, -1.0));
							pointArray.push_back(Vector2D(3.0, 1.0));
							pointArray.push_back(Vector2D(-3.0, 1.0));

							body->MakeShape(pointArray, 1.0);
							break;
						}
					}
				}
			}
		}
	}

	return true;
}