#pragma once

#include "SDL.h"
#include "DrawHelper.h"
#include "Engine.h"

class App
{
public:
	App();
	virtual ~App();

	bool Setup();
	bool Shutdown();

	int Run();

private:

	bool HandleKeyboard();

	DrawHelper drawHelper;
	PlanarPhysics::Engine engine;
};