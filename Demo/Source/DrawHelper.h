#pragma once

#include "PlanarPhysics/Math/Utilities/BoundingBox.h"
#include "PlanarPhysics/Math/GeometricAlgebra/Vector2D.h"
#include "SDL3/SDL.h"

class DrawHelper
{
public:
	DrawHelper();
	virtual ~DrawHelper();

	void BeginRender();
	void EndRender();
	void DrawLine(const PlanarPhysics::Vector2D& pointA, const PlanarPhysics::Vector2D& pointB, int r, int g, int b);
	void DrawCircle(const PlanarPhysics::Vector2D& center, double radius, int r, int g, int b, int segments = 12);
	PlanarPhysics::Vector2D WorldToScreen(const PlanarPhysics::Vector2D& worldPoint) const;
	PlanarPhysics::Vector2D WorldFromScreen(const PlanarPhysics::Vector2D& screenPoint) const;

	SDL_Window* window;
	SDL_Renderer* renderer;
	PlanarPhysics::BoundingBox worldBox;
	PlanarPhysics::BoundingBox worldBoxFit;
	PlanarPhysics::BoundingBox windowBox;
};