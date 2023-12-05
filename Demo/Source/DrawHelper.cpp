#include "DrawHelper.h"

using namespace PlanarPhysics;

DrawHelper::DrawHelper()
{
	this->window = nullptr;
	this->renderer = nullptr;
	this->worldBox.min = Vector2D(-50.0, -50.0);
	this->worldBox.max = Vector2D(50.0, 50.0);
}

/*virtual*/ DrawHelper::~DrawHelper()
{
}

void DrawHelper::BeginRender()
{
	int windowWidth = 0;
	int windowHeight = 0;
	SDL_GetWindowSize(window, &windowWidth, &windowHeight);

	this->windowBox.min.x = 0.0;
	this->windowBox.min.y = 0.0;
	this->windowBox.max.x = (double)windowWidth;
	this->windowBox.max.y = (double)windowHeight;

	this->worldBoxFit = this->worldBox;
	this->worldBoxFit.MatchAspectRatio(windowBox.AspectRatio(), BoundingBox::MatchMethod::EXPAND);

	SDL_SetRenderDrawColor(this->renderer, 0, 0, 0, 0);
	SDL_RenderClear(this->renderer);
}

void DrawHelper::EndRender()
{
	SDL_RenderPresent(this->renderer);
}

void DrawHelper::DrawLine(const Vector2D& pointA, const Vector2D& pointB, int r, int g, int b)
{
	Vector2D screenPointA = this->WorldToScreen(pointA);
	Vector2D screenPointB = this->WorldToScreen(pointB);

	SDL_SetRenderDrawColor(this->renderer, r, g, b, 0);
	SDL_RenderDrawLineF(this->renderer, (float)screenPointA.x, (float)screenPointA.y, (float)screenPointB.x, (float)screenPointB.y);
}

void DrawHelper::DrawCircle(const Vector2D& center, double radius, int r, int g, int b, int segments /*= 12*/)
{
	int pointCount = segments + 1;
	SDL_FPoint* pointArray = new SDL_FPoint[pointCount];

	for (int i = 0; i < pointCount; i++)
	{
		double angle = 2.0 * PLNR_PHY_PI * (double(i) / double(segments));
		Vector2D direction(::cos(angle), ::sin(angle));
		Vector2D worldVertex = center + radius * direction;
		Vector2D screenVertex = this->WorldToScreen(worldVertex);
		pointArray[i].x = (float)::round(screenVertex.x);
		pointArray[i].y = (float)::round(screenVertex.y);
	}

	SDL_SetRenderDrawColor(this->renderer, r, g, b, 0);
	SDL_RenderDrawLinesF(this->renderer, pointArray, pointCount);
	delete[] pointArray;
}

Vector2D DrawHelper::WorldToScreen(const Vector2D& worldPoint) const
{
	Vector2D screenPoint = this->windowBox.Transform(worldPoint, this->worldBoxFit);
	screenPoint.y = this->windowBox.Height() - screenPoint.y;
	return screenPoint;
}

Vector2D DrawHelper::WorldFromScreen(const Vector2D& screenPoint) const
{
	Vector2D flippedScreenPoint(screenPoint);
	flippedScreenPoint.y = this->windowBox.Height() - flippedScreenPoint.y;
	Vector2D worldPoint = this->worldBoxFit.Transform(flippedScreenPoint, this->windowBox);
	return worldPoint;
}