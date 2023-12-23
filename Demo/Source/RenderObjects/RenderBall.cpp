#include "RenderBall.h"
#include "DrawHelper.h"

using namespace PlanarPhysics;

RenderBall::RenderBall()
{
}

/*virtual*/ RenderBall::~RenderBall()
{
}

/*static*/ RenderBall* RenderBall::Create()
{
	return new RenderBall();
}

/*virtual*/ PlanarObject* RenderBall::CreateNew() const
{
	return new RenderBall();
}

/*virtual*/ void RenderBall::Render(DrawHelper& drawHelper) const
{
	// TODO: Draw spokes when we add angular velocity and orientation to the ball.
	drawHelper.DrawCircle(this->position, this->radius, this->r, this->g, this->b, 32);
}

/*virtual*/ void RenderBall::DeleteSelf()
{
	delete this;
}