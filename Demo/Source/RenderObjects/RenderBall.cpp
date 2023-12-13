#include "RenderBall.h"
#include "DrawHelper.h"

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

/*virtual*/ void RenderBall::Render(DrawHelper& drawHelper) const
{
	drawHelper.DrawCircle(this->position, this->radius, this->r, this->g, this->b, 32);
}

/*virtual*/ void RenderBall::DeleteSelf()
{
	delete this;
}