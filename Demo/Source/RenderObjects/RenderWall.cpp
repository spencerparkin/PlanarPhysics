#include "RenderWall.h"
#include "DrawHelper.h"

using namespace PlanarPhysics;

RenderWall::RenderWall()
{
}

/*virtual*/ RenderWall::~RenderWall()
{
}

/*static*/ RenderWall* RenderWall::Create()
{
	return new RenderWall();
}

/*virtual*/ PlanarObject* RenderWall::CreateNew() const
{
	return new RenderWall();
}

/*virtual*/ void RenderWall::Render(DrawHelper& drawHelper) const
{
	drawHelper.DrawLine(this->lineSeg.vertexA, this->lineSeg.vertexB, this->r, this->b, this->g);
}

/*virtual*/ void RenderWall::DeleteSelf()
{
	delete this;
}