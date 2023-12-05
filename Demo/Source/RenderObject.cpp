#include "RenderObject.h"

RenderObject::RenderObject()
{
	this->r = 255;
	this->g = 255;
	this->b = 255;
}

/*virtual*/ RenderObject::~RenderObject()
{
}

/*virtual*/ void RenderObject::Render(DrawHelper& drawHelper) const
{
}