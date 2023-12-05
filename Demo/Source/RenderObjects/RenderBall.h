#pragma once

#include "RenderObject.h"
#include "PlanarObjects/Ball.h"

class RenderBall : public PlanarPhysics::Ball, public RenderObject
{
public:
	RenderBall();
	virtual ~RenderBall();

	static RenderBall* Create();

	virtual void Render(DrawHelper& drawHelper) const override;
	virtual void DeleteSelf() override;
};