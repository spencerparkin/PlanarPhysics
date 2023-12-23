#pragma once

#include "RenderObject.h"
#include "PlanarObjects/Wall.h"

class RenderWall : public PlanarPhysics::Wall, public RenderObject
{
public:
	RenderWall();
	virtual ~RenderWall();

	static RenderWall* Create();

	virtual PlanarObject* CreateNew() const override;
	virtual void Render(DrawHelper& drawHelper) const override;
	virtual void DeleteSelf() override;
};