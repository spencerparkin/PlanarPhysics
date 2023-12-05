#pragma once

#include "PlanarObjects/RigidBody.h"
#include "RenderObject.h"

class RenderRigidBody : public PlanarPhysics::RigidBody, public RenderObject
{
public:
	RenderRigidBody();
	virtual ~RenderRigidBody();

	static RenderRigidBody* Create();

	virtual void Render(DrawHelper& drawHelper) const override;
	virtual void DeleteSelf() override;
};