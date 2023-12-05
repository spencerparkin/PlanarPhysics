#include "RenderRigidBody.h"
#include "DrawHelper.h"

using namespace PlanarPhysics;

RenderRigidBody::RenderRigidBody()
{
}

/*virtual*/ RenderRigidBody::~RenderRigidBody()
{
}

/*static*/ RenderRigidBody* RenderRigidBody::Create()
{
	return new RenderRigidBody();
}

/*virtual*/ void RenderRigidBody::Render(DrawHelper& drawHelper) const
{
	const std::vector<Vector2D>& worldVertexArray = this->GetWorldVertexArray();

	for (int i = 0; i < (signed)worldVertexArray.size(); i++)
	{
		int j = (i + 1) % worldVertexArray.size();

		const Vector2D& vertexA = worldVertexArray[i];
		const Vector2D& vertexB = worldVertexArray[j];

		drawHelper.DrawLine(vertexA, vertexB, this->r, this->g, this->b);
	}
}

/*virtual*/ void RenderRigidBody::DeleteSelf()
{
	delete this;
}