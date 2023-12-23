#include "Ball.h"
#include "Engine.h"

using namespace PlanarPhysics;

Ball::Ball()
{
	this->radius = 1.0;
	this->density = 1.0;
	this->inRestingContact = false;
}

/*virtual*/ Ball::~Ball()
{
}

/*static*/ PlanarObject::Type Ball::StaticType()
{
	return Type::BALL;
}

/*static*/ Ball* Ball::Create()
{
	return new Ball();
}

/*virtual*/ PlanarObject* Ball::CreateNew() const
{
	return new Ball();
}

/*virtual*/ PlanarObject::Type Ball::GetType() const
{
	return Type::BALL;
}

double Ball::Area() const
{
	return PLNR_PHY_PI * this->radius * this->radius;
}

double Ball::Mass() const
{
	return this->density * this->Area();
}

double Ball::Inertia() const
{
	return PLNR_PHY_PI * (this->radius * this->radius * this->radius * this->radius) / 4.0;
}

/*virtual*/ void Ball::Integrate(double deltaTime)
{
	Vector2D acceleration = this->netForce / this->Mass();
	this->velocity += acceleration * deltaTime;
	this->position += this->velocity * deltaTime;

	if (this->velocity != Vector2D(0.0, 0.0))
		this->cachedBoundingBoxValid = false;
}

/*virtual*/ void Ball::AccumulateForces(const Engine* engine)
{
	if ((this->flags & PLNR_OBJ_FLAG_INFLUENCED_BY_GRAVITY) != 0)
	{
		this->netForce += engine->accelerationDueToGravity * this->Mass();
	}

	if (this->inRestingContact)
	{
		double frictionCoeficient = 0.5;
		Vector2D frictionForce = this->Mass() * this->velocity * -1.0 * frictionCoeficient;
		this->netForce += frictionForce;
	}
}

/*virtual*/ void Ball::AdvanceBegin()
{
	this->netForce = Vector2D(0.0, 0.0);
}

/*virtual*/ void Ball::CalcBoundingBox(BoundingBox& box) const
{
	Vector2D vector(this->radius, this->radius);
	box.min = this->position - vector;
	box.max = this->position + vector;
}