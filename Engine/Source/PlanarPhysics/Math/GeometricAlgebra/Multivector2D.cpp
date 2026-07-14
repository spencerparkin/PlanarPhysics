#include "Multivector2D.h"
#include "Vector2D.h"
#include "Rotor2D.h"
#include "PScalar2D.h"

using namespace PlanarPhysics;

Multivector2D::Multivector2D()
{
	this->w = 0.0;
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

Multivector2D::Multivector2D(const Multivector2D& multivector)
{
	this->w = multivector.w;
	this->x = multivector.x;
	this->y = multivector.y;
	this->z = multivector.z;
}

Multivector2D::Multivector2D(double w, double x, double y, double z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

Multivector2D::Multivector2D(double scalar)
{
	this->w = scalar;
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

Multivector2D::Multivector2D(const Vector2D& vector)
{
	this->w = 0.0;
	this->x = vector.x;
	this->y = vector.y;
	this->z = 0.0;
}

Multivector2D::Multivector2D(const Rotor2D& rotor)
{
	this->w = rotor.w;
	this->x = 0.0;
	this->y = 0.0;
	this->z = rotor.z;
}

Multivector2D::Multivector2D(const PScalar2D& pscalar)
{
	this->w = 0.0;
	this->x = 0.0;
	this->y = 0.0;
	this->z = pscalar.z;
}

/*virtual*/ Multivector2D::~Multivector2D()
{
}

void Multivector2D::operator=(const Multivector2D& multivector)
{
	this->w = multivector.w;
	this->x = multivector.x;
	this->y = multivector.y;
	this->z = multivector.z;
}

void Multivector2D::operator=(double scalar)
{
	this->w = scalar;
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

void Multivector2D::operator=(const Vector2D& vector)
{
	this->w = 0.0;
	this->x = vector.x;
	this->y = vector.y;
	this->z = 0.0;
}

void Multivector2D::operator=(const Rotor2D& rotor)
{
	this->w = rotor.w;
	this->x = 0.0;
	this->y = 0.0;
	this->z = rotor.z;
}

void Multivector2D::operator=(const PScalar2D& pscalar)
{
	this->w = 0.0;
	this->x = 0.0;
	this->y = 0.0;
	this->z = pscalar.z;
}

Multivector2D Multivector2D::Reversed() const
{
	return Multivector2D(
		this->w,
		this->x,
		this->y,
		-this->z
	);
}

Multivector2D Multivector2D::Inverted() const
{
	// TODO: Write this!  Of course, some multivectors are singular, so we'll return zero in that case, I guess.
	return Multivector2D(0.0, 0.0, 0.0, 0.0);
}

namespace PlanarPhysics
{
	Multivector2D operator*(const Multivector2D& multivector, double scalar)
	{
		return Multivector2D(
			multivector.w * scalar,
			multivector.x * scalar,
			multivector.y * scalar,
			multivector.z * scalar
		);
	}

	Multivector2D operator/(const Multivector2D& multivector, double scalar)
	{
		return Multivector2D(
			multivector.w / scalar,
			multivector.x / scalar,
			multivector.y / scalar,
			multivector.z / scalar
		);
	}

	Multivector2D operator*(double scalar, const Multivector2D& multivector)
	{
		return Multivector2D(
			multivector.w * scalar,
			multivector.x * scalar,
			multivector.y * scalar,
			multivector.z * scalar
		);
	}

	Multivector2D operator+(const Multivector2D& multivector1, const Multivector2D& multivector2)
	{
		return Multivector2D(
			multivector1.w + multivector2.w,
			multivector1.x + multivector2.x,
			multivector1.y + multivector2.y,
			multivector1.z + multivector2.z
		);
	}

	Multivector2D operator-(const Multivector2D& multivector1, const Multivector2D& multivector2)
	{
		return Multivector2D(
			multivector1.w - multivector2.w,
			multivector1.x - multivector2.x,
			multivector1.y - multivector2.y,
			multivector1.z - multivector2.z
		);
	}

	Multivector2D operator*(const Multivector2D& multivector1, const Multivector2D& multivector2)
	{
		return Multivector2D(
			multivector1.w * multivector2.w + multivector1.x * multivector2.x + multivector1.y * multivector2.y - multivector1.z * multivector2.z,
			multivector1.w * multivector2.x + multivector1.x * multivector2.w - multivector1.y * multivector2.z + multivector1.z * multivector2.y,
			multivector1.w * multivector2.y + multivector1.x * multivector2.z + multivector1.y * multivector2.w - multivector1.z * multivector2.x,
			multivector1.w * multivector2.z + multivector1.x * multivector2.y - multivector1.y * multivector2.x + multivector1.z * multivector2.w
		);
	}

	Multivector2D operator/(const Multivector2D& multivector1, const Multivector2D& multivector2)
	{
		return multivector1 * multivector2.Inverted();
	}

	Multivector2D operator|(const Multivector2D& multivector1, const Multivector2D& multivector2)
	{
		return Multivector2D(
			multivector1.w * multivector2.w + multivector1.x * multivector2.x + multivector1.y * multivector2.y - multivector1.z * multivector2.z,
			multivector1.w * multivector2.x + multivector1.x * multivector2.w - multivector1.y * multivector2.z + multivector1.z * multivector2.y,
			multivector1.w * multivector2.y + multivector1.x * multivector2.z + multivector1.y * multivector2.w - multivector1.z * multivector2.x,
			multivector1.w * multivector2.z + multivector1.z * multivector2.w
		);
	}

	Multivector2D operator^(const Multivector2D& multivector1, const Multivector2D& multivector2)
	{
		return Multivector2D(
			multivector1.w * multivector2.w,
			multivector1.w * multivector2.x + multivector1.x * multivector2.w,
			multivector1.w * multivector2.y + multivector1.y * multivector2.w,
			multivector1.w * multivector2.z + multivector1.x * multivector2.y - multivector1.y * multivector2.x + multivector1.z + multivector2.w
		);
	}
}