#include "Rotor2D.h"
#include "Vector2D.h"
#include "PScalar2D.h"

using namespace PlanarPhysics;

Rotor2D::Rotor2D()
{
	this->w = 0.0;
	this->z = 0.0;
}

Rotor2D::Rotor2D(const Rotor2D& rotor)
{
	this->w = rotor.w;
	this->z = rotor.z;
}

Rotor2D::Rotor2D(double w, double z)
{
	this->w = w;
	this->z = z;
}

/*virtual*/ Rotor2D::~Rotor2D()
{
}

void Rotor2D::operator=(const Rotor2D& rotor)
{
	this->w = rotor.w;
	this->z = rotor.z;
}

Vector2D Rotor2D::operator*(const Vector2D& vector) const
{
	return Vector2D(
		this->w * vector.x + this->z * vector.y,
		this->w * vector.x - this->z * vector.x
	);
}

Rotor2D Rotor2D::Reversed() const
{
	return Rotor2D(
		this->w,
		-this->z
	);
}

Rotor2D Rotor2D::Inverted() const
{
	return this->Reversed() / (this->w * this->w + this->z * this->z);
}

Rotor2D Rotor2D::Normalized() const
{
	return *this / this->Magnitude();
}

double Rotor2D::Magnitude() const
{
	return ::sqrt(this->w * this->w + this->z * this->z);
}

void Rotor2D::Interpolate(const Rotor2D& rotorA, const Rotor2D& rotorB, double alpha)
{
	PScalar2D logA = rotorA.Log();
	PScalar2D logB = rotorB.Log();

	PScalar2D log = logA + alpha * (logB - logA);

	*this = log.Exponent();
}

PScalar2D Rotor2D::Log() const
{
	return PScalar2D(::atan2(this->z, this->w));
}

namespace PlanarPhysics
{
	Rotor2D operator*(const Rotor2D& rotor, double scalar)
	{
		return Rotor2D(
			rotor.w * scalar,
			rotor.z * scalar
		);
	}

	Rotor2D operator/(const Rotor2D& rotor, double scalar)
	{
		return Rotor2D(
			rotor.w / scalar,
			rotor.z / scalar
		);
	}

	Rotor2D operator*(double scalar, const Rotor2D& rotor)
	{
		return Rotor2D(
			rotor.w * scalar,
			rotor.z * scalar
		);
	}

	Rotor2D operator*(const Rotor2D& rotor1, const Rotor2D& rotor2)
	{
		return Rotor2D(
			rotor1.w * rotor2.w - rotor1.z * rotor2.z,
			rotor1.w * rotor2.z + rotor1.z * rotor2.w
		);
	}

	Rotor2D operator/(const Rotor2D& rotor1, const Rotor2D& rotor2)
	{
		return rotor1 * rotor2.Inverted();
	}

	Rotor2D operator|(const Rotor2D& rotor1, const Rotor2D& rotor2)
	{
		return Rotor2D(
			rotor1.w * rotor2.w - rotor1.z * rotor2.z,
			rotor1.w * rotor2.z + rotor1.z * rotor2.w
		);
	}

	Rotor2D operator^(const Rotor2D& rotor1, const Rotor2D& rotor2)
	{
		return Rotor2D(
			rotor1.w * rotor2.w,
			rotor1.w * rotor2.z + rotor1.z * rotor2.w
		);
	}
}