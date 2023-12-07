#include "PScalar2D.h"
#include "Rotor2D.h"

using namespace PlanarPhysics;

PScalar2D::PScalar2D()
{
	this->z = 0.0;
}

PScalar2D::PScalar2D(const PScalar2D& pscalar)
{
	this->z = pscalar.z;
}

PScalar2D::PScalar2D(double z)
{
	this->z = z;
}

/*virtual*/ PScalar2D::~PScalar2D()
{
}

void PScalar2D::operator=(const PScalar2D& pscalar)
{
	this->z = pscalar.z;
}

void PScalar2D::operator*=(double scalar)
{
	this->z *= scalar;
}

void PScalar2D::operator+=(const PScalar2D& pscalar)
{
	this->z += pscalar.z;
}

void PScalar2D::operator-=(const PScalar2D& pscalar)
{
	this->z -= pscalar.z;
}

PScalar2D PScalar2D::operator-() const
{
	return PScalar2D(-this->z);
}

bool PScalar2D::operator==(const PScalar2D& pscalar) const
{
	return this->z == pscalar.z;
}

bool PScalar2D::operator!=(const PScalar2D& pscalar) const
{
	return this->z != pscalar.z;
}

PScalar2D PScalar2D::Reversed() const
{
	return PScalar2D(
		-this->z
	);
}

PScalar2D PScalar2D::Inverted() const
{
	return this->Reversed() / (this->z * this->z);
}

PScalar2D PScalar2D::Magnitude() const
{
	return ::sqrt(this->z * this->z);
}

Rotor2D PScalar2D::Exponent() const
{
	return Rotor2D(
		::cos(this->z),
		::sin(this->z)
	);
}

namespace PlanarPhysics
{
	PScalar2D operator*(const PScalar2D& pscalar, double scalar)
	{
		return PScalar2D(
			pscalar.z * scalar
		);
	}

	PScalar2D operator/(const PScalar2D& pscalar, double scalar)
	{
		return PScalar2D(
			pscalar.z / scalar
		);
	}

	PScalar2D operator*(double scalar, const PScalar2D& pscalar)
	{
		return PScalar2D(
			pscalar.z * scalar
		);
	}

	PScalar2D operator+(const PScalar2D& pscalar1, const PScalar2D& pscalar2)
	{
		return PScalar2D(
			pscalar1.z + pscalar2.z
		);
	}

	PScalar2D operator-(const PScalar2D& pscalar1, const PScalar2D& pscalar2)
	{
		return PScalar2D(
			pscalar1.z - pscalar2.z
		);
	}

	double operator*(const PScalar2D& pscalar1, const PScalar2D& pscalar2)
	{
		return -pscalar1.z * pscalar2.z;
	}

	double operator/(const PScalar2D& pscalar1, const PScalar2D& pscalar2)
	{
		return pscalar1 * pscalar2.Inverted();
	}

	double operator|(const PScalar2D& pscalar1, const PScalar2D& pscalar2)
	{
		return -pscalar1.z * pscalar2.z;
	}

	double operator^(const PScalar2D& pscalar1, const PScalar2D& pscalar2)
	{
		return 0.0;
	}
}