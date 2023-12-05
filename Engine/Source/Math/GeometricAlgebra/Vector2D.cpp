#include "Vector2D.h"
#include "Rotor2D.h"
#include "PScalar2D.h"

using namespace PlanarPhysics;

Vector2D::Vector2D()
{
	this->x = 0.0;
	this->y = 0.0;
}

Vector2D::Vector2D(const Vector2D& vector)
{
	this->x = vector.x;
	this->y = vector.y;
}

Vector2D::Vector2D(double x, double y)
{
	this->x = x;
	this->y = y;
}

/*virtual*/ Vector2D::~Vector2D()
{
}

void Vector2D::operator=(const Vector2D& vector)
{
	this->x = vector.x;
	this->y = vector.y;
}

Vector2D Vector2D::Inverted() const
{
	return *this / (*this | *this);
}

Vector2D Vector2D::Normalized() const
{
	return *this / this->Magnitude();
}

double Vector2D::Magnitude() const
{
	return ::sqrt(*this | *this);
}

namespace PlanarPhysics
{
	Vector2D operator*(const Vector2D& vector, double scalar)
	{
		return Vector2D(
			vector.x * scalar,
			vector.y * scalar
		);
	}

	Vector2D operator/(const Vector2D& vector, double scalar)
	{
		return Vector2D(
			vector.x / scalar,
			vector.y / scalar
		);
	}

	Vector2D operator*(double scalar, const Vector2D& vector)
	{
		return Vector2D(
			vector.x * scalar,
			vector.y * scalar
		);
	}

	Vector2D operator+(const Vector2D& vector1, const Vector2D& vector2)
	{
		return Vector2D(
			vector1.x + vector2.x,
			vector1.y + vector2.y
		);
	}

	Vector2D operator-(const Vector2D& vector1, const Vector2D& vector2)
	{
		return Vector2D(
			vector1.x - vector2.x,
			vector1.y - vector2.y
		);
	}

	Rotor2D operator*(const Vector2D& vector1, const Vector2D& vector2)
	{
		return Rotor2D(
			vector1.x * vector2.x + vector1.y  *vector2.y,
			vector1.x * vector2.y - vector1.y * vector2.x
		);
	}

	Rotor2D operator/(const Vector2D& vector1, const Vector2D& vector2)
	{
		return vector1 * vector2.Inverted();
	}

	double operator|(const Vector2D& vector1, const Vector2D& vector2)
	{
		return vector1.x * vector2.x + vector1.y * vector2.y;
	}

	PScalar2D operator^(const Vector2D& vector1, const Vector2D& vector2)
	{
		return PScalar2D(
			vector1.x * vector2.y - vector1.y * vector2.x
		);
	}
}