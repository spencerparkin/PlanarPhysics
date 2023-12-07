#include "BoundingBox.h"

using namespace PlanarPhysics;

BoundingBox::BoundingBox()
{
	this->min = Vector2D(0.0, 0.0);
	this->max = Vector2D(0.0, 0.0);
}

BoundingBox::BoundingBox(const BoundingBox& box)
{
	this->min = box.min;
	this->max = box.max;
}

BoundingBox::BoundingBox(const Vector2D& min, const Vector2D& max)
{
	this->min = min;
	this->max = max;
}

/*virtual*/ BoundingBox::~BoundingBox()
{
}

Vector2D BoundingBox::Transform(const Vector2D& point, const BoundingBox& pointSpace) const
{
	return this->PointFromUVs(pointSpace.PointToUVs(point));
}

bool BoundingBox::IsValid() const
{
	return this->min.x <= this->max.x && this->min.y <= this->max.y;
}

Vector2D BoundingBox::Center() const
{
	return (this->min + this->max) / 2.0;
}

void BoundingBox::operator=(const BoundingBox& box)
{
	this->min = box.min;
	this->max = box.max;
}

double BoundingBox::Width() const
{
	return this->max.x - this->min.x;
}

double BoundingBox::Height() const
{
	return this->max.y - this->min.y;
}

double BoundingBox::Area() const
{
	return this->Width() * this->Height();
}

double BoundingBox::AspectRatio() const
{
	return this->Width() / this->Height();
}

bool BoundingBox::ContainsPoint(const Vector2D& point, double thickness /*= PLNR_PHY_EPSILON*/) const
{
	if (!(-thickness / 2.0 + this->min.x <= point.x && point.x <= this->max.x + thickness / 2.0))
		return false;

	if (!(-thickness / 2.0 + this->min.y <= point.y && point.y <= this->max.y + thickness / 2.0))
		return false;

	return true;
}

bool BoundingBox::PointOnEdge(const Vector2D& point, double thickness /*= PLNR_PHY_EPSILON*/) const
{
	return this->ContainsPoint(point, thickness) && !this->PointIsInterior(point, thickness);
}

bool BoundingBox::PointIsInterior(const Vector2D& point, double thickness /*= PLNR_PHY_EPSILON*/) const
{
	if (!(thickness / 2.0 + this->min.x < point.x && point.x < this->max.x - thickness / 2.0))
		return false;

	if (!(thickness / 2.0 + this->min.y < point.y && point.y < this->max.y - thickness / 2.0))
		return false;

	return true;
}

void BoundingBox::Merge(const BoundingBox& boxA, const BoundingBox& boxB)
{
	this->min = boxA.min;
	this->max = boxA.max;
	this->ExpandToIncludePoint(boxB.min);
	this->ExpandToIncludePoint(boxB.max);
}

bool BoundingBox::Intersect(const BoundingBox& boxA, const BoundingBox& boxB)
{
	this->min.x = PLNR_PHY_MAX(boxA.min.x, boxB.min.x);
	this->min.y = PLNR_PHY_MAX(boxA.min.y, boxB.min.y);

	this->max.x = PLNR_PHY_MIN(boxA.max.x, boxB.max.x);
	this->max.y = PLNR_PHY_MIN(boxA.max.y, boxB.max.y);

	return this->IsValid();
}

void BoundingBox::MatchAspectRatio(double aspectRatio, MatchMethod matchMethod)
{
	double currentAspectRatio = this->AspectRatio();

	switch (matchMethod)
	{
		case MatchMethod::EXPAND:
		{
			if (currentAspectRatio < aspectRatio)
			{
				double delta = (this->Height() * aspectRatio - this->Width()) / 2.0;
				this->min.x -= delta;
				this->max.x += delta;
			}
			else if (currentAspectRatio > aspectRatio)
			{
				double delta = (this->Width() / aspectRatio - this->Height()) / 2.0;
				this->min.y -= delta;
				this->max.y += delta;
			}

			break;
		}
		case MatchMethod::CONTRACT:
		{
			if (currentAspectRatio < aspectRatio)
			{
				double delta = (this->Height() - this->Width() / aspectRatio) / 2.0;
				this->min.y += delta;
				this->max.y -= delta;
			}
			else if (currentAspectRatio > aspectRatio)
			{
				double delta = (this->Width() - this->Height() * aspectRatio) / 2.0;
				this->min.x += delta;
				this->max.x -= delta;
			}

			break;
		}
	}
}

Vector2D BoundingBox::PointToUVs(const Vector2D& point) const
{
	return Vector2D(
		(point.x - this->min.x) / this->Width(),
		(point.y - this->min.y) / this->Height()
	);
}

Vector2D BoundingBox::PointFromUVs(const Vector2D& uvs) const
{
	return Vector2D(
		this->min.x + uvs.x * this->Width(),
		this->min.y + uvs.y * this->Height()
	);
}

void BoundingBox::ExpandToIncludePoint(const Vector2D& point)
{
	this->min.x = PLNR_PHY_MIN(this->min.x, point.x);
	this->min.y = PLNR_PHY_MIN(this->min.y, point.y);
	this->max.x = PLNR_PHY_MAX(this->max.x, point.x);
	this->max.y = PLNR_PHY_MAX(this->max.y, point.y);
}

void BoundingBox::Split(BoundingBox& boxA, BoundingBox& boxB) const
{
	if (this->Width() > this->Height())
	{
		boxA.min.x = this->min.x;
		boxA.max.x = (this->min.x + this->max.x) / 2.0;
		boxB.min.x = (this->min.x + this->max.x) / 2.0;
		boxB.max.x = this->max.x;

		boxA.min.y = this->min.y;
		boxA.max.y = this->max.y;
		boxB.min.y = this->min.y;
		boxB.max.y = this->max.y;
	}
	else
	{
		boxA.min.y = this->min.y;
		boxA.max.y = (this->min.y + this->max.y) / 2.0;
		boxB.min.y = (this->min.y + this->max.y) / 2.0;
		boxB.max.y = this->max.y;

		boxA.min.x = this->min.x;
		boxA.max.x = this->max.x;
		boxB.min.x = this->min.x;
		boxB.max.x = this->max.x;
	}
}

bool BoundingBox::OverlapsWith(const BoundingBox& box) const
{
	BoundingBox intersection;
	return intersection.Intersect(*this, box);
}

bool BoundingBox::ContainsBox(const BoundingBox& box) const
{
	return this->ContainsPoint(box.min) && this->ContainsPoint(box.max);
}