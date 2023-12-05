#pragma once

class DrawHelper;

class RenderObject
{
public:
	RenderObject();
	virtual ~RenderObject();

	virtual void Render(DrawHelper& drawHelper) const;

	int r, g, b;
};