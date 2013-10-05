package com.sjsu.physics.shapes;

import com.sjsu.physics.utils.Vector2;

/* 
 * A circle that can be used for rough collision detection.
 * We assume that this circle will fully enclose the rigid body that it
 * is attached to. That is all points of the rigidBody will be within this boundingCircle.
 */
public class BoundingBox
{
	private float hWidth; // half height
	private float hHeight; // half width

	public BoundingBox(float hw, float hh)
	{
		hWidth = hw;
		hHeight = hh;
	}

	public float halfHeight()
	{
		return hHeight;
	}

	public float halfWidth()
	{
		return hWidth;
	}

	public float radius()
	{
		return (Math.max(hWidth, hHeight));
	}

	public float bottomY(Vector2 center)
	{
		return center.y() + hHeight;
	}

	public float topY(Vector2 center)
	{
		return center.y() - hHeight;
	}

	public float leftX(Vector2 center)
	{
		return center.x() - hWidth;
	}

	public float rightX(Vector2 center)
	{
		return center.x() + hWidth;
	}

	public void setHalfHeight(float hh)
	{
		hHeight = hh;
	}

	public void setHalfWidth(float hw)
	{
		hWidth = hw;
	}

	@Override
	public String toString()
	{
		String s = "halfWidth: " + hWidth + "  halfHeight: " + hHeight;
		return s;
	}
}
