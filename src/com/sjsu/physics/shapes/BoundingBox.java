package com.sjsu.physics.shapes;

import com.sjsu.physics.utils.Vector2;

/**
 * A BoundingBox used for collision detection.
 * The bounding box needs to fully enclose all points of the rigidBody it describes
 * Greatly simplifies rough collision detection
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
