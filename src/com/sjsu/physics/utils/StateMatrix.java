package com.sjsu.physics.utils;

public class StateMatrix
{
	private Vector2 row0; // the x axis
	private Vector2 row1; // y axis
	private Vector2 position;

	public StateMatrix(Vector2 p, float a)
	{
		position = p;
		row0 = Vector2.fromAngle(a);
		row1 = row0.rotate90();
	}

	public Vector2 rotateIntoSpaceOf(Vector2 v)
	{
		return new Vector2(v.dot(row0), v.dot(row1));
	}

	public Vector2 rotateBy(Vector2 vec)
	{
		Vector2 v0 = row0.multiplyBy(vec.x());
		Vector2 v1 = row1.multiplyBy(vec.y());

		return v0.addTo(v1);
	}

	public Vector2 TransformBy(Vector2 v)
	{
		Vector2 vec = rotateBy(v);

		return vec.addTo(position);
	}

	public Vector2 TransformIntoSpaceOf(Vector2 v)
	{
		Vector2 vec = v.subtractBy(position);

		return rotateIntoSpaceOf(vec);
	}

	public void set(StateMatrix m)
	{
		position = m.position();
		row0 = m.xRow();
		row1 = m.yRow();
	}

	public void setPosition(float x, float y)
	{
		position.set(x, y);
	}

	public void setXRow(float x, float y)
	{
		row0.set(x, y);
	}

	public void setYRow(float x, float y)
	{
		row1.set(x, y);
	}

	public Vector2 position()
	{
		return position;
	}

	public Vector2 xRow()
	{
		return row0;
	}

	public Vector2 yRow()
	{
		return row1;
	}

}
