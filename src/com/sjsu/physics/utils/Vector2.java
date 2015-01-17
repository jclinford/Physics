package com.sjsu.physics.utils;

/**
 * Two dimensional vector with appropriate related functions.
 * Uses float precisision for memory conservation.
 */
public class Vector2
{
	public float x;
	public float y;

	public Vector2()
	{
		x = 0;
		y = 0;
	}

	public Vector2(float fX, float fY)
	{
		x = fX;
		y = fY;
	}

	public Vector2(Vector2 v)
	{
		x = v.x();
		y = v.y();
	}

	public void set(float fX, float fY)
	{
		x = fX;
		y = fY;
	}

	public void set(Vector2 v)
	{
		x = v.x();
		y = v.y();
	}

	public float x()
	{
		return x;
	}

	public float y()
	{
		return y;
	}

	public Vector2 invert()
	{
		return new Vector2(-x, -y);
	}

	public float magnitude()
	{
		return (float) Math.sqrt(x * x + y * y);
	}

	/* Mag^2 is a quicker calculation since it doesn't have sqrt */
	public float magnitudeSquared()
	{
		return (x * x + y * y);
	}

	/* Turns this vector into unit-length 1 */
	public Vector2 normalize()
	{
		float invertedMag = (float) (1.0 / magnitude());
		return new Vector2(x * invertedMag, y * invertedMag);
	}

	/* sum vec with this vector and return result */
	public Vector2 addTo(Vector2 vec)
	{
		return new Vector2(x + vec.x(), y + vec.y());
	}

	public Vector2 addTo(float vx, float vy)
	{
		return new Vector2(x + vx, y + vy);
	}

	/* sum this vector with a vector that is scaled by variable scale */
	public Vector2 addToScaled(Vector2 vec, float scale)
	{
		return new Vector2(x + (vec.x() * scale), y + (vec.y() * scale));
	}

	/* subtract vec with this vector */
	public Vector2 subtractBy(Vector2 vec)
	{
		return new Vector2(x - vec.x(), y - vec.y());
	}

	public Vector2 subtractBy(float vx, float vy)
	{
		return new Vector2(x - vx, y - vy);
	}

	/* multiply a scalar with this vector */
	public Vector2 multiplyBy(float scalar)
	{
		return new Vector2(x * scalar, y * scalar);
	}

	/* divide a scalar with this vector */
	public Vector2 divideBy(float scalar)
	{
		return new Vector2(x / scalar, y / scalar);
	}

	/* Returns the angle this vector makes with respect to the x axis */
	public float angle()
	{
		return ((float) Math.atan2(y, x));
	}

	/* returns the distance from this point to another point P */
	public float distanceTo(Vector2 p)
	{
		double dist = (Math.pow((this.x() - p.x()), 2) + Math.pow((this.y() - p.y()), 2));
		return ((float) (Math.sqrt(dist)));
	}

	/**
	 * Project this vector onto line segment AB
	 */
	public Vector2 projectPointOntoEdge(Vector2 a, Vector2 b)
	{
		// vector from edge to point
		Vector2 v = this.subtractBy(a);

		// edge vector
		Vector2 e = b.subtractBy(a);

		// time along edge
		float t = e.dot(v) / e.magnitudeSquared();

		// clamp to edge bounds
		t = Math.min(Math.max(t, 0), 1);

		// form point and return
		return a.addTo(e.multiplyBy(t));
	}

	/* returns the midpoint of this vector to another vector */
	public Vector2 midPoint(Vector2 b)
	{
		float x = (this.x() + b.x()) / 2;
		float y = (this.y() + b.y()) / 2;

		return new Vector2(x, y);
	}

	/* Return the minimum distance between line segment AB and this point */
	public float minimumDistanceToLine(Vector2 a, Vector2 b)
	{
		Vector2 closestPoint = this.projectPointOntoEdge(a, b);
		return this.distanceTo(closestPoint);
	}

	/**
	 * Returns the projection of this vector onto vector B (Ab = A dot
	 * Bnormalized)
	 */
	public Vector2 project(Vector2 b)
	{
		float dp = this.dot(b);

		float projX = (dp / (b.x() * b.x() + b.y() * b.y())) * b.x();
		float projY = (dp / (b.x() * b.x() + b.y() * b.y())) * b.y();

		return new Vector2(projX, projY);
	}

	/* returns the rotation of this vector around it's origin by rad radians */
	public Vector2 rotate(float d)
	{
		double cs = Math.cos(d);
		double sn = Math.sin(d);

		float px = (float) (this.x() * cs - this.y() * sn);
		float py = (float) (this.x() * sn + this.y() * cs);

		return new Vector2(px, py);
	}

	/* rotate this vector by 90 degrees */
	public Vector2 rotate90()
	{
		return new Vector2(-this.y(), this.x());
	}

	/* rotate this vector by 270 degrees */
	public Vector2 rotate270()
	{
		return new Vector2(this.y(), -this.x());
	}

	/* Return scalar product (dot product) of this and v */
	public float dot(Vector2 v)
	{
		return (x * v.x() + y * v.y());
	}

	public final static float dot(Vector2 a, Vector2 b)
	{
		return a.x * b.x + a.y * b.y;
	}

	public float cross(Vector2 v)
	{
		return (x * v.y() - y * v.x());
	}

	/* makes a vector out of an angle */
	public final static Vector2 fromAngle(float a)
	{
		return new Vector2((float) Math.cos(a), (float) Math.sin(a));
	}

	/* Returns a deep copy vector with same values as this vector */
	public Vector2 getCopy()
	{
		Vector2 copy = new Vector2(this.x, this.y);
		return copy;
	}

	@Override
	public String toString()
	{
		String s = "(" + x + ", " + y + ")";
		return s;
	}
}
