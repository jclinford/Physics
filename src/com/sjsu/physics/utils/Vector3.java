package com.sjsu.physics.utils;

/*
 * 3 dimensional vector with appropriate related functions.
 * Uses float precisision for memory conservation.
 */
public class Vector3 
{
	public float x;
	public float y;
	public float z;

	public Vector3()
	{
		x = 0;
		y = 0;
		z = 0;
	}

	public Vector3(float fX, float fY, float fZ)
	{
		x = fX;
		y = fY;
		z = fZ;
	}

	public Vector3(Vector3 v)
	{
		x = v.x();
		y = v.y();
		z = v.z();
	}

	public void set(float fX, float fY, float fZ)
	{
		x = fX;
		y = fY;
		z = fZ;
	}

	public void set (Vector3 v)
	{
		x = v.x();
		y = v.y();
		z = v.z();
	}

	public void setZero()
	{
		x = 0f;
		y = 0f;
		z = 0f;
	}

	public float x()
	{
		return x;
	}

	public float y()
	{
		return y;
	}

	public float z()
	{
		return z;
	}

	public void invert()
	{
		x = -x;
		y = -y;
		z = -z;
	}

	public float magnitude()
	{
		return (float) Math.sqrt(x * x + y * y + z * z);
	}

	/* Mag^2 is a quicker calculation since it doesn't have sqrt */
	public float magnitudeSquared()
	{
		return (x * x + y * y + z * z);
	}

	/* Turns this vector into unit-length 1 */
	public void normalize() 
	{
		float invertedMag = (float) (1.0 / magnitude());
		x = x * invertedMag;
		y = y * invertedMag;
		z = z * invertedMag;
	}

	/* sum vec with this vector */
	public void add(Vector3 vec)
	{
		x = x + vec.x();
		y = y + vec.y();
		z = z + vec.z();
	}

	/* sum this vector with a vector that is scaled by variable scale */
	public void addScaled(Vector3 vec, float scale)
	{
		//		System.out.println("Initial X, Y: " + x + ", " + y + "   vec:  " + vec.x() + ", " + vec.y() + "  scale: " + scale);
		x = x + vec.x() * scale;
		y = y + vec.y() * scale;
		z = z + vec.z() * scale;
	}

	/* subtract vec with this vector */
	public void subtract(Vector3 vec)
	{
		x = x - vec.x();
		y = y - vec.y();
		z = z - vec.z();
	}

	/* multiply a scalar with this vector */
	public void multiply(float scalar)
	{
		x = x * scalar;
		y = y * scalar;
		z = z * scalar;
	}

	/* divide a scalar with this vector */
	public void divide(float scalar)
	{
		x = x / scalar;
		y = y / scalar;
		z = z / scalar;
	}

	/* Return scalar product (dot product) of this and v */
	public float dot(Vector3 v)
	{
		return (x * v.x() + y * v.y() + z * v.z());
	}

	public final static float dot(Vector3 a, Vector3 b) 
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	/* Returns a vector with same values as this vector */
	public Vector3 copy()
	{
		Vector3 copy = new Vector3(this.x, this.y, this.z);
		return copy;
	}

	/* Return the cross product between a and b */
	public final static Vector3 cross(Vector3 a, Vector3 b) 
	{
		return new Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	public final static void crossToOut(Vector3 a, Vector3 b, Vector3 out) 
	{
		final float tempy = a.z * b.x - a.x * b.z;
		final float tempz = a.x * b.y - a.y * b.x;
		out.x = a.y * b.z - a.z * b.y;
		out.y = tempy;
		out.z = tempz;
	}

	public final static void crossToOutUnsafe(Vector3 a, Vector3 b, Vector3 out)
	{
		assert(out != b);
		assert(out != a);
		out.x = a.y * b.z - a.z * b.y;
		out.y = a.z * b.x - a.x * b.z;
		out.z = a.x * b.y - a.y * b.x;
	}
}

