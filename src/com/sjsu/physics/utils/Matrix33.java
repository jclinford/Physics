package com.sjsu.physics.utils;

import java.io.Serializable;

/* A 3x3 matrix class with some
 * standard functions */
public class Matrix33 implements Serializable
{
	private static final long SERIAL_VERSION_UID = 2L;

	public final Vector3 ex, ey, ez;

	public Matrix33()
	{
		ex = new Vector3();
		ey = new Vector3();
		ez = new Vector3();
	}

	public Matrix33(Vector3 argCol1, Vector3 argCol2, Vector3 argCol3)
	{
		ex = argCol1.copy();
		ey = argCol2.copy();
		ez = argCol3.copy();
	}

	public void setZero()
	{
		ex.setZero();
		ey.setZero();
		ez.setZero();
	}

	// / Multiply a matrix times a vector.
	public static final Vector3 mul(Matrix33 A, Vector3 v)
	{
		return new Vector3(v.x * A.ex.x + v.y * A.ey.x + v.z + A.ez.x, v.x * A.ex.y + v.y * A.ey.y
				+ v.z * A.ez.y, v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z);
	}

	public static final Vector2 mul22(Matrix33 A, Vector2 v)
	{
		return new Vector2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
	}

	public static final void mul22ToOut(Matrix33 A, Vector2 v, Vector2 out)
	{
		final float tempx = A.ex.x * v.x + A.ey.x * v.y;
		out.y = A.ex.y * v.x + A.ey.y * v.y;
		out.x = tempx;
	}

	public static final void mul22ToOutUnsafe(Matrix33 A, Vector2 v, Vector2 out)
	{
		assert (v != out);
		out.y = A.ex.y * v.x + A.ey.y * v.y;
		out.x = A.ex.x * v.x + A.ey.x * v.y;
	}

	public static final void mulToOut(Matrix33 A, Vector3 v, Vector3 out)
	{
		final float tempy = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
		final float tempz = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
		out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
		out.y = tempy;
		out.z = tempz;
	}

	public static final void mulToOutUnsafe(Matrix33 A, Vector3 v, Vector3 out)
	{
		assert (out != v);
		out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
		out.y = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
		out.z = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
	}

	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient than
	 * computing the inverse in one-shot cases.
	 * 
	 * @param b
	 * @return
	 */
	public final Vector2 solve22(Vector2 b)
	{
		Vector2 x = new Vector2();
		solve22ToOut(b, x);
		return x;
	}

	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient than
	 * computing the inverse in one-shot cases.
	 * 
	 * @param b
	 * @return
	 */
	public final void solve22ToOut(Vector2 b, Vector2 out)
	{
		final float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		float det = a11 * a22 - a12 * a21;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		out.x = det * (a22 * b.x - a12 * b.y);
		out.y = det * (a11 * b.y - a21 * b.x);
	}

	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient than
	 * computing the inverse in one-shot cases.
	 * 
	 * @param b
	 * @return
	 */
	public final Vector3 solve33(Vector3 b)
	{
		Vector3 x = new Vector3();
		solve33ToOut(b, x);
		return x;
	}

	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient than
	 * computing the inverse in one-shot cases.
	 * 
	 * @param b
	 * @param out
	 *            the result
	 */
	public final void solve33ToOut(Vector3 b, Vector3 out)
	{
		assert (b != out);
		Vector3.crossToOutUnsafe(ey, ez, out);
		float det = Vector3.dot(ex, out);
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		Vector3.crossToOutUnsafe(ey, ez, out);
		final float x = det * Vector3.dot(b, out);
		Vector3.crossToOutUnsafe(b, ez, out);
		final float y = det * Vector3.dot(ex, out);
		Vector3.crossToOutUnsafe(ey, b, out);
		float z = det * Vector3.dot(ex, out);
		out.x = x;
		out.y = y;
		out.z = z;
	}

	public void getInverse22(Matrix33 M)
	{
		float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		float det = a * d - b * c;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}

		M.ex.x = det * d;
		M.ey.x = -det * b;
		M.ex.z = 0.0f;
		M.ex.y = -det * c;
		M.ey.y = det * a;
		M.ey.z = 0.0f;
		M.ez.x = 0.0f;
		M.ez.y = 0.0f;
		M.ez.z = 0.0f;
	}

	// / Returns the zero matrix if singular.
	public void getSymInverse33(Matrix33 M)
	{
		float bx = ey.y * ez.z - ey.z * ez.y;
		float by = ey.z * ez.x - ey.x * ez.z;
		float bz = ey.x * ez.y - ey.y * ez.x;
		float det = ex.x * bx + ex.y * by + ex.z * bz;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}

		float a11 = ex.x, a12 = ey.x, a13 = ez.x;
		float a22 = ey.y, a23 = ez.y;
		float a33 = ez.z;

		M.ex.x = det * (a22 * a33 - a23 * a23);
		M.ex.y = det * (a13 * a23 - a12 * a33);
		M.ex.z = det * (a12 * a23 - a13 * a22);

		M.ey.x = M.ex.y;
		M.ey.y = det * (a11 * a33 - a13 * a13);
		M.ey.z = det * (a13 * a12 - a11 * a23);

		M.ez.x = M.ex.z;
		M.ez.y = M.ey.z;
		M.ez.z = det * (a11 * a22 - a12 * a12);
	}

	@Override
	public int hashCode()
	{
		final int prime = 31;
		int result = 1;
		result = prime * result + ((ex == null) ? 0 : ex.hashCode());
		result = prime * result + ((ey == null) ? 0 : ey.hashCode());
		result = prime * result + ((ez == null) ? 0 : ez.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj)
	{
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Matrix33 other = (Matrix33) obj;
		if (ex == null)
		{
			if (other.ex != null)
				return false;
		} else if (!ex.equals(other.ex))
			return false;
		if (ey == null)
		{
			if (other.ey != null)
				return false;
		} else if (!ey.equals(other.ey))
			return false;
		if (ez == null)
		{
			if (other.ez != null)
				return false;
		} else if (!ez.equals(other.ez))
			return false;
		return true;
	}

	/**
	 * Transform the given direction vector by the transformational inverse of
	 * this matrix.
	 * 
	 * @note This function relies on the fact that the inverse of a pure
	 *       rotation matrix is its transpose. It separates the translational
	 *       and rotation components, transposes the rotation, and multiplies
	 *       out. If the matrix is not a scale and shear free transform matrix,
	 *       then this function will not give correct results.
	 * 
	 * @note When a direction is converted between frames of reference, there is
	 *       no translation required.
	 * 
	 * @param vector
	 *            The vector to transform.
	 */
	public Vector2 transformInverse(final Vector2 vector)
	{
		return new Vector2(vector.x * ex.x() + vector.y * ex.y(), vector.x * ey.x() + vector.y
				* ey.y());
	}
}
