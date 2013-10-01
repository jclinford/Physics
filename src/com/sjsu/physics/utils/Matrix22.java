package com.sjsu.physics.utils;

import java.io.Serializable;

/**
 * A 2-by-2 matrix. Stored in column-major order.
 */
public class Matrix22 implements Serializable 
{
	private static final long SERIAL_VERSION_UID = 2L;

	private Vector2 ex;
	private Vector2 ey;


	/** Convert the matrix to printable format. */
	@Override
	public String toString() 
	{
		String s = "";
		s += "[" + ex.x() + "," + ey.x() + "]\n";
		s += "[" + ex.y() + "," + ey.y() + "]";
		return s;
	}

	/**
	 * Construct zero matrix. Note: this is NOT an identity matrix!
	 */
	public Matrix22() 
	{
		ex = new Vector2();
		ey = new Vector2();
	}

	/**
	 * Create a matrix with given vectors as columns.
	 * 
	 * @param c1 Column 1 of matrix
	 * @param c2 Column 2 of matrix
	 */
	public Matrix22(final Vector2 c1, final Vector2 c2) 
	{
		ex = c1.getCopy();
		ey = c2.getCopy();
	}

	/**
	 * Create a matrix from four floats.
	 * 
	 * @param exx
	 * @param col2x
	 * @param exy
	 * @param col2y
	 */
	public Matrix22(final float exx, final float col2x, final float exy, final float col2y) 
	{
		ex = new Vector2(exx, exy);
		ey = new Vector2(col2x, col2y);
	}

	/**
	 * Set as a copy of another matrix.
	 * 
	 * @param m Matrix to copy
	 */
	public final Matrix22 set(final Matrix22 m)
	{
		ex.set(m.ex);
		ey.set(m.ey);
		return this;
	}

	public final Matrix22 set(final float exx, final float col2x, final float exy, final float col2y) 
	{
		ex.set(exx, exy);
		ey.set(col2x, col2y);
		return this;
	}

	/**
	 * Return a clone of this matrix
	 */
	public final Matrix22 copy() 
	{
		return new Matrix22(ex, ey);
	}

	/**
	 * Set as a matrix representing a rotation.
	 * 
	 * @param angle Rotation (in radians) that matrix represents.
	 */
	public final void set(final float angle)
	{
		final float c = (float) Math.cos(angle), s = (float) Math.sin(angle);
		ex.set(c, s);
		ey.set(-s, c);
	}

	/**
	 * Set as the identity matrix.
	 */
	public final void setIdentity() 
	{
		ex.set(1.0f, 1.0f);
		ey.set(1.0f, 1.0f);
	}

	/**
	 * Set as the zero matrix.
	 */
	public final void setZero() 
	{
		ex.set(0f, 0f);
		ey.set(0f, 0f);
	}

	/**
	 * Extract the angle from this matrix (assumed to be a rotation matrix).
	 * 
	 * @return
	 */
	public final float getAngle() 
	{
		return (float) Math.atan2(ex.y(), ex.x());
	}

	/**
	 * Set by column vectors.
	 * 
	 * @param c1 Column 1
	 * @param c2 Column 2
	 */
	public final void set(final Vector2 c1, final Vector2 c2) 
	{
		ex.set(c1);
		ey.set(c2);
	}

	/** Returns the inverted Matrix22 - does NOT invert the matrix locally! */
	public final Matrix22 invert() 
	{
		final float a = ex.x(), b = ey.x(), c = ex.y(), d = ey.y();
		final Matrix22 B = new Matrix22();
		float det = a * d - b * c;

		if (det != 0)
			det = 1.0f / det;

		B.ex.set(det * d, -det * c);
		B.ey.set(-det * b, det * a);
		return B;
	}

	public final Matrix22 invertLocal() 
	{
		final float a = ex.x(), b = ey.x(), c = ex.y(), d = ey.y();
		float det = a * d - b * c;
		
		if (det != 0)
			det = 1.0f / det;

		ex.set(det * d, -det * c);
		ey.set(-det * b, det * a);
		return this;
	}


	/**
	 * Return the matrix composed of the absolute values of all elements.
	 * 
	 * @return Absolute value matrix
	 */
	public final Matrix22 abs() 
	{
		return new Matrix22(Math.abs(ex.x()), Math.abs(ey.x()), Math.abs(ex.y()),
				Math.abs(ey.y()));
	}

	/**
	 * Return the matrix composed of the absolute values of all elements.
	 * 
	 * @return Absolute value matrix
	 */
	public final static Matrix22 abs(final Matrix22 R)
	{
		return R.abs();
	}

	/* djm created */
	public static void absToOut(final Matrix22 R, final Matrix22 out)
	{
		out.ex.set(Math.abs(R.ex.x()), Math.abs(R.ex.y()));
		out.ey.set(Math.abs(R.ey.x()), Math.abs(R.ey.y()));
	}

	/**
	 * Multiply a vector by this matrix.
	 * 
	 * @param v Vector to multiply by matrix.
	 * @return Resulting vector
	 */
	public final Vector2 mul(final Vector2 v)
	{
		return new Vector2(ex.x() * v.x() + ey.x() * v.y(), ex.y() * v.x() + ey.y() * v.y());
	}

	public final void mulToOut(final Vector2 v, final Vector2 out)
	{
		final float tempy = ex.y() * v.x() + ey.y() * v.y();
		out.set(ex.x() * v.x() + ey.x() * v.y(), tempy);
	}

	public final void mulToOutUnsafe(final Vector2 v, final Vector2 out) 
	{
		assert (v != out);
		out.set((ex.x() * v.x() + ey.x() * v.y()), (ex.y() * v.x() + ey.y() * v.y()));
	}


	/**
	 * Multiply another matrix by this one (this one on left). djm optimized
	 * 
	 * @param R
	 * @return
	 */
	public final Matrix22 mul(final Matrix22 R) 
	{
		final Matrix22 C = new Matrix22();
		C.ex.set((ex.x() * R.ex.x() + ey.x() * R.ex.y()), (ex.y() * R.ex.x() + ey.y() * R.ex.y()));
		C.ey.set((ex.x() * R.ey.x() + ey.x() * R.ey.y()), (ex.y() * R.ey.x() + ey.y() * R.ey.y()));
		return C;
	}

	public final Matrix22 mulLocal(final Matrix22 R) 
	{
		mulToOut(R, this);
		return this;
	}

	public final void mulToOut(final Matrix22 R, final Matrix22 out) 
	{
		final float tempy1 = this.ex.y() * R.ex.x() + this.ey.y() * R.ex.y();
		final float tempx1 = this.ex.x() * R.ex.x() + this.ey.x() * R.ex.y();
		out.ex.set(tempx1, tempy1);

		final float tempy2 = this.ex.y() * R.ey.x() + this.ey.y() * R.ey.y();
		final float tempx2 = this.ex.x() * R.ey.x() + this.ey.x() * R.ey.y();
		out.ey.set(tempx2, tempy2);
	}

	public final void mulToOutUnsafe(final Matrix22 R, final Matrix22 out) 
	{
		assert (out != R);
		assert (out != this);
		
		final float tempy1 = this.ex.y() * R.ex.x() + this.ey.y() * R.ex.y();
		final float tempx1 = this.ex.x() * R.ex.x() + this.ey.x() * R.ex.y();
		out.ex.set(tempx1, tempy1);

		final float tempy2 = this.ex.y() * R.ey.x() + this.ey.y() * R.ey.y();
		final float tempx2 = this.ex.x() * R.ey.x() + this.ey.x() * R.ey.y();
		out.ey.set(tempx2, tempy2);
	}

	/**
	 * Multiply another matrix by the transpose of this one (transpose of this one on left). 
	 * 
	 * @param B
	 * @return
	 */
	public final Matrix22 mulTrans(final Matrix22 B) 
	{
		final Matrix22 C = new Matrix22();

		C.ex.x = ex.dot(B.ex);
		C.ex.y = ey.dot(B.ex);

		C.ey.x = ex.dot(B.ey);
		C.ey.y = ey.dot(B.ey);
		return C;
	}

	public final Matrix22 mulTransLocal(final Matrix22 B) {
		mulTransToOut(B, this);
		return this;
	}

	public final void mulTransToOut(final Matrix22 B, final Matrix22 out) {
		/*
		 * out.ex.x = Vector2.dot(this.ex, B.ex); out.ex.y = Vector2.dot(this.ey, B.ex); out.ey.x =
		 * Vector2.dot(this.ex, B.ey); out.ey.y = Vector2.dot(this.ey, B.ey);
		 */
		final float x1 = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
		final float y1 = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
		final float x2 = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
		final float y2 = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
		out.ex.x = x1;
		out.ey.x = x2;
		out.ex.y = y1;
		out.ey.y = y2;
	}

	public final void mulTransToOutUnsafe(final Matrix22 B, final Matrix22 out) {
		assert (B != out);
		assert (this != out);
		out.ex.x = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
		out.ey.x = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
		out.ex.y = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
		out.ey.y = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
	}

	/**
	 * Multiply a vector by the transpose of this matrix.
	 * 
	 * @param v
	 * @return
	 */
	public final Vector2 mulTrans(final Vector2 v) {
		// return new Vector2(Vector2.dot(v, ex), Vector2.dot(v, col2));
		return new Vector2((v.x * ex.x + v.y * ex.y), (v.x * ey.x + v.y * ey.y));
	}

	/* djm added */
	public final void mulTransToOut(final Vector2 v, final Vector2 out) {
		/*
		 * out.x = Vector2.dot(v, ex); out.y = Vector2.dot(v, col2);
		 */
		final float tempx = v.x * ex.x + v.y * ex.y;
		out.y = v.x * ey.x + v.y * ey.y;
		out.x = tempx;
	}

	/**
	 * Add this matrix to B, return the result.
	 * 
	 * @param B
	 * @return
	 */
	public final Matrix22 add(final Matrix22 B) {
		// return new Matrix22(ex.add(B.ex), col2.add(B.ey));
		Matrix22 m = new Matrix22();
		m.ex.x = ex.x + B.ex.x;
		m.ex.y = ex.y + B.ex.y;
		m.ey.x = ey.x + B.ey.x;
		m.ey.y = ey.y + B.ey.y;
		return m;
	}

	/**
	 * Add B to this matrix locally.
	 * 
	 * @param B
	 * @return
	 */
	public final Matrix22 addLocal(final Matrix22 B) {
		// ex.addLocal(B.ex);
		// col2.addLocal(B.ey);
		ex.x += B.ex.x;
		ex.y += B.ex.y;
		ey.x += B.ey.x;
		ey.y += B.ey.y;
		return this;
	}

	/**
	 * Solve A * x = b where A = this matrix.
	 * 
	 * @return The vector x that solves the above equation.
	 */
	public final Vector2 solve(final Vector2 b) {
		final float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		float det = a11 * a22 - a12 * a21;
		if (det != 0.0f) {
			det = 1.0f / det;
		}
		final Vector2 x = new Vector2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
		return x;
	}

	public final void solveToOut(final Vector2 b, final Vector2 out) {
		final float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		float det = a11 * a22 - a12 * a21;
		if (det != 0.0f) {
			det = 1.0f / det;
		}
		final float tempy = det * (a11 * b.y - a21 * b.x);
		out.x = det * (a22 * b.x - a12 * b.y);
		out.y = tempy;
	}

	public final static Vector2 mul(final Matrix22 R, final Vector2 v) {
		// return R.mul(v);
		return new Vector2(R.ex.x * v.x + R.ey.x * v.y, R.ex.y * v.x + R.ey.y * v.y);
	}

	public final static void mulToOut(final Matrix22 R, final Vector2 v, final Vector2 out) {
		final float tempy = R.ex.y * v.x + R.ey.y * v.y;
		out.x = R.ex.x * v.x + R.ey.x * v.y;
		out.y = tempy;
	}

	public final static void mulToOutUnsafe(final Matrix22 R, final Vector2 v, final Vector2 out) {
		assert (v != out);
		out.x = R.ex.x * v.x + R.ey.x * v.y;
		out.y = R.ex.y * v.x + R.ey.y * v.y;
	}

	public final static Matrix22 mul(final Matrix22 A, final Matrix22 B) {
		// return A.mul(B);
		final Matrix22 C = new Matrix22();
		C.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
		C.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
		C.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
		C.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
		return C;
	}

	public final static void mulToOut(final Matrix22 A, final Matrix22 B, final Matrix22 out) {
		final float tempy1 = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
		final float tempx1 = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
		final float tempy2 = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
		final float tempx2 = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
		out.ex.x = tempx1;
		out.ex.y = tempy1;
		out.ey.x = tempx2;
		out.ey.y = tempy2;
	}

	public final static void mulToOutUnsafe(final Matrix22 A, final Matrix22 B, final Matrix22 out) {
		assert (out != A);
		assert (out != B);
		out.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
		out.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
		out.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
		out.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
	}

	public final static Vector2 mulTrans(final Matrix22 R, final Vector2 v) {
		return new Vector2((v.x * R.ex.x + v.y * R.ex.y), (v.x * R.ey.x + v.y * R.ey.y));
	}

	public final static void mulTransToOut(final Matrix22 R, final Vector2 v, final Vector2 out) {
		float outx = v.x * R.ex.x + v.y * R.ex.y;
		out.y = v.x * R.ey.x + v.y * R.ey.y;
		out.x = outx;
	}

	public final static void mulTransToOutUnsafe(final Matrix22 R, final Vector2 v, final Vector2 out) {
		assert (out != v);
		out.y = v.x * R.ey.x + v.y * R.ey.y;
		out.x = v.x * R.ex.x + v.y * R.ex.y;
	}

	public final static Matrix22 mulTrans(final Matrix22 A, final Matrix22 B) {
		final Matrix22 C = new Matrix22();
		C.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
		C.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
		C.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
		C.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
		return C;
	}

	public final static void mulTransToOut(final Matrix22 A, final Matrix22 B, final Matrix22 out) {
		final float x1 = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
		final float y1 = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
		final float x2 = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
		final float y2 = A.ey.x * B.ey.x + A.ey.y * B.ey.y;

		out.ex.x = x1;
		out.ex.y = y1;
		out.ey.x = x2;
		out.ey.y = y2;
	}

	public final static void mulTransToOutUnsafe(final Matrix22 A, final Matrix22 B, final Matrix22 out) {
		assert (A != out);
		assert (B != out);
		out.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
		out.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
		out.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
		out.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
	}

	public final static Matrix22 createRotationalTransform(float angle) {
		Matrix22 mat = new Matrix22();
		final float c = (float) Math.cos(angle);
		final float s = (float) Math.sin(angle);
		mat.ex.x = c;
		mat.ey.x = -s;
		mat.ex.y = s;
		mat.ey.y = c;
		return mat;
	}

	public final static void createRotationalTransform(float angle, Matrix22 out) {
		final float c = (float) Math.cos(angle);
		final float s = (float) Math.sin(angle);
		out.ex.x = c;
		out.ey.x = -s;
		out.ex.y = s;
		out.ey.y = c;
	}

	public final static Matrix22 createScaleTransform(float scale) {
		Matrix22 mat = new Matrix22();
		mat.ex.x = scale;
		mat.ey.y = scale;
		return mat;
	}

	public final static void createScaleTransform(float scale, Matrix22 out) {
		out.ex.x = scale;
		out.ey.y = scale;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((ex == null) ? 0 : ex.hashCode());
		result = prime * result + ((ey == null) ? 0 : ey.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj) return true;
		if (obj == null) return false;
		if (getClass() != obj.getClass()) return false;
		Matrix22 other = (Matrix22) obj;
		if (ex == null) {
			if (other.ex != null) return false;
		} else if (!ex.equals(other.ex)) return false;
		if (ey == null) {
			if (other.ey != null) return false;
		} else if (!ey.equals(other.ey)) return false;
		return true;
	}
}