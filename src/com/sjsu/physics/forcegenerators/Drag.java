package com.sjsu.physics.forcegenerators;

import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.utils.Vector2;

/* 
 * Drag force Fd = 1/2 (p) (v^2) (Cd) (A)
 * Fd = Drag Force
 * p = density of fluid
 * v = velocity of rigid body
 * A = cross-sectional area
 * Cd = drag coefficient
 * 
 * We are simplifying drag here to be Fd = -b (v) where b is some
 * coefficient passed into the Constructor. Not very accurate but greatly
 * reduces complexity with not too much affect on accuracy
 */
public class Drag implements Force
{
	private float b;

	public void Drag()
	{
		b = .5f;
	}

	public void Drag(float coef)
	{
		b = coef;
	}

	public void setCoef(float coef)
	{
		b = coef;
	}

	public float coef()
	{
		return b;
	}

	@Override
	public void updateForce(RigidBody body, float t)
	{
		/* Fd = -b * v */
		Vector2 Fd = body.velocity().normalize();
		float drag = b * Fd.magnitude();
		Fd = Fd.multiplyBy(-drag);

		body.addForce(Fd);
	}

}
