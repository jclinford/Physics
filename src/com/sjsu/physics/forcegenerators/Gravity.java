package com.sjsu.physics.forcegenerators;

import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

/*
 * Gravity force defined as F = mg
 */
public class Gravity implements Force
{
	private static Vector2 gravity;

	/* Set the default to earth gravity */
	public Gravity()
	{
		gravity = new Vector2(0, -Globals.g);
	}

	public Gravity(Vector2 g)
	{
		gravity = g;
	}

	public void setGravity(Vector2 g)
	{
		gravity = g;
	}

	public Vector2 gravity()
	{
		return gravity;
	}

	/* Apply gravity to the rigid body */
	@Override
	public void updateForce(RigidBody body, float t)
	{
		/* Don't apply forces to bodies w/ infinite mass */
		if (body.hasInfiniteMass())
			return;

		/* F = ma = mg */
		Vector2 gF = new Vector2(gravity).multiplyBy(body.mass());
		body.addForce(gF);
	}

}
