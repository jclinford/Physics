package com.sjsu.physics.core;

import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

/**
 * A contact between two rigid bodies
 * 
 * Contains all the necessary information to resolve a collision
 * between two bodies
 */
public class Contact
{
	private RigidBody a;
	private RigidBody b;
	private Vector2 contactPoint;
	private Vector2 contactNormal;
	private float restitution;
	private float penetration;

	public Contact(RigidBody bodyA, RigidBody bodyB, float r, float p)
	{
		a = bodyA;
		b = bodyB;
		restitution = r;
		penetration = p;
	}

	/** Resolve the contact (angular and linear components) */
	public void resolve(float dt)
	{
		resolveVelocity(dt);
		resolvePenetration(dt);
	}

	/** Resolve the contact for duration t */
	private void resolveVelocity(float dt)
	{
		// f the objects are already moving apart (Vs > 0), we do not need to resolve
		float velAlongNorm = velocityAlongNormal();
		if (velAlongNorm > 0)
			return;

		// If we have an infinite mass system, impulses have no affect, no need to resolve
		float sumInverseMass = a.inverseMass() + b.inverseMass();
		if (sumInverseMass <= 0)
			return;

		// calc impulse scalar
		float j = -(1 + restitution) * velAlongNorm;
		j = j / (a.inverseMass() + b.inverseMass());
		Vector2 impulse = contactNormal.multiplyBy(j);

		// Apply impulses to rigid bodies
		Vector2 iA = impulse.multiplyBy(a.inverseMass());
		Vector2 iB = impulse.multiplyBy(b.inverseMass());
		Vector2 Va = a.velocity().subtractBy(iA);
		Vector2 Vb = b.velocity().addTo(iB);

		if (Va.magnitudeSquared() < Globals.SLEEP_EPSILON)
			Va = Globals.ZERO_VECTOR;
		if (Vb.magnitudeSquared() < Globals.SLEEP_EPSILON)
			Vb = Globals.ZERO_VECTOR;

		a.setVelocity(Va);
		b.setVelocity(Vb);

		// calculate angular velocity
		Vector2 aToContact = a.center().subtractBy(contactPoint);
		Vector2 bToContact = b.center().subtractBy(contactPoint);

		// impulsive torque = impulse cross relativeContactPos
		float impulsiveTorqueA = aToContact.cross(impulse);
		float impulsiveTorqueB = -bToContact.cross(impulse);

		// take into account body mass helps realism on rotation
		impulsiveTorqueA *= (10 * a.inverseMass());
		impulsiveTorqueB *= (10 * b.inverseMass());

		if (impulsiveTorqueA < Globals.EPSILON && impulsiveTorqueA > -Globals.EPSILON)
			impulsiveTorqueA = 0;
		if (impulsiveTorqueB < Globals.EPSILON && impulsiveTorqueB > -Globals.EPSILON)
			impulsiveTorqueB = 0;

		a.addAngularVelocity(contactNormal.cross(a.inverseMoment()) * impulsiveTorqueA);
		b.addAngularVelocity(contactNormal.cross(b.inverseMoment()) * impulsiveTorqueB);
	}

	/**
	 * Resolve the penetration between the two objects by moving the objects
	 * apart in proportion to their mass (ie massive objects move less, light
	 * objects move more
	 */
	private void resolvePenetration(float t)
	{
		// If no penetration, just return
		if (penetration <= 0.0f)
			return;

		float sumInverseMass = a.inverseMass() + b.inverseMass();

		// If we have an infinite mass system, no need to resolve
		if (sumInverseMass <= 0)
			return;

		/* Distance needed to move per invese mass */
		Vector2 distPerIMass = contactNormal.multiplyBy(penetration / sumInverseMass);

		a.setCenter(distPerIMass.multiplyBy(a.inverseMass()).addTo(a.center()));
		b.setCenter(distPerIMass.multiplyBy(b.inverseMass()).addTo(b.center()));
	}

	/** Calculate the velocity at which the two objects are moving apart */
	protected float velocityAlongNormal()
	{
		// Relative velocity = Va - Vb
		Vector2 Vrelative = b.velocity().subtractBy(a.velocity());

		// Separting velocity = Vr dot N
		return Vrelative.dot(contactNormal);
	}

	public void setRestitution(float r)
	{
		restitution = r;
	}

	public void setPenetration(float p)
	{
		penetration = p;
	}

	public void setNormal(Vector2 v)
	{
		contactNormal = v;
	}

	public void setContactPoint(Vector2 p)
	{
		contactPoint = p;
	}

	public Vector2 normal()
	{
		return contactNormal;
	}

	public Vector2 contactPoint()
	{
		return contactPoint;
	}

	public RigidBody a()
	{
		return a;
	}

	public RigidBody b()
	{
		return b;
	}

	public float restitution()
	{
		return restitution;
	}

	public float penetration()
	{
		return penetration;
	}

	@Override
	public String toString()
	{
		String s = "ContactPoint: " + contactPoint + "  contactNormal: " + contactNormal
				+ "  penetration: " + penetration;
		return s;
	}
}