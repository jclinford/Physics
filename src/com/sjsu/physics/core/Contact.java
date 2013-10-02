package com.sjsu.physics.core;

import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

/*
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
	private float restitution;					// Coefficient of Restitution (1 = perfect elastic, 0 = inelastic)
	private float penetration;					// How much the two objects are penetrating one another
	
	
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
		/* if the objects are already moving apart (Vs > 0), we do not need to resolve */
		float velAlongNorm = velocityAlongNormal();
		if (velAlongNorm > 0)
		{
//			System.out.println("Objects moving away, not resolving velocity. Separating v: " + velAlongNorm + "   v1:" + a.velocity() + "    v2:" + b.velocity());
			return;
		}
		
		/* If we have an infinite mass system, impulses have no affect, no need to resolve */
		float sumInverseMass = a.inverseMass() + b.inverseMass();
		if (sumInverseMass <= 0)
		{
			System.out.println("Warning: Inverse mass, returning");
			return;
		}

		// initial momentum
//		float pi = (a.mass() * a.velocity().magnitude()) + (b.mass() * b.velocity().magnitude());
		
		// calc impulse scalar
		float j = - (1 + restitution) * velAlongNorm;
		j = j / (a.inverseMass() + b.inverseMass());
		Vector2 impulse = contactNormal.multiplyBy(j);

		/* Apply impulses to rigid bodies */
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
		float impulsiveTorqueB = - bToContact.cross(impulse);
		
		// take into account body mass helps realism on rotation
		impulsiveTorqueA *= (10 * a.inverseMass());
		impulsiveTorqueB *= (10 * b.inverseMass());
		
		if (impulsiveTorqueA < Globals.EPSILON && impulsiveTorqueA > -Globals.EPSILON)
			impulsiveTorqueA = 0;
		if (impulsiveTorqueB < Globals.EPSILON && impulsiveTorqueB > -Globals.EPSILON)
			impulsiveTorqueB = 0;
				
		a.addAngularVelocity(contactNormal.cross(a.inverseMoment()) * impulsiveTorqueA);
		b.addAngularVelocity(contactNormal.cross(b.inverseMoment()) * impulsiveTorqueB);

		// final momentum
//		float pf = (a.mass() * a.velocity().magnitude()) + (b.mass() * b.velocity().magnitude());
//		System.out.println("Vaf: " + Va + "   Vbf: " + Vb + "    initial momentum: " + pi + "   final momentum: " + pf);
	}
	
	/** 
	 * Resolve the penetration between the two objects by moving the objects apart
	 * in proportion to their mass (ie massive objects move less, light objects move more
	 */
	private void resolvePenetration(float t)
	{
		/* If no penetration, just return */
		if (penetration <= 0.0f)
		{
//			System.out.println("No penetration, not resolvin");
			return;
		}
				
		float sumInverseMass = a.inverseMass() + b.inverseMass();
		
		/* If we have an infinite mass system, no need to resolve */
		if (sumInverseMass <= 0)
		{
			System.out.println("Warning, infinite mass system! No contact resolving will happen");
			return;
		}
		
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
		String s = "ContactPoint: " + contactPoint + "  contactNormal: " + contactNormal + "  penetration: " + penetration;
		return s;
	}
}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	// Old code doesn't really work well, but more exact
///** Calculate the velocity along the perpindicular of the contact normal */
//private float velocityAlongPerp()
//{
//	// relative velocity
//	Vector2 Vrelative = b.velocity().subtractBy(a.velocity());
//	
//	return Vrelative.dot(contactNormal.rotate90());
//}
//	
//	/** Resolve the linear portion of the impulse calculated in updateInternals */
//	private void resolveVelocity()
//	{
//		Vector2 impulseWorld = impulse.addTo(contactPoint);
//		float impulsiveTorqueA = relativeContactPosition[0].cross(impulseWorld);
//		float impulsiveTorqueB = relativeContactPosition[1].cross(impulseWorld);
//		
//		a.addVelocity(impulseWorld.multiplyBy(a.inverseMass()));
//		a.addAngularVelocity(contactNormal.dot(a.inverseMoment()) * impulsiveTorqueA);
//		
//		b.addVelocity(impulseWorld.multiplyBy(b.inverseMass()));
//		b.addAngularVelocity(contactNormal.dot(b.inverseMoment()) * impulsiveTorqueB);
//	}
//	
//	/** Resolve the angular component of the impulse calculated in updateInternals */
//	private void resolveAngular()
//	{
//		// impulsive torque = impulse cross relativeContactPos
//		float impulsiveTorqueA = - relativeContactPosition[0].cross(impulse);
//		float impulsiveTorqueB = relativeContactPosition[1].cross(impulse);
//		
//		if (impulsiveTorqueA < Globals.EPSILON && impulsiveTorqueA > -Globals.EPSILON)
//			impulsiveTorqueA = 0;
//		if (impulsiveTorqueB < Globals.EPSILON && impulsiveTorqueB > -Globals.EPSILON)
//			impulsiveTorqueB = 0;
//				
//		a.addAngularVelocity(contactNormal.dot(a.inverseMoment()) * impulsiveTorqueA);
//		b.addAngularVelocity(contactNormal.dot(b.inverseMoment()) * impulsiveTorqueB);
//	}
//	
//	/** Calculate the velocity local to the contact point */
//	private Vector2 calculateLocalVelocity(RigidBody body, float dt)
//	{
//		Vector2 relPos;
//		if (a.equals(body))
//		{
//			relPos = relativeContactPosition[0];
//		}
//		else
//		{
//			relPos = relativeContactPosition[1];
//		}
//		
//		// TODO probably goin wrong here...
//		Vector2 velocity = relPos.multiplyBy(body.angularVelocity()).addTo(body.velocity());
//		float vMag = velocity.magnitude();
//		
//		// turn into contact coords
//		Vector2 localVelocity = contactNormal.multiplyBy(vMag);
//		
//		// calc amount of velocity that is due to forces w/out reactions
//		Vector2 accVelocity = body.acceleration().multiplyBy(dt);
//		float accVMag = accVelocity.magnitude();
//		// ignore x component along contact normal
//		accVelocity.set(0, accVMag * contactNormal.y());
//		
//		localVelocity = localVelocity.addTo(accVelocity);
//				
//		return localVelocity;
//	}
//	
//	/** Caulculates the desired change in velocity for this contact */
//	private float calculateDesiredDeltaV(float dt)
//	{
//		float velocityLimit = .25f;
//		float contactVAlongNorm = contactVelocity.dot(contactNormal);
//		
//		// calc accel induced velocity this frame
//		float velocityFromAccel = 0;
//		
//		velocityFromAccel += a.acceleration().multiplyBy(dt).dot(contactNormal);
//		velocityFromAccel -= b.acceleration().multiplyBy(dt).dot(contactNormal);
//		
//		// if velocity is very slow, limit restitution
//		float thisRestitution = restitution;
//		if (Math.abs(contactVAlongNorm) < velocityLimit)
//			thisRestitution = 0.0f;
//				
//		return (-contactVAlongNorm - 
//				thisRestitution * (contactVAlongNorm - velocityFromAccel));
//	}
//	
//	/** Calculate the impulse associated with this contact */
//	private void calculateImpulse(float desiredDeltaV)
//	{			
//		float deltaV = 0;
//		
//		// get vPerImpulse for rotational aspect
//		// (Center - ContactPoint) cross contactNormal
//		float deltaVA = relativeContactPosition[0].cross(contactNormal);
//		deltaVA *= a.inverseMoment().dot(contactNormal);
//		
//		// add linear component
//		deltaVA += a.inverseMass();
//		deltaV += deltaVA;
//		
//		// repeat for body b
//		float deltaVB = relativeContactPosition[1].cross(contactNormal);
//		deltaVB *= b.inverseMoment().dot(contactNormal);
//		
//		// add linear component for b
//		deltaVB += b.inverseMass();
//		deltaV += deltaVB;
//		
//		
//		// impulse without friction = desiredDeltaV / deltaV along contact normal
//		float impulseScalar = (desiredDeltaV / deltaV);
//		
//		// rotate to contact coords
//		impulse = contactNormal.multiplyBy(impulseScalar);
//		
//		System.out.println("Impulse1: " + impulse);
//	}
//	
//	/** Update internal data */
//	private void updateInternals(float dt)
//	{
//		// relative position of contact to each body [0] = a, [1] = b
//		relativeContactPosition[0] = contactPoint.subtractBy(a.center());
//		relativeContactPosition[1] = contactPoint.subtractBy(b.center());
//		
//		// relative velocity of bodies at contactPoint
//		// the x position will be the velocity along the contactPoint
//		// the y position is the "slide"
//		contactVelocity = calculateLocalVelocity(a, dt);
//		contactVelocity = contactVelocity.subtractBy(calculateLocalVelocity(b, dt));
//		
//		// calculate the desired change in velocity needed for resolution
//		float desiredDeltaV = calculateDesiredDeltaV(dt);
//		
//		// calculate the impulse
//		calculateImpulse(desiredDeltaV);
//	}