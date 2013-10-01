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
	private float time;							// time of contact
	private Vector2 contactPoint;
	private Vector2 contactNormal;
	private float restitution;					// Coefficient of Restitution (1 = perfect elastic, 0 = inelastic)
	private float penetration;					// How much the two objects are penetrating one another
	private Vector2 particleMovement[];			// tracks how much the particle moved due to penetration
	private Vector2 relativeContactPosition[];	// position of the contact point from each body's center
	private Vector2 impulse;					// the estimated change in v that we will see after this contact is resolved
	private Vector2 contactVelocity;			// Velocity relative to the contact point
	
	public Contact(RigidBody bodyA, RigidBody bodyB, float r, float p)
	{
		a = bodyA;
		b = bodyB;
		restitution = r;
		penetration = p;
		particleMovement = new Vector2[2];
		relativeContactPosition = new Vector2[2];
		impulse = new Vector2(0, 0);
	}
	
	/** Resolve the contact (angular and linear components) */
	public void resolve(float dt)
	{		
		// update internal data before doing calculations
		updateInternals(dt);
		resolvePenetration(dt);
		resolveVelocity();
//		resolveAngular();
//		resolveVelocity(dt);
	}
	
	/** Resolve the linear portion of the impulse calculated in updateInternals */
	private void resolveVelocity()
	{
		Vector2 impulseWorld = impulse.addTo(contactPoint);
		float impulsiveTorqueA = relativeContactPosition[0].cross(impulseWorld);
		float impulsiveTorqueB = relativeContactPosition[1].cross(impulseWorld);
		
		a.addVelocity(impulseWorld.multiplyBy(a.inverseMass()));
		a.addAngularVelocity(contactNormal.dot(a.inverseMoment()) * impulsiveTorqueA);
		
		b.addVelocity(impulseWorld.multiplyBy(b.inverseMass()));
		b.addAngularVelocity(contactNormal.dot(b.inverseMoment()) * impulsiveTorqueB);
	}
	
	/** Resolve the angular component of the impulse calculated in updateInternals */
	private void resolveAngular()
	{
		// impulsive torque = impulse cross relativeContactPos
		float impulsiveTorqueA = - relativeContactPosition[0].cross(impulse);
		float impulsiveTorqueB = relativeContactPosition[1].cross(impulse);
		
		if (impulsiveTorqueA < Globals.EPSILON && impulsiveTorqueA > -Globals.EPSILON)
			impulsiveTorqueA = 0;
		if (impulsiveTorqueB < Globals.EPSILON && impulsiveTorqueB > -Globals.EPSILON)
			impulsiveTorqueB = 0;
				
		a.addAngularVelocity(contactNormal.dot(a.inverseMoment()) * impulsiveTorqueA);
		b.addAngularVelocity(contactNormal.dot(b.inverseMoment()) * impulsiveTorqueB);
	}
	
	/** Calculate the velocity local to the contact point */
	private Vector2 calculateLocalVelocity(RigidBody body, float dt)
	{
		Vector2 relPos;
		if (a.equals(body))
		{
			relPos = relativeContactPosition[0];
		}
		else
		{
			relPos = relativeContactPosition[1];
		}
		
		// TODO probably goin wrong here...
		Vector2 velocity = relPos.multiplyBy(body.angularVelocity()).addTo(body.velocity());
		float vMag = velocity.magnitude();
		
		// turn into contact coords
		Vector2 localVelocity = contactNormal.multiplyBy(vMag);
		
		// calc amount of velocity that is due to forces w/out reactions
		Vector2 accVelocity = body.acceleration().multiplyBy(dt);
		float accVMag = accVelocity.magnitude();
		// ignore x component along contact normal
		accVelocity.set(0, accVMag * contactNormal.y());
		
		localVelocity = localVelocity.addTo(accVelocity);
				
		return localVelocity;
	}
	
	/** Caulculates the desired change in velocity for this contact */
	private float calculateDesiredDeltaV(float dt)
	{
		float velocityLimit = .25f;
		float contactVAlongNorm = contactVelocity.dot(contactNormal);
		
		// calc accel induced velocity this frame
		float velocityFromAccel = 0;
		
		velocityFromAccel += a.acceleration().multiplyBy(dt).dot(contactNormal);
		velocityFromAccel -= b.acceleration().multiplyBy(dt).dot(contactNormal);
		
		// if velocity is very slow, limit restitution
		float thisRestitution = restitution;
		if (Math.abs(contactVAlongNorm) < velocityLimit)
			thisRestitution = 0.0f;
				
		return (-contactVAlongNorm - 
				thisRestitution * (contactVAlongNorm - velocityFromAccel));
	}
	
	/** Calculate the impulse associated with this contact */
	private void calculateImpulse(float desiredDeltaV)
	{			
		float deltaV = 0;
		
		// get vPerImpulse for rotational aspect
		// (Center - ContactPoint) cross contactNormal
		float deltaVA = relativeContactPosition[0].cross(contactNormal);
		deltaVA *= a.inverseMoment().dot(contactNormal);
		
		// add linear component
		deltaVA += a.inverseMass();
		deltaV += deltaVA;
		
		// repeat for body b
		float deltaVB = relativeContactPosition[1].cross(contactNormal);
		deltaVB *= b.inverseMoment().dot(contactNormal);
		
		// add linear component for b
		deltaVB += b.inverseMass();
		deltaV += deltaVB;
		
		
		// impulse without friction = desiredDeltaV / deltaV along contact normal
		float impulseScalar = (desiredDeltaV / deltaV);
		
		// rotate to contact coords
		impulse = contactNormal.multiplyBy(impulseScalar);
		
		System.out.println("Impulse1: " + impulse);
	}
	
	/** Update internal data */
	private void updateInternals(float dt)
	{
		// relative position of contact to each body [0] = a, [1] = b
		relativeContactPosition[0] = contactPoint.subtractBy(a.center());
		relativeContactPosition[1] = contactPoint.subtractBy(b.center());
		
		// relative velocity of bodies at contactPoint
		// the x position will be the velocity along the contactPoint
		// the y position is the "slide"
		contactVelocity = calculateLocalVelocity(a, dt);
		contactVelocity = contactVelocity.subtractBy(calculateLocalVelocity(b, dt));
		
		// calculate the desired change in velocity needed for resolution
		float desiredDeltaV = calculateDesiredDeltaV(dt);
		
		// calculate the impulse
		calculateImpulse(desiredDeltaV);
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
		
//		System.out.println("normal: " + contactNormal + "   penetration: " + penetration + "   sumIMass: " + sumInverseMass + "  velAlongNorm: " + velAlongNorm);
//		System.out.println("Vai: " + a.velocity() + "   iMa: " + a.inverseMass() + "   Vbi: " + b.velocity() + "    iMb: " + b.inverseMass());
		
		/* Apply impulses to rigid bodies */
		Vector2 iA = impulse.multiplyBy(a.inverseMass());
		Vector2 iB = impulse.multiplyBy(b.inverseMass());
		Vector2 Va = a.velocity().subtractBy(iA);
		Vector2 Vb = b.velocity().addTo(iB);
		
		if (Va.magnitudeSquared() < Globals.SLEEP_EPSILON)
			Va = new Vector2(0, 0);
		if (Vb.magnitudeSquared() < Globals.SLEEP_EPSILON)
			Vb = new Vector2(0, 0);
		
		a.setVelocity(Va);
		b.setVelocity(Vb);
		
		
		// calculate angular velocity
//		Vector2 momentArmA = contactPoint.subtractBy(a.center());
//		Vector2 momentArmB = contactPoint.subtractBy(b.center());
//		float tmpA = (impulse.dot(momentArmA)) / (momentArmA.dot(momentArmA));
//		float tmpB = (impulse.dot(momentArmB)) / (momentArmB.dot(momentArmB));
//		Vector2 parallelCompA = momentArmA.multiplyBy(tmpA);
//		Vector2 parallelCompB = momentArmB.multiplyBy(tmpB);
//		Vector2 angularForceA = impulse.subtractBy(parallelCompA);
//		Vector2 angularForceB = impulse.subtractBy(parallelCompB);
//		Vector2 torqueA = angularForceA.multiplyBy(momentArmA.magnitude());
//		Vector2 torqueB = angularForceB.multiplyBy(momentArmB.magnitude());
		
		
		// calculate angular velocity
		Vector2 aToContact = contactPoint.subtractBy(a.center());
		Vector2 bToContact = contactPoint.subtractBy(b.center());
		float torqueA = aToContact.x() * impulse.y() - aToContact.y() * impulse.x();
		float torqueB = bToContact.x() * impulse.y() - bToContact.y() * impulse.x();
		float angAccelA = torqueA * (a.inverseMoment().dot(aToContact));
		float angAccelB = torqueB * (b.inverseMoment().dot(bToContact));
		
//		if (torqueA < Globals.SLEEP_EPSILON && torqueA > -Globals.SLEEP_EPSILON)
//			torqueA = 0;
//		if (torqueB < Globals.SLEEP_EPSILON && torqueA > -Globals.SLEEP_EPSILON)
//			torqueB = 0;
		
//		System.out.println("TorqueA: " + torqueA + "   B: " + torqueB);
//		a.addTorque(angAccelA);		// not really torque, angAcceleration		
//		b.addTorque(angAccelB);
				
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
			System.out.println("No penetration, not resolvin");
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
		
		/* Set positions of bodies to correct for penetration ; newCenter = center + (distPerIMass * inverseMass) */
		particleMovement[0] = distPerIMass.multiplyBy(a.inverseMass());
		particleMovement[1] = distPerIMass.multiplyBy(b.inverseMass());
		
//		System.out.println("normal: " + contactNormal.x() + ", " + contactNormal.y() + "   penetration: " + penetration + 
//				"   sumIMass: " + sumInverseMass + "   distA: " + particleMovement[0] + "  distB: " + particleMovement[1]);
		
		a.setCenter(particleMovement[0].addTo(a.center()));
		b.setCenter(particleMovement[1].addTo(b.center()));
		
//		System.out.println("A :" + a + "   B: " + b);
	}
	
	/** Calculate the velocity at which the two objects are moving apart */
	protected float velocityAlongNormal()
	{
		// Relative velocity = Va - Vb
		Vector2 Vrelative = b.velocity().subtractBy(a.velocity());
		
		// Separting velocity = Vr dot N
		return Vrelative.dot(contactNormal);
	}
	
	/** Calculate the velocity along the perpindicular of the contact normal */
	private float velocityAlongPerp()
	{
		// relative velocity
		Vector2 Vrelative = b.velocity().subtractBy(a.velocity());
		
		return Vrelative.dot(contactNormal.rotate90());
	}
	
	
	
	public void setTime(float t)
	{
		time = t;
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
	
	public void setParticleMovement(RigidBody body, Vector2 move)
	{
		if (body.equals(a))
			particleMovement[0] = move;
		else
			particleMovement[1] = move;
	}
	
	public Vector2 particleMovement(RigidBody body)
	{
		if (body.equals(a))
			return particleMovement[0];
		else
			return particleMovement[1];
	}
	
	public float time()
	{
		return time;
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
}