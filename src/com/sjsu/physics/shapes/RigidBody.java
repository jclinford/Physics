package com.sjsu.physics.shapes;

import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.StateMatrix;
import com.sjsu.physics.utils.Vector2;

/**
 * A general rigid body object to be inherited from.
 */
public abstract class RigidBody
{
	// Bouding rect fully contains polygon
	protected BoundingBox bounds;

	// Contains position, orientation
	private StateMatrix state;

	private Vector2 velocity;
	private Vector2 acceleration;
	private Vector2 netForce;
	private float angularVelocity;
	private float angularAcceleration;
	private float netTorque;

	protected Vector2 inverseMomentOfInertia;
	private float inverseMass;
	private float damping;
	private float angularDamping;

	private boolean isAwake;
	private boolean canSleep;
	private float motion;

	private BodyType type;
	private int id;
	private int process;
	public int depth;

	public RigidBody()
	{
		// Set everything to defaults..
		bounds = new BoundingBox(Globals.DEFAULT_CIRCLE_RADIUS, Globals.DEFAULT_CIRCLE_RADIUS);
		state = new StateMatrix(new Vector2(0, 0), 0);
		velocity = new Vector2();
		acceleration = new Vector2();
		netForce = new Vector2();
		damping = 1;

		netTorque = 0;
		angularVelocity = 0;
		angularAcceleration = 0;
		angularDamping = 1;
		inverseMomentOfInertia = new Vector2(0, 0);

		type = BodyType.RIGIDBODY;
		isAwake = true;
		canSleep = false;
		motion = 0;

		setMass(Globals.DEFAULT_MASS);
	}

	/** Rigid body types */
	public enum BodyType
	{
		RIGIDBODY, CIRCLE, POLYGON;
	}

	/** Recalculate the moment of inertia */
	public abstract void calculateMoment();

	/** Update function for this rigid body Essentially updates the pos & velocity each frame for time t */
	public final void update(float dt)
	{
		if (dt < 0.0)
			throw new IndexOutOfBoundsException("Trying to update for negative time value");

		// Update linear acceleartion / velocity
		acceleration = acceleration.addToScaled(netForce, inverseMass);
		velocity = velocity.addToScaled(acceleration, dt).multiplyBy((float) Math.pow(damping, dt));
		if (velocity.magnitudeSquared() > 0)
			setCenter(center().addToScaled(velocity, dt));

		// Update angular acceleration and velocity
		float newAngAcc = netTorque + angularAcceleration;
		angularVelocity += newAngAcc * dt;
		angularVelocity = angularVelocity * (float) Math.pow(angularDamping, dt);

		// Update orientation
		if (angularVelocity != 0)
			rotateBy(angularVelocity * dt);

		// Remove previous forces
		clearForces();

		// Update the kinetic energy store, and possibly put the body to sleep.
		if (canSleep)
		{
			float curMotion = velocity.dot(velocity) + angularVelocity;

			float bias = (float) Math.pow(0.5, dt);
			motion = bias * motion + (1 - bias) * curMotion;

			if (motion < Globals.SLEEP_EPSILON)
				setAwake(false);
			else if (motion > 10 * Globals.SLEEP_EPSILON)
				motion = 10 * Globals.SLEEP_EPSILON;
		}
	}

	/** Rotate state matrix by rad radians */
	public void rotateBy(float rad)
	{
		Vector2 row0 = state.xRow().rotate(rad);
		Vector2 row1 = row0.rotate90();

		state.setXRow(row0.x(), row0.y());
		state.setYRow(row1.x(), row1.y());
	}

	/** Clears all forces and torques that are being applied to this object */
	private void clearForces()
	{
		netForce.set(0, 0);
		netTorque = 0;
	}

	/** Add a force to object */
	public final void addForce(Vector2 F)
	{
		// no need to apply to infinite mass objs
		if (inverseMass == 0)
			return;

		netForce = netForce.addTo(F);
		isAwake = true;
	}

	/** Add a torque to object */
	public final void addTorque(float T)
	{
		// No need to apply to infinite moments objs
		if (inverseMomentOfInertia.magnitudeSquared() == 0)
			return;

		netTorque += T;
		isAwake = true;
	}

	public final void setId(int i)
	{
		id = i;
	}

	public final void setProcess(int p)
	{
		process = p;
	}

	public final void setVelocity(float x, float y)
	{
		velocity.set(x, y);
	}

	public final void setVelocity(Vector2 v)
	{
		velocity.set(v.x(), v.y());
	}

	public final void addVelocity(Vector2 v)
	{
		velocity.set(velocity.x() + v.x(), velocity.y() + v.y());
	}

	public final void setAcceleration(Vector2 a)
	{
		acceleration.set(a.x(), a.y());
	}

	public final void addAcceleration(Vector2 a)
	{
		acceleration.set(acceleration.x() + a.x(), acceleration.y() + a.y());
	}

	public final void setAngularVelocity(float w)
	{
		angularVelocity = w;
	}

	public final void addAngularVelocity(float w)
	{
		angularVelocity += w;
	}

	public final void setAngularAccel(float wp)
	{
		angularAcceleration = wp;
	}

	public final void addAngularAccel(float wp)
	{
		angularAcceleration += wp;
	}

	public final void setMass(float m)
	{
		if (m >= Globals.INFINITY)
			inverseMass = 0;
		else
			inverseMass = 1 / m;

		// recalc the moment
		calculateMoment();
	}

	public final void setInverseMass(float im)
	{
		inverseMass = im;

		// recalc moment
		calculateMoment();
	}

	public final void setDamping(float d)
	{
		damping = d;
	}

	public final void setAngularDamping(float dw)
	{
		angularDamping = dw;
	}

	public final void setType(BodyType t)
	{
		type = t;
	}

	public final void setCenter(Vector2 cen)
	{
		state.setPosition(cen.x(), cen.y());
	}

	public final void setCenter(float X, float Y)
	{
		state.setPosition(X, Y);
	}

	/** TODO not implemented yet */
	public final void setAwake(boolean awake)
	{
		if (awake)
		{
			isAwake = true;

			// Add a bit of motion to avoid it falling asleep immediately.
			setVelocity(Globals.EPSILON, Globals.EPSILON);
		} else
		{
			isAwake = false;
			setVelocity(0, 0);
			this.setAngularVelocity(0);
		}
	}

	/** TODO not implemented */
	public final void setCanSleep(boolean sleep)
	{
		canSleep = sleep;

		if (!canSleep && !isAwake)
			setAwake(true);
	}

	public final Vector2 center()
	{
		return state.position();
	}

	/** Returns the angle row0 makes with the x axis */
	public final float orientation()
	{
		float o = state.xRow().angle();
		return o;
	}

	public final int id()
	{
		return id;
	}
	public final BodyType type()
	{
		return type;
	}
	public final int process()
	{
		return process;
	}
	public final Vector2 velocity()
	{
		return velocity;
	}
	public final Vector2 acceleration()
	{
		return acceleration;
	}
	public final float angularVelocity()
	{
		return angularVelocity;
	}
	public final float mass()
	{
		return (1 / inverseMass);
	}
	public final float inverseMass()
	{
		return inverseMass;
	}
	public final Vector2 inverseMoment()
	{
		return inverseMomentOfInertia;
	}
	public final StateMatrix state()
	{
		return state;
	}
	public final float damping()
	{
		return damping;
	}
	public final float angularDamping()
	{
		return angularDamping;
	}
	public final BoundingBox bounds()
	{
		return bounds;
	}
	public final boolean isAwake()
	{
		return isAwake;
	}
	public final boolean hasInfiniteMass()
	{
		if (inverseMass == 0)
			return true;
		else
			return false;
	}
}