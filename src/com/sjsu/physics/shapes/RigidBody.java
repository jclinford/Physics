package com.sjsu.physics.shapes;

import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.StateMatrix;
import com.sjsu.physics.utils.Vector2;

// 


/* 
 * A general rigid body object to be inherited from.
 */
public abstract class RigidBody 
{
	protected BoundingBox bounds;			// Bouding rect fully contains polygon
	
	private StateMatrix state;				// Contains position, orientation
	private Vector2 velocity;
	private Vector2 acceleration;
	private Vector2 netForce;				// The sum of the forces that are being applied to this object - zero'd every iteration
	private float angularVelocity;			// Angular velocity (w) in radians per second
	private float angularAcceleration;		// angular accell (dw/dt) in radians per second^2
	private float netTorque;				// The sum of the torques that are being applied to this object

	protected Vector2 inverseMomentOfInertia;
	private float inverseMass;				// inverse mass is easier to determine fixed objects, infinite mass, etc
	private float damping;					// 'drag' force, ie energy removed per update
	private float angularDamping;			// 'drag' on angular velocity
	
	private boolean isAwake;				// if false we don't do updates on this
	private boolean canSleep;
	private float motion;					// used for determining if it can sleep
	
	private BodyType type;
	private int id;
	private int process;
	
	public RigidBody()
	{	
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

	
	/* Rigid body types */
	public enum BodyType 
	{
		RIGIDBODY, CIRCLE, POLYGON;
	}
	
	/** Recalculate the moment of inertia */
	public abstract void calculateMoment();
	
	
	/* 
	 * Update function for this rigid body
	 * Essentially updates the pos & velocity each frame
	 * for time t
	 */
	public void update(float dt)
	{
		if (dt < 0.0)
		{
			System.out.println("Warning, trying to update for negative time.");
			return;
		}
		
		// Update linear acceleartion / velocity
		acceleration = acceleration.addToScaled(netForce, inverseMass);
		velocity = velocity.addToScaled(acceleration, dt).multiplyBy((float) Math.pow(damping, dt));
		if (velocity.magnitudeSquared() > 0)
			setCenter(center().addToScaled(velocity, dt));
		
		
		// Update angular acceleration and velocity TODO moment is wrong..
		float newAngAcc = netTorque + angularAcceleration;
		angularVelocity += newAngAcc * dt;
		angularVelocity = angularVelocity * (float)Math.pow(angularDamping, dt);
		
		// Update orientation
		if (angularVelocity != 0)
			rotateBy(angularVelocity * dt);

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
	
	/** Rotate state matrix by rad radians
	 * 
	 * @param rad Radians to rotate by
	 */
	public void rotateBy(float rad)
	{
		Vector2 row0 = state.xRow().rotate(rad);
		Vector2 row1 = row0.rotate90();
		
		state.setXRow(row0.x(), row0.y());
		state.setYRow(row1.x(), row1.y());
	}
	
	/* Clears all forces and torques that are being applied to this object */
	public void clearForces()
	{
		netForce.set(0, 0);
		netTorque = 0;
	}
	
	public void addForce(Vector2 F)
	{
		if (inverseMass == 0)
			return;
		
		netForce = netForce.addTo(F);
		isAwake = true;
	}
	
	public void addTorque(float T)
	{
		if (inverseMomentOfInertia.magnitudeSquared() == 0)
			return;
		
		netTorque += T;
		isAwake = true;
	}
	
	public void setId(int i)
	{
		id = i;
	}
	public void setProcess(int p)
	{
		process = p;
	}
	
	public void setVelocity(float x, float y)
	{
		velocity.set(x, y);
	}
	
	public void setVelocity(Vector2 v)
	{
		velocity.set(v.x(), v.y());
	}
	
	public void addVelocity(Vector2 v)
	{
		velocity.set(velocity.x() + v.x(), velocity.y() + v.y());
	}
	
	public void setAcceleration(Vector2 a)
	{
		acceleration.set(a.x(), a.y());
	}
	
	public void addAcceleration(Vector2 a)
	{
		acceleration.set(acceleration.x() + a.x(), acceleration.y() + a.y());
	}
	
	public void setAngularVelocity(float w)
	{
		angularVelocity = w;
	}
	
	public void addAngularVelocity(float w)
	{
		angularVelocity += w;
	}
	
	public void setAngularAccel(float wp)
	{
		angularAcceleration = wp;
	}
	
	public void addAngularAccel(float wp)
	{
		angularAcceleration += wp;
	}
	
	public void setMass(float m)
	{
		if (m >= Globals.INFINITY)
			inverseMass = 0;
		else
			inverseMass = 1 / m;
		
		// recalc the moment
		calculateMoment();
	}
	
	public void setInverseMass(float im)
	{
		inverseMass = im;
		
		// recalc moment
		calculateMoment();
	}
	
	public void setDamping(float d)
	{
		damping = d;
	}
	
	public void setAngularDamping(float dw)
	{
		angularDamping = dw;
	}
	
	public void setType(BodyType t)
	{
		type = t;
	}
	
	public void setCenter(Vector2 cen)
	{
		state.setPosition(cen.x(), cen.y());
	}
	
	public void setCenter(float X, float Y)
	{
		state.setPosition(X, Y);
	}

	public void setAwake(boolean awake)
	{
		if (awake) 
		{
			isAwake= true;

			// Add a bit of motion to avoid it falling asleep immediately.
			setVelocity(Globals.EPSILON, Globals.EPSILON);
		} 
		else 
		{
			isAwake = false;
			setVelocity(0, 0);
			this.setAngularVelocity(0);
		}
	}
	
	public void setCanSleep(boolean sleep)
	{
		canSleep = sleep;
		
	    if (!canSleep && !isAwake) setAwake(true);
	}
	
	public Vector2 center()
	{
		return state.position();
	}
	
	/* returns the angle row0 makes with the x axis */
	public float orientation()
	{
		float o = state.xRow().angle();
		return o;
	}

	public int id()
	{
		return id;
	}
	
	public BodyType type()
	{
		return type;
	}
	
	public int process()
	{
		return process;
	}
	
	public Vector2 velocity()
	{
		return velocity;
	}
	
	public Vector2 acceleration()
	{
		return acceleration;
	}
	
	public float angularVelocity()
	{
		return angularVelocity;
	}
	
	public float mass()
	{
		return (1 / inverseMass);
	}
	
	public float inverseMass()
	{
		return inverseMass;
	}
	
	public Vector2 inverseMoment()
	{
		return inverseMomentOfInertia;
	}
	
	public StateMatrix state()
	{
		return state;
	}
	
	public float damping()
	{
		return damping;
	}
	
	public float angularDamping()
	{
		return angularDamping;
	}
	
	public BoundingBox bounds()
	{
		return bounds;
	}
	
	public boolean isAwake()
	{
		return isAwake;
	}
	
	public boolean hasInfiniteMass()
	{
		if (inverseMass == 0)
			return true;
		else
			return false;
	}
}