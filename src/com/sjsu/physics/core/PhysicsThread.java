package com.sjsu.physics.core;

import java.util.ArrayList;
import java.util.LinkedList;

import com.sjsu.physics.collisiondetection.FineCollision;
import com.sjsu.physics.collisiondetection.QuadTreeNode;
import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

/* A single thread to perform core physics routines in parallel */
public class PhysicsThread extends Thread
{
	private int pNum; // Specific processor number

	private QuadTreeNode myTreeRoot; // Quadtree associated with this processor;
	private ArrayList<RigidBody> myBodies; // Bodies that belong to this thread
	private LinkedList<RigidBody> newBodies;// New bodies that were added in
											// between a game loop
	public ArrayList<Contact> contacts; // A list of contacts that need to be
										// resolved

	private ContactSolver contactSolver; // Solves the contacts for this thread

	private World myWorld; // The world the controls this thread
	private boolean isProcessing; // Flag to denote whether this thread is
									// currently processing

	public PhysicsThread(World w, int p, QuadTreeNode root)
	{
		isProcessing = false;
		myWorld = w;
		pNum = p;
		myTreeRoot = root;
		myBodies = new ArrayList<RigidBody>(Globals.DEFAULT_BODY_SIZE);
		newBodies = new LinkedList<RigidBody>();
		contacts = new ArrayList<Contact>(Globals.DEFAULT_BODY_SIZE / 2);

		contactSolver = new ContactSolver(Globals.CONTACT_SOLVER_DEFAULT_ITERATIONS);
	}

	// We are no longer overiding this thread's ride function.. make sure that
	// is okay for parallel processing.
	// Must I call run in order for threads to run concurrently??
	@Override
	public void run()
	{
		long timeStep = 0;

		// While loop to do all necessary tasks, once run returns the thread
		// will die
		while (true)
		{
			// Get the time that has passed
			long startTime = System.nanoTime();
			// System.out.println("Pnum: " + pNum + "   CurTime: " + startTime +
			// "  timeStep: " + timeStep);

			//System.out.println("Pnum: " + pNum + "  bodies: " + myBodies.size());

			// Take a step forward in time
			integrate(timeStep * Globals.NANOSEC_TO_SECONDS * 5);

			// Generate contacts / check for collisions
			generateContacts();

			// Resolve contacts
			resolveContacts(timeStep * Globals.NANOSEC_TO_SECONDS * 5);

			// Transfer all newly added bodies to our body list
			refreshBodyList();

			timeStep = (System.nanoTime() - startTime);

			// System.out.println("Finished while loop pnum: " + pNum + " timestep: " + timeStep);
		}
	}

	/**
	 * Integrate all of the bodies that belong to this processor forward by time
	 * t.
	 * 
	 * @param dt
	 *            Time to step forward
	 */
	public void integrate(float dt)
	{
		RigidBody body;
		for (int i = 0; i < myBodies.size(); i++)
		{
			body = myBodies.get(i);
			body.update(dt);

			// After we update the body we need to make sure its still in this processor's bounds
			// If its not we remove it from this processor and reinsert into the main world
			Vector2 center = body.center();
			if (!myTreeRoot.bounds().contains(center.x(), center.y()))
			{
				myTreeRoot.removeBody(body);
				myBodies.remove(i);
				
				myWorld.addBodyToWorld(body);
				return;
			}
			
			// if the object moved we need to remove it from quadtree and reinsert
			if (body.velocity().magnitudeSquared() > Globals.EPSILON)
			{
				myTreeRoot.removeBody(body);
				myTreeRoot.insert(body);
			}
		}
	}

	/** Check for collisions and generate contacts if there is a collision */
	public void generateContacts()
	{
		ArrayList<RigidBody> possibleCollisions;

		/* First check for collisions against all our own bodies */
		for (int i = 0; i < myBodies.size(); i++)
		{
			// TODO this is probably not the best way.. this will call retrieve
			// a lot of times
			// I should be able to optimize this to call retrieve, check all
			// bodies that it retrieves
			// then move on to the next node. This way every node is only
			// retrieved once
			possibleCollisions = myTreeRoot.getPossibleCollisions(myBodies.get(i));
			if (possibleCollisions.size() < 1)
				continue;

			for (int x = 0; x < possibleCollisions.size(); x++)
			{
				if (possibleCollisions.get(x).id() == myBodies.get(i).id())
					continue;

				// check for collisions. If there is a collision it will be added to contacts list
				FineCollision.getContactPoints(myBodies.get(i), possibleCollisions.get(x), contacts);
			}
		}
	}

	/** Resolve any contacts in our list. To be called after generateContacts */
	public void resolveContacts(float dt)
	{
		if (contacts.size() > 0)
			contactSolver.resolveContacts(contacts, dt);
	}

	/**
	 * Called by world. Inserts a rigid body into a waiting list, will be added
	 * to the full list at the end of the loop
	 */
	protected void insertBody(RigidBody b)
	{
		// System.out.println("Body added to process: " + pNum);
		newBodies.add(b);
	}

	/** Place newly added bodies into the arrayList */
	private void refreshBodyList()
	{
		while (newBodies.size() > 0)
		{
			RigidBody b = newBodies.removeFirst();
			myBodies.add(b);
			myTreeRoot.insert(b);
		}
	}

	/**
	 * Insert all of myBodies into the processor's tree
	 */
	protected void insertMyBodiesToTree()
	{
		for (int i = 0; i < myBodies.size(); i++)
		{
			myTreeRoot.insert(myBodies.get(i));
		}
	}

	/** 
	 * Remove a body from the world
	 * @param b The body to remove
	 */
	protected void removeBody(RigidBody b)
	{
		myTreeRoot.removeBody(b);
		myBodies.remove(b);
	}


	/* Clear all bodies from this processor */
	protected void clearBodies()
	{
		myBodies.clear();
	}

	/* return all bodies that belong to this thread */
	public ArrayList<RigidBody> bodies()
	{
		return myBodies;
	}

	public QuadTreeNode treeRoot()
	{
		return myTreeRoot;
	}

	public boolean isProcessing()
	{
		return isProcessing;
	}
}
