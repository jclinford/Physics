package com.sjsu.physics.core;

import java.awt.Rectangle;
import java.util.ArrayList;

import com.sjsu.physics.collisiondetection.QuadTreeNode;
import com.sjsu.physics.shapes.*;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

/* A physics world keeps track of all
 * particles within its world, and provides a means
 * to update them in parallel. Also does necessary
 * parallel procedures like sync'ing the threads,
 * starting or stopping, etc
 */
public class World 
{
	private int bodyCount;
	private static ArrayList<PhysicsThread> threads;
	
	private static QuadTreeNode worldRootNode;					// This node is the root node for the entire world (all objects)
	
	public World()
	{
		bodyCount = 0;
		worldRootNode = new QuadTreeNode().init(null, Globals.GAME_RECT, 0);
		threads = new ArrayList<PhysicsThread>();
		
		switch(Globals.NUM_PROCESSORS)
		{
		// single core is responsible for entire world
		case 1:
			PhysicsThread thread = new PhysicsThread(this, 0, worldRootNode);
			threads.add(thread);
			break;
			
		// quad core, each thread gets its own tree root
		case 4:
			for (int i = 0; i < Globals.NUM_PROCESSORS; i++)
			{
				PhysicsThread t = new PhysicsThread(this, i, worldRootNode.children()[i]);
				t.setPriority(1);
				threads.add(t);
			}
			break;
		default:
			System.out.println("Unsupported Processor count");
		}
	}
	
	public void startThreads()
	{
		for (int i = 0; i < threads.size(); i++)
		{
			threads.get(i).start();
		}
	}
	
	
	/* Physics loop. Runs all threads to do everything */
	public void physicsStep()
	{
		/* Clear all objects from all the physics children's quadtrees */
		worldRootNode.clearObjects();
		
		for (int i = 0; i < threads.size(); i++)
			threads.get(i).run();
		
		
		// Wait for all threads to finish
		int threadFinishCount = 0;
		while (threadFinishCount < threads.size())
		{
			threadFinishCount = 0;
			for (int i = 0; i < threads.size(); i++)
			{
				if (threads.get(i).isProcessing() == false)
					threadFinishCount++;
			}
		}
	}
	
	
	/* Determine which physics thread it should be in based on geometry,
	 * then add body to that thread */
	public void addBodyToWorld(RigidBody b)
	{
		Vector2 center = b.center();
		Rectangle bounds;
		
		/* If the body is outside of our world then we do not add it */
		if (!Globals.GAME_RECT.contains(b.center().x(), b.center().y()))
		{
//			System.out.println("Object has gone out of bounds, cannot add object");
			return;
		}
		
		for (int i = 0; i < threads.size(); i++)
		{
			bounds = threads.get(i).treeRoot().bounds();
			if (bounds.contains(center.x(), center.y()))
			{
				threads.get(i).insertBody(b);
				b.setProcess(i);
				b.setId(bodyCount++);
				return;
			}
		}
		
		/* if we can't find a process for it, just put it into process1 */
		threads.get(0).insertBody(b);
		b.setProcess(0);
		b.setId(bodyCount++);		
		return;
	}
	
	/* Find the thread the body is located in and remove it.. costly.. */
	public void removeBodyFromWorld(RigidBody b)
	{
		Vector2 center = b.center();
		Rectangle bounds;
		
		for (int i = 0; i < threads.size(); i++)
		{
			bounds = threads.get(i).treeRoot().bounds();
			if (bounds.contains(center.x(), center.y()))
				threads.get(i).removeBody(b);
		}
		
		bodyCount--;
	}
	
	/* Return all bodies in every thread and this rootNode */
	public static ArrayList<RigidBody> allBodies()
	{
		ArrayList<RigidBody> allBodies = new ArrayList<RigidBody>(Globals.DEFAULT_BODY_SIZE * Globals.NUM_PROCESSORS);
		
		for (int i = 0; i < threads.size(); i++)
		{
			// Add all thread bodies
			allBodies.addAll(threads.get(i).bodies());
		}
		
		return allBodies;
	}
	
	/* Returns the thread # n */
	public static PhysicsThread getThread(int n)
	{
		if (n > Globals.NUM_PROCESSORS)
		{
			System.out.println("Invalid thread num");
			return null;
		}
		
		PhysicsThread t = threads.get(n);
		return t;
	}
	
	/* Return the bodies that belong solely to the world node..
	 * that is bodies that are overlapping two or more processors */
	public ArrayList<RigidBody> overlappingBodies()
	{
		return worldRootNode.bodies();
	}
	
	public static QuadTreeNode getTreeRoot()
	{
		return worldRootNode;
	}
}
