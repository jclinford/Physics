package com.sjsu.physics.collisiondetection;

import java.awt.Rectangle;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.utils.Globals;

//TODO currently just a point-quadtree.. need to account for geometries and check if the rigidBody is overlapping two or more quadrants.. if so add it to the parent node
//Continue physics book on pg 257 (280/481)

/* 
 * A node of a quad-tree, built for parallel processing.
 * Similar to a Binary tree, but with four children.
 * 
 * 
 * If there is overlap, the object will be placed with the parent node's borderBodies.
 *  So if an object is overlapping two processor's boundaries, it will be placed
 * in the worldRoot node. Each processor can then check for collisions solely 
 * within it's own children and also against the worldRootNode's children.
 * 
 * 
 * 							  World (All bodies)
 * 									  |
 * 			Process1		Process2	Process3		Process4
 * 				|				|			|				|				
 * 			Children		Children	Children		Children
 */

public class QuadTreeNode 
{	
	private Rectangle bounds;
	private int depth;
	
	private ArrayList<RigidBody> myBodies;			// Bodies that belong to this node
	private QuadTreeNode parent;
	private QuadTreeNode[] children;				// Children nodes of this node
	
	public QuadTreeNode()
	{
		// Use init method for recursive initiation
	}
	
	/* Recursively init the quadtree */
	public QuadTreeNode init(QuadTreeNode par, Rectangle grid, int d)
	{
		bounds = grid;
		depth = d;
		
		myBodies = new ArrayList<RigidBody>(Globals.MAX_QUADTREE_CHILDREN);
		children = new QuadTreeNode[4];
		parent = par;
		
		if (Globals.MAX_QUADTREE_DEPTH > depth)
			this.subDivide();
		
		return this;
	}
	
	/* Subdivide a node into four subnodes */
	private void subDivide()
	{
		if (depth > Globals.MAX_QUADTREE_DEPTH)
			return;
		
		int bX = bounds.x;
		int bY = bounds.y;
		
		int bwh = (int) Math.floor((bounds.width / 2));
		int bhh = (int) Math.floor((bounds.height / 2));
		int bXbwh = bX + bwh;
		int bYbhh = bY + bhh;
		
		/* Rect's for new node bounds */
		Rectangle topLeftBounds = new Rectangle(bX, bY, bwh, bhh);
		Rectangle topRightBounds = new Rectangle(bXbwh, bY, bwh, bhh);
		Rectangle bottomLeftBounds = new Rectangle(bX, bYbhh, bwh, bhh);
		Rectangle bottomRightBounds = new Rectangle(bXbwh, bYbhh, bwh, bhh);
		
		/* Create the new Nodes */
		children[Globals.TOP_LEFT] = new QuadTreeNode().init(this, topLeftBounds, depth + 1);
		children[Globals.TOP_RIGHT] = new QuadTreeNode().init(this, topRightBounds, depth + 1);
		children[Globals.BOTTOM_RIGHT] = new QuadTreeNode().init(this, bottomRightBounds, depth + 1);
		children[Globals.BOTTOM_LEFT] = new QuadTreeNode().init(this, bottomLeftBounds, depth + 1);
	}
	
	
	
	
	
	/* Return our children / subnodes */
	public QuadTreeNode[] children()
	{
		return children;
	}
	
	/* Return the bounds that this Node occupies */
	public Rectangle bounds()
	{
		return bounds;
	}
	
	/* Return only myBodies (objects that reside fully in the bounds of the node) */
	public ArrayList<RigidBody> bodies()
	{
		return myBodies;
	}
	
	
	/* Test whether a rigidBody is fully contained inside this node's bounds
	 * if the node can fully contain the body then it returns true, else false 
	 * TODO can I be more efficient here??*/
	public boolean contains(RigidBody body)
	{	
		Rectangle2D.Float boundingRect = new Rectangle2D.Float(body.center().x() - body.bounds().halfWidth(),
				body.center().y() - body.bounds().halfHeight(), body.bounds().halfWidth() * 2, body.bounds().halfHeight() * 2);
		if (bounds.contains(boundingRect))
			return true;
		
		return false;
	}
	
	/* Retrieve the node that the rigidBody belongs to. If not this node, search myBodies */
	public ArrayList<RigidBody> retrieveNode(RigidBody body)
	{
		// If we have subnodes, find the index
		if (!isLeaf())
		{
			int index = findIndex(body);
			return children[index].retrieveNode(body);
		}
		
		// if we are at the base node already, return this node's children and this node's borderBodies
//		System.out.println("Retrieving, have total nodes: " + myBodies.size() + "   have total border: " + borderBodies.size());
		return myBodies;
	}
	
	/* Insert a body to our node. If it's full split our node */
	public void insert(RigidBody body)
	{
		boolean added = true;
		
		// If we have subnodes, find the index our body belongs to and insert it there
		if (!isLeaf())
		{
			// Find which quadrant the center is in
			int index = findIndex(body);
			
			// Make sure the quadrant can fully contain the body's geometry
			if (children[index].contains(body))
			{
				children[index].insert(body);
				return;
			}
			// If the child cannot fully contain the body we will add it to this node
			else
				added = false;
		}
		// if we have no children we will add it to this node
		else
			added = false;
		
		if (!added)
		{
			myBodies.add(body);
			added = true;
		}
	}
	
	/* Finds the index that a body belongs to based on center point */
	private int findIndex(RigidBody body)
	{
		int index;
		boolean left = (body.center().x() > (bounds.x + bounds.width / 2)) ? false : true;
		boolean top = (body.center().y() > (bounds.y + bounds.height / 2)) ? false : true;
		
		if (left)
		{
			if (top)
				index = Globals.TOP_LEFT;
			else
				index = Globals.BOTTOM_LEFT;
		}
		else
		{
			if (top)
				index = Globals.TOP_RIGHT;
			else
				index = Globals.BOTTOM_RIGHT;
		}
		
		return index;
	}
	
	/* Returns whether this node is a leaf or not */
	public boolean isLeaf()
	{
		// If we do not have children allocated, then we are a leaf
		if (children[0] == null || children[1] == null || children[2] == null || children[3] == null)
			return true;
		
		// If we have children allocated, but all of our children don't have bodies then we are a leaf
		if (children[0].bodies().size() < 1 && children[1].bodies().size() < 1 && 
				children[2].bodies().size() < 1 && children[3].bodies().size() < 1)
			return true;
		
		return false;
	}
	
	/* Clear all bodies this this node and subnodes.. does not unallocate the node memory
	 * nor does it unallocate or mess with the init'd tree structure */
	public void clearObjects()
	{
		myBodies.clear();
		
		for (int i = 0; i < 4; i++)
		{
			if (children[i] != null)
				children[i].clearObjects();
		}
	}
}
